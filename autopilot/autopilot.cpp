/*
 ============================================================================
 Name        : autopilot.c
 Author      : Martin
 Version     :
 Copyright   : (c) 2015 Martin Lane-Smith
 Description : Receives MOVE commands and issues commands to the motors, comparing
 navigation results against the goal
 ============================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stropts.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include <chrono>
using namespace std::chrono;

#include "ps.h"
#include "robot.h"
#include "autopilot_debug.h"
#include "autopilot.hpp"

#include "NavigatorClass.hpp"
#include "BehaviorClass.hpp"
#include "MotorClass.hpp"

FILE *pilotDebugFile;

#define RANDOM_MOVE_SPREAD 500	//mm

#define PILOT_LOOP_MS 				100
#define PILOT_REGISTRY_UPDATE_MS	1000

//BT Leaf
int luaPilotAction(lua_State *L)
{
	PilotAction_enum actionCode 	= (PilotAction_enum) lua_tointeger(L, 1);

	if (actionCode < PILOT_ACTION_COUNT)
	{
		the_behaviors().lastLuaCall = pilotActionList[actionCode];

		DEBUGPRINT("Pilot Action: %s ...", pilotActionList[actionCode]);

		return actionReply(L, the_autopilot_instance().Action(actionCode));
	}
	else
	{
		ERRORPRINT("Pilot Action %i invalid", actionCode);

		return fail(L);
	}
}

void AutopilotHandleEvent(void *arg, ps_event_id_t event)
{
	the_autopilot_instance().HandleEvent(event);
}

Autopilot::Autopilot() {

	pilotDebugFile = fopen_logfile("pilot");

	ps_add_event_observer(BATTERY_SHUTDOWN_EVENT, AutopilotHandleEvent, this);

	//Create thread
	pilot_thread = new std::thread([this](){autopilot_thread_method();});
}

//incoming actions - called from LUA Behavior Tree leaf actions
ActionResult_enum Autopilot::Action(PilotAction_enum _action)
{
	ActionResult_enum result = ACTION_FAIL;

	//critical section
	std::unique_lock<std::mutex> lck {pilotStateMtx};

	switch(_action)
	{
	//simple actions, handled here
	case WaitUntilPilotReady:
		return WaitUntilReadyToMove();
		break;
	case isPilotReady:
		return IsReadyToMove();
		break;
	case ComputeHeelRoute:
		return pilotSetGoalWaypoint((char*)"Heel");
		break;
	case ComputeRandomRoute:
		return pilotSelectRandomWayopint();
		break;
	case ComputeRandomClosePosition:
		return pilotSetRandomGoal(RANDOM_MOVE_SPREAD);
		break;
	case PilotReset:
		//cancel any current operation
		CancelPilotOperation(PILOT_STATE_IDLE);
		pilotEngaged = false;
		return ACTION_SUCCESS;
		break;
	case isAtGoal:
		return pilotIsAtWaypoint();
		break;
	default:
	{
		if (pilotEngaged)
		{
			//monitor on-going operation
			switch(pilotState)
			{
			case PILOT_STATE_ORIENT:		//monitoring motion using compass
			case PILOT_STATE_MOVE:			//monitoring move using pose msg
			case PILOT_STATE_ORIENT_MOVE:
				result = ACTION_RUNNING;
				break;
			case PILOT_STATE_DONE:			//move complete
				pilotEngaged = false;
				result =  ACTION_SUCCESS;
				break;
			case PILOT_STATE_FAILED:		//move failed
				pilotEngaged = false;
				result = ACTION_FAIL;
				break;
			case PILOT_STATE_IDLE:			//ready for a command
			case PILOT_STATE_INACTIVE:		//motors disabled
			default:
				pilotEngaged = false;
				result = ACTION_FAIL;
				the_behaviors().lastLuaCallReason = "Inactive";
				break;
			}
		}
		else if (pilotState != PILOT_STATE_INACTIVE)
		{
			//start new action
			//verify pre-requisites
			switch (_action) {

			case Orient:
				if (VerifyCompass()) {

					if (route_planned)
					{
						//prepare goal
						FindRabbit();
						LogRoutine("Pilot: Orient to: %i", desiredCompassHeading);
						pilotState = PILOT_STATE_ORIENT;
						result = ACTION_RUNNING;
					}
					else
					{
						result = ACTION_FAIL;
						the_behaviors().lastLuaCallReason = "No Route";
					}
				}
				else
				{
					result = ACTION_FAIL;
				}
				break;
			case Engage:
				if (VerifyCompass() && VerifyGPS()) {

					if (route_planned)
					{
						//prepare goal
						FindRabbit();
						LogRoutine("Pilot: Move to: %f, %f", toWaypoint.longitude_in_degrees(), toWaypoint.latitude_in_degrees());
						pilotState = PILOT_STATE_ORIENT_MOVE;
						result = ACTION_RUNNING;
					}
					else
					{
						result = ACTION_FAIL;
						the_behaviors().lastLuaCallReason = "No Route";
					}
				}
				else
				{
					result = ACTION_FAIL;
				}

				break;
			default:
				LogError("Pilot action: %i", _action);
				result = ACTION_FAIL;
				the_behaviors().lastLuaCallReason = "BadCode";
				break;
			}

			if (result == ACTION_RUNNING) pilotEngaged = true;
		}
		else
		{
			result = ACTION_FAIL;
			the_behaviors().lastLuaCallReason = "Inactive";
		}
	}
	break;
	}

	return result;
}


bool Autopilot::CancelPilotOperation(PilotState_enum newState)
{
	//cancel any current operation
	//assumes we already have the mutex
	//returns true if stop sent

	switch(pilotState)
	{
	case PILOT_STATE_ORIENT:
	case PILOT_STATE_ORIENT_MOVE:
	case PILOT_STATE_MOVE:
		//send stop message
		the_motors().cancel_action();
		LogRoutine("Pilot %s Cancelled.", pilotStateNames[pilotState]);
		pilotState = newState;
		return true;			//stop sent
		break;
	default:
		return false;
		break;
	}
}

ActionResult_enum Autopilot::WaitUntilReadyToMove()
{
	//Hard Errors

	if (pilotState == PILOT_STATE_INACTIVE)
	{
		the_behaviors().lastLuaCallReason = "Inactive";
		return ACTION_FAIL;
	}
	else if (!route_planned)
	{
		the_behaviors().lastLuaCallReason = "No Route";
		return ACTION_FAIL;
	}
	else if (!the_motors().is_motor_ready())
	{
		the_behaviors().lastLuaCallReason = the_motors().fail_reason(the_motors().get_motor_status());
		return ACTION_FAIL;
	}

	//Soft Errors

	else if (!VerifyGPS())
	{
		return ACTION_RUNNING;
	}
	else if (!VerifyCompass())
	{
		return ACTION_RUNNING;
	}
	else
	{
		return ACTION_SUCCESS;
	}
}

ActionResult_enum Autopilot::IsReadyToMove()
{
	if (pilotState == PILOT_STATE_INACTIVE)
	{
		the_behaviors().lastLuaCallReason = "Inactive";
		return ACTION_FAIL;
	}
	else if (!route_planned)
	{
		the_behaviors().lastLuaCallReason = "No Route";
		return ACTION_FAIL;
	}
	else if (the_motors().is_motor_ready() != ACTION_SUCCESS)
	{
		the_behaviors().lastLuaCallReason = the_motors().fail_reason(the_motors().get_motor_status());
		return ACTION_FAIL;
	}
	else if (!VerifyGPS())
	{
		the_behaviors().lastLuaCallReason = "GPS";
		return ACTION_FAIL;
	}
	else if (!VerifyCompass())
	{
		the_behaviors().lastLuaCallReason = "Compass";
		return ACTION_FAIL;
	}
	else
	{
		return ACTION_SUCCESS;
	}
}

ActionResult_enum Autopilot::pilotSetGoalWaypoint(const char *waypointName)
{
	if (strcmp(waypointName, "Heel") == 0)
	{
		return pilotSetGoalPosition(the_behaviors().heel_location);
	}
	else
	{
		Location wp = the_waypoints().GetWaypointByName(waypointName).location();

		if (wp.type == Location::NAMED_WAYPOINT)
		{
			if (pilotSetGoalPosition(wp) == ACTION_SUCCESS)
			{
				return ACTION_SUCCESS;
			}
			else
			{
				return ACTION_FAIL;
			}
		}
		else
		{
			the_behaviors().lastLuaCallReason = "NoWaypoint";
			route_planned = false;
			return ACTION_FAIL;
		}
	}
}

ActionResult_enum Autopilot::pilotSetGoalPosition(Location _goal)
{
	if (the_navigator().isLocationValid())
	{
		goalWaypoint = _goal;
		fromWaypoint = the_navigator().current_location;

		int result = planner->PlanPointToPoint(fromWaypoint, goalWaypoint);

		if (result == 0)
		{
			toWaypoint = planner->planWaypoints[planner->routeWaypointIndex];
			route_planned = true;
			return ACTION_SUCCESS;
		}
		else
		{
			the_behaviors().lastLuaCallReason = "No Route";
			route_planned = false;
			return ACTION_FAIL;
		}
	}
	else
	{
		the_behaviors().lastLuaCallReason = "NoLocation";
		route_planned = false;
		return ACTION_FAIL;
	}
}

ActionResult_enum Autopilot::pilotSetRelativeGoal(int _rangeMM, int _bearingDegrees)
{
	if (!the_navigator().isLocationValid())
	{
		the_behaviors().lastLuaCallReason = "NoLocation";
		route_planned = false;
		return ACTION_FAIL;
	}

	Location goal = the_navigator().current_location.range_bearing(_rangeMM, _bearingDegrees);

	return pilotSetGoalPosition(goal);
}

ActionResult_enum Autopilot::pilotSetRandomGoal(int _rangeMM)
{
	return pilotSetRelativeGoal((drand48() - 0.5) * _rangeMM * 2, drand48() * 360.0);
}

ActionResult_enum Autopilot::pilotSelectRandomWayopint()
{
	int count = the_waypoints().waypoints_size();
	int rand = (int) (count * drand48());

	if (rand == count) rand--;

	return pilotSetGoalPosition(the_waypoints()[rand].location());
}

ActionResult_enum Autopilot::pilotIsAtWaypoint()
{
	int rangeMM = the_navigator().current_location.mm_range_to_point(goalWaypoint);

	if (rangeMM < (arrivalRange * 10)) return ACTION_SUCCESS;
	return ACTION_FAIL;
}

//thread to send updates to the motors
void Autopilot::autopilot_thread_method() {

	//initialize lua callbacks
	the_behaviors().register_lua_callback("PilotAction", &luaPilotAction);

	int i;
	for (i=0; i< PILOT_ACTION_COUNT; i++)
	{
		the_behaviors().register_lua_global(pilotActionList[i], i);
	}

	auto last_pilot_review = system_clock::now();
	auto last_status_update = system_clock::now();

	int priorPilotState = PILOT_STATE_INACTIVE;	//used to cancel notifications

	//initialize planning structures
	planner = new Planner();

	ps_registry_add_new("Pose", "Waypoint", PS_REGISTRY_TEXT_TYPE, PS_REGISTRY_SRC_WRITE);
	ps_registry_add_new("Pose", "Pilot", PS_REGISTRY_TEXT_TYPE, PS_REGISTRY_SRC_WRITE);
	ps_registry_set_text("Pose", "Pilot", "standby");

	DEBUGPRINT("Pilot thread ready");

	try {
		//loop
		while (1)
		{
			{
				//critical section
				std::unique_lock<std::mutex> lck {pilotStateMtx};

				//make pilot available
				if (pilotState == PILOT_STATE_INACTIVE)
				{
					if (motorEnable && initComplete)
					{
						pilotState = PILOT_STATE_IDLE;
						DEBUGPRINT("Pilot Available");
					}
				}
				else
				{
					if (!motorEnable)
					{
						CancelPilotOperation(PILOT_STATE_INACTIVE);
						DEBUGPRINT("Pilot Unavailable");
					}
				}

				//review what's happening

				if (the_navigator().last_location_update > last_pilot_review )
				{
					last_pilot_review = std::chrono::system_clock::now();

					switch (pilotState)
					{
					case PILOT_STATE_ORIENT:
					{
						// turn to 'desiredCompassHeading'
						if (VerifyCompass())
						{
							if (!the_motors().commandRunning)
							{
								prior_heading_error = 0;
							}

							ActionResult_enum result = the_motors().TurnToHeading(desiredCompassHeading);

							switch (result)
							{
							case ACTION_RUNNING:
								if (abs(the_motors().heading_error - prior_heading_error) < 10)
								{
									DEBUGPRINT("Pilot: Turn Stalled?");
						//			pilotState = PILOT_STATE_FAILED;
						//			return ACTION_FAIL;
									prior_heading_error = the_motors().heading_error;
								}
								break;
							case ACTION_SUCCESS:
								DEBUGPRINT("Pilot: Orient done");
								pilotState = PILOT_STATE_DONE;
								break;
							default:
								DEBUGPRINT("Pilot: Orient failed");
								pilotState = PILOT_STATE_FAILED;
								break;
							}
						}
						else
						{
							pilotState = PILOT_STATE_FAILED;
						}
						break;
					}
					case PILOT_STATE_ORIENT_MOVE:
					{
						// turn to 'desiredCompassHeading'
						if (VerifyCompass())
						{
							if (!the_motors().commandRunning)
							{
								prior_heading_error = 0;
							}
							ActionResult_enum result = the_motors().TurnToHeading(desiredCompassHeading);

							switch (result)
							{
							case ACTION_RUNNING:
								if (abs(the_motors().heading_error - prior_heading_error) < 10)
								{
									DEBUGPRINT("Pilot: Orient_Move Stalled?");
						//			pilotState = PILOT_STATE_FAILED;
						//			return ACTION_FAIL;
									prior_heading_error = the_motors().heading_error;
								}
								break;
							case ACTION_SUCCESS:
								DEBUGPRINT("Pilot: Orient_move done");
								pilotState = PILOT_STATE_MOVE;
								break;
							default:
								DEBUGPRINT("Pilot: Orient_move failed");
								pilotState = PILOT_STATE_FAILED;
								break;
							}
							break;
						}
						else
						{
							pilotState = PILOT_STATE_FAILED;
						}
					}
					break;

					case PILOT_STATE_MOVE:
						if (VerifyCompass() && VerifyGPS())
						{
							FindRabbit();
							PerformMovement();
						}
						else
						{
							pilotState = PILOT_STATE_FAILED;
						}
						break;
					default:
						break;
					}
				}

				//check timeouts
				switch (pilotState)
				{
				case PILOT_STATE_ORIENT:
				case PILOT_STATE_ORIENT_MOVE:
				case PILOT_STATE_MOVE:

//					if (MOVE_XXX_time == 0)
//						MOVE_XXX_time = time(NULL);
//					else
//					{
//						if (MOVE_XXX_time + motorsRunTimeout < time(NULL))
//						{
//							CancelPilotOperation(PILOT_STATE_FAILED);
//							the_behaviors().lastLuaCallReason = "RunTO";
//							MOVE_XXX_time = 0;
//							LogWarning("Motors Run TO");
//						}
//					}
					break;
				default:
					MOVE_XXX_time = 0;
					break;
				}

				//update notifications
				if (priorPilotState != pilotState) {
					DEBUGPRINT("Pilot State: %s", pilotStateNames[pilotState]);

					bool pilot_engaged 	= ps_test_condition(SOURCE, PILOT_ENGAGED);
					bool pilot_idle 	= ps_test_condition(SOURCE, PILOT_IDLE);
					bool pilot_failed 	= ps_test_condition(SOURCE, PILOT_FAILED);

					switch (pilotState) {
					case PILOT_STATE_IDLE:
						if (pilot_engaged) ps_cancel_condition(PILOT_ENGAGED);
						if (!pilot_idle) ps_set_condition(PILOT_IDLE);
						if (pilot_failed) ps_cancel_condition(PILOT_FAILED);
						break;
					case PILOT_STATE_INACTIVE:
						if (pilot_engaged) ps_cancel_condition(PILOT_ENGAGED);
						if (pilot_idle) ps_cancel_condition(PILOT_IDLE);
						if (pilot_failed) ps_cancel_condition(PILOT_FAILED);
						break;
					case PILOT_STATE_ORIENT:
					case PILOT_STATE_MOVE:
					case PILOT_STATE_ORIENT_MOVE:
						if (pilot_idle) ps_cancel_condition(PILOT_IDLE);
						if (pilot_failed) ps_cancel_condition(PILOT_FAILED);
						if (!pilot_engaged) ps_set_condition(PILOT_ENGAGED);
						break;
					case PILOT_STATE_DONE:
						if (!pilot_idle) ps_set_condition(PILOT_IDLE);
						if (pilot_failed) ps_cancel_condition(PILOT_FAILED);
						if (pilot_engaged) ps_cancel_condition(PILOT_ENGAGED);
						break;
					case PILOT_STATE_FAILED:
					case PILOT_STATE_ABORT:
						if (pilot_idle) ps_cancel_condition(PILOT_IDLE);
						if (!pilot_failed) ps_set_condition(PILOT_FAILED);
						if (pilot_engaged) ps_cancel_condition(PILOT_ENGAGED);
						break;
					}

					priorPilotState = pilotState;
				}
				char pilotStatusBuffer[REGISTRY_TEXT_LENGTH];

				if (last_status_update + std::chrono::milliseconds(PILOT_REGISTRY_UPDATE_MS) < std::chrono::system_clock::now())
				{
					switch (pilotState) {

					case PILOT_STATE_ORIENT:
					case PILOT_STATE_MOVE:
					case PILOT_STATE_ORIENT_MOVE:
						if (toWaypoint.type == Location::NAMED_WAYPOINT)
						{
							ps_registry_set_text("Pose", "Waypoint", toWaypoint.name.c_str());
						}

						snprintf(pilotStatusBuffer, REGISTRY_TEXT_LENGTH,
									"DTW = %icm. BTW = %i. XTE = %icm",
									(int) length_currentToNext / 10, (int) bearing_currentToNext, XTE / 10);

						ps_registry_set_text("Pose", "Pilot", pilotStatusBuffer);
						break;

					default:
//						ps_registry_set_text("Pose", "Waypoint", "");
//						ps_registry_set_text("Pose", "Pilot", "standby");
						break;
					}
				}

			}
			usleep((PILOT_LOOP_MS * 1000));
		}
	} catch (exception &e) {
		PS_ERROR("pilot: thread exception: %s", e.what());
	}
}

void Autopilot::HandleEvent(ps_event_id_t event)
{
	//critical section
	std::unique_lock<std::mutex> lck {pilotStateMtx};

	switch(event)
	{
	case BATTERY_SHUTDOWN_EVENT:
		CancelPilotOperation(PILOT_STATE_INACTIVE);
		break;
	default:
		break;
	}
}

bool Autopilot::VerifyCompass()
{
	if (!the_navigator().isOrientationDR()) {
		DEBUGPRINT("Pilot: No compass fail");
		the_behaviors().lastLuaCallReason = "Compass";
		return false;
	}
	return true;
}

bool Autopilot::VerifyGPS()
{
	if (!the_navigator().isLocationDGPS()) {
		DEBUGPRINT("Pilot: No GPS fail");
		the_behaviors().lastLuaCallReason = "GPS";
		return false;
	}
	return true;
}

bool Autopilot::isArrived()
{
	int rangeMM = the_navigator().current_location.mm_range_to_point(goalWaypoint);

	if (rangeMM < (arrivalRange * 10))
	{
		return true;
	}

	return false;
}

ActionResult_enum Autopilot::PerformMovement()
{
	ActionResult_enum result;

	if (!the_motors().commandRunning)
	{
		//motors are not running
		//check end condition

		int rangeMM = the_navigator().current_location.mm_range_to_point(goalWaypoint);

		if (rangeMM < (arrivalRange * 10))
		{
			DEBUGPRINT("Pilot: Route done");

			pilotState = PILOT_STATE_DONE;
			return ACTION_SUCCESS;
		}
		else
		{
			DEBUGPRINT("Pilot: Range to goal = %i mm", rangeMM);
		}

		//check  orientation

		int heading_deviation = abs(desiredCompassHeading - the_navigator().heading);

		if (heading_deviation > veerLimit)
		{
			//large deviation - turn first
			prior_heading_error = 0;
			result = the_motors().TurnToHeading(desiredCompassHeading);

			switch(result)
			{
			case ACTION_RUNNING:
				pilotState = PILOT_STATE_ORIENT_MOVE;
				if (abs(the_motors().heading_error - prior_heading_error) < 10)
				{
					DEBUGPRINT("Pilot: Turn Stalled?");
		//			pilotState = PILOT_STATE_FAILED;
		//			return ACTION_FAIL;
					prior_heading_error = the_motors().heading_error;
				}
				return ACTION_RUNNING;
				break;
			case ACTION_SUCCESS:
				pilotState = PILOT_STATE_MOVE;
				break;
			default:
				DEBUGPRINT("Pilot: Move failed");
				pilotState = PILOT_STATE_FAILED;
				return ACTION_FAIL;
				break;
			}
		}
		XTE_heading_adjustment = 0;
		prior_DTG = 0;
	}

	if (final_approach & (length_lastToCurrent > length_lastToNext))
	{
		DEBUGPRINT("pilot: passed waypoint");
		the_motors().cancel_action();
		pilotState = PILOT_STATE_DONE;
		return ACTION_SUCCESS;
	}

	int new_heading = (360 + desiredCompassHeading + (int)XTE_heading_adjustment) % 360;
	int course_change = (new_heading - the_navigator().heading) ;
	if (abs(course_change) < veerLimit)
	{

		result = the_motors().Move_towards(new_heading, desiredDistance, true);

		switch (result)
		{
		case ACTION_RUNNING:
//			if (prior_DTG - the_motors().DTG < 50)
//			{
//				DEBUGPRINT("Pilot: Move Stalled?");
//				//			pilotState = PILOT_STATE_FAILED;
//				//			return ACTION_FAIL;
//				prior_DTG = the_motors().DTG;
//			}
			return ACTION_RUNNING;
			break;
		case ACTION_SUCCESS:
			return ACTION_RUNNING;
			break;
		default:
			DEBUGPRINT("Pilot: Move failed");
			pilotState = PILOT_STATE_FAILED;
			return ACTION_FAIL;
			break;
		}
	}
	else
	{
		DEBUGPRINT("pilot: course change too great: %+i", course_change);
		return the_motors().cancel_action();
	}
}

#define RABBIT_LEAD_MM 1000
int Autopilot::FindRabbit()
{
	 length_lastToCurrent = fromWaypoint.mm_range_to_point(the_navigator().current_location);
	 length_lastToNext = fromWaypoint.mm_range_to_point(toWaypoint);
	 length_currentToNext = the_navigator().current_location.mm_range_to_point(toWaypoint);

	 bearing_lastToCurrent = fromWaypoint.bearing_to_point(the_navigator().current_location);
	 bearing_lastToNext = fromWaypoint.bearing_to_point(toWaypoint);
	 bearing_currentToNext = the_navigator().current_location.bearing_to_point(toWaypoint);

	double driftAngle 	= (bearing_lastToNext - bearing_currentToNext);
	XTE 				= (int) length_currentToNext * sin(DEGREES_TO_RADIANS(driftAngle));

	//project onto route:
	int lengthAlongRoute = (int) (length_lastToNext - length_currentToNext * cos(DEGREES_TO_RADIANS(driftAngle)));

	int rabbitLead = (RABBIT_LEAD_MM > 2 * abs(XTE) ? RABBIT_LEAD_MM : RABBIT_LEAD_MM + abs(XTE));

	int rabbitFromStart = rabbitLead + lengthAlongRoute;
	if (rabbitFromStart < 0) rabbitFromStart = 0;

	while ((rabbitFromStart > length_lastToNext) && (planner->routeWaypointIndex < planner->planWaypointCount-1))
	{
		DEBUGPRINT("pilot: Rabbit passed waypoint %d: %s", planner->routeWaypointIndex, planner->planWaypoints[planner->routeWaypointIndex].name.c_str());

		rabbitFromStart = (int) rabbitFromStart - length_lastToNext;

		//next leg
		fromWaypoint = toWaypoint;
		toWaypoint = planner->planWaypoints[++planner->routeWaypointIndex];

		length_lastToCurrent = fromWaypoint.mm_range_to_point(the_navigator().current_location);
		length_lastToNext = fromWaypoint.mm_range_to_point(toWaypoint);
		length_currentToNext = the_navigator().current_location.mm_range_to_point(toWaypoint);

		bearing_lastToCurrent = fromWaypoint.bearing_to_point(the_navigator().current_location);
		bearing_lastToNext = fromWaypoint.bearing_to_point(toWaypoint);
		bearing_currentToNext = the_navigator().current_location.bearing_to_point(toWaypoint);

		driftAngle 	= (bearing_lastToNext - bearing_currentToNext);
		XTE 				= (int) length_currentToNext * sin(DEGREES_TO_RADIANS(driftAngle));

		//project onto route:
		lengthAlongRoute = (int) (length_lastToNext - length_currentToNext * cos(DEGREES_TO_RADIANS(driftAngle)));

		rabbitLead = (RABBIT_LEAD_MM > 2 * abs(XTE) ? RABBIT_LEAD_MM : RABBIT_LEAD_MM + abs(XTE));

		rabbitFromStart = rabbitLead + lengthAlongRoute;
		if (rabbitFromStart < 0) rabbitFromStart = 0;
	}

	if (toWaypoint.type == Location::NAMED_WAYPOINT)
	{
		DEBUGPRINT("pilot: current to %s. %icm @ %03i",
				toWaypoint.name.c_str(), (int) length_currentToNext / 10, (int) bearing_currentToNext);
	}
	else
	{
		DEBUGPRINT("pilot: current to %f,%f. %icm @ %03i",
				toWaypoint.latitude_in_degrees(), toWaypoint.longitude_in_degrees(),
				(int)length_currentToNext / 10, (int) bearing_currentToNext);
	}

	if (fromWaypoint.type == Location::NAMED_WAYPOINT)
	{

		DEBUGPRINT("pilot: %s to current is %icm @ %03i", fromWaypoint.name.c_str(),
				(int) length_lastToCurrent/10, (int)bearing_lastToCurrent);

		if (toWaypoint.type == Location::NAMED_WAYPOINT)
		{
			DEBUGPRINT("pilot: %s to %s is %icm @ %03i", fromWaypoint.name.c_str(), toWaypoint.name.c_str(),
					(int) length_lastToNext/10, (int)bearing_lastToNext);
		}
		else
		{
			DEBUGPRINT("pilot: %s to nextWP is %icm @ %03i", fromWaypoint.name.c_str(),
					(int) length_lastToNext/10, (int)bearing_lastToNext);
		}
	}
	else
	{
		DEBUGPRINT("pilot: fromWP to current is %icm @ %03i",
				(int) length_lastToCurrent/10, (int)bearing_lastToCurrent);

		if (toWaypoint.type == Location::NAMED_WAYPOINT)
		{
			DEBUGPRINT("pilot: fromWP to %s is %icm @ %03i", toWaypoint.name.c_str(),
					(int) length_lastToNext/10, (int)bearing_lastToNext);
		}
		else
		{
			DEBUGPRINT("pilot: fromWP to nextWP is %icm @ %03i",
					(int) length_lastToNext/10, (int)bearing_lastToNext);
		}
	}

	if (rabbitFromStart < length_lastToNext)
	{
		rabbit = fromWaypoint.range_bearing(rabbitFromStart, bearing_lastToNext);
		final_approach = false;
	}
	else
	{
		//end of route
		rabbit = toWaypoint;
		final_approach = true;
	}

	desiredCompassHeading = (int) the_navigator().current_location.bearing_to_point(rabbit);
	desiredDistance = (int) the_navigator().current_location.mm_range_to_point(rabbit);

	if (XTE > XTEdeadZone)
	{
		if ((XTE > lastXTE) || (XTE > 2 * XTEdeadZone))
		{
			XTE_heading_adjustment += XTEadjustment;
		}
	}
	else if (XTE < -XTEdeadZone)
	{
		if ((XTE < lastXTE) || (XTE < -2 * XTEdeadZone))
		{
			XTE_heading_adjustment -= XTEadjustment;
		}
	}
	lastXTE = XTE;

	DEBUGPRINT("pilot: rabbit is %icm @ %i degrees", desiredDistance / 10, desiredCompassHeading);
	DEBUGPRINT("pilot: XTE = %icm, Adj = %0.1f", XTE / 10, XTE_heading_adjustment);
	return 0;
}

AutopilotClass& the_autopilot()
{
	return the_autopilot_instance();
}

Autopilot& the_autopilot_instance()
{
	static Autopilot pilot;
	return pilot;
}

const char *pilotStateNames[] = PILOT_STATE_NAMES;
const char *pilotActionList[] = PILOT_ACTION_LIST;
