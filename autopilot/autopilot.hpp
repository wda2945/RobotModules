/*
 * autopilot.hpp
 *
 *  Created on: 2015
 *      Author: martin
 */


#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

#include <thread>
#include <mutex>

#include "ps.h"
#include "AutopilotClass.hpp"

#include "mapping/waypoints.hpp"
#include "mapping/planner.hpp"

#include "BehaviorClass.hpp"
#include "MotorClass.hpp"
#include "NavigatorClass.hpp"

//autopilot state machine
typedef enum {
	PILOT_STATE_IDLE,			//ready for a command
	PILOT_STATE_ORIENT,			//just turning
	PILOT_STATE_ORIENT_MOVE,	//turn then move
	PILOT_STATE_MOVE,			//just moving
	PILOT_STATE_DONE,			//move complete
	PILOT_STATE_FAILED,			//move failed
	PILOT_STATE_ABORT,
	PILOT_STATE_INACTIVE		//motors disabled
} PilotState_enum;

#define PILOT_STATE_NAMES {\
		"IDLE",\
		"ORIENT",\
		"ORIENT_MOVE",\
		"MOVE",\
		"DONE",\
		"FAILED",\
		"ABORT",\
		"INACTIVE"\
};

//pilot actions
typedef enum {
	isPilotReady,
	WaitUntilPilotReady,
	ComputeHeelRoute,
	ComputeRandomRoute,
	ComputeRandomClosePosition,
	Orient,
	Engage,
	PilotReset,
	isAtGoal,
	PILOT_ACTION_COUNT
} PilotAction_enum;

#define PILOT_ACTION_LIST {\
		"Pilot_isPilotReady",\
		"Pilot_WaitUntilPilotReady",\
		"Pilot_ComputeHeelRoute",\
		"Pilot_ComputeRandomRoute",\
		"Pilot_ComputeRandomClosePosition",\
		"Pilot_Orient",\
		"Pilot_Engage",\
		"Pilot_PilotReset",\
		"Pilot_isAtGoal",\
}

class Autopilot : public AutopilotClass
{
private:
	Autopilot();
	~Autopilot() {}

	ActionResult_enum pilotSetGoalWaypoint(const char *waypointName) override;
	ActionResult_enum pilotSetGoalPosition(Location _goal) override;
	ActionResult_enum pilotSetRelativeGoal(int _rangeCM, int _bearingDegrees) override;
	ActionResult_enum pilotSetRandomGoal(int _rangeCM) override;
	ActionResult_enum pilotSelectRandomWayopint() override;
	ActionResult_enum pilotIsAtWaypoint() override;

	ActionResult_enum Action(PilotAction_enum _action);

	ActionResult_enum WaitUntilReadyToMove() override;
	ActionResult_enum IsReadyToMove() override;

	void HandleEvent(ps_event_id_t event);

	//final target
	Location goalWaypoint;
	bool route_planned {false};

	Location toWaypoint;
	Location fromWaypoint;
	Location rabbit;

	PilotState_enum pilotState = PILOT_STATE_INACTIVE;

	Waypoints *waypoints {nullptr};
	Planner   *planner {nullptr};
	
private:
	std::mutex	pilotStateMtx;
	std::thread *pilot_thread;

	//Autopilot thread
	void autopilot_thread_method();

	bool CancelPilotOperation(PilotState_enum newState);

	bool VerifyCompass();
	bool VerifyGPS();
	bool isArrived();

	// action
	ActionResult_enum PerformMovement();
	int FindRabbit();

	bool final_approach 		{false};	//desiredDistance is to destination, not rabbit
	int desiredCompassHeading 	{0};	//degrees
	int desiredDistance 		{0};	//mm
	int prior_desiredDistance 		{0};	//mm
	int prior_DTG		 		{0};	//mm
	int prior_heading_error		{0};
	int XTE 					{0};	//mm
	int lastXTE					{0};
	float XTE_heading_adjustment	{0};

	uint32_t left_encoder_start {0};
	uint32_t right_encoder_start {0};

	bool motorsInhibit 	= false;
	bool pilotEngaged   = false;

	time_t MOVE_XXX_SENT_time = 0;
	time_t MOVE_XXX_time = 0;
	int motorsRunTimeout {0};

	float motorSpeed {0};

	double length_lastToCurrent	{0};
	double length_lastToNext	{0};
	double length_currentToNext	{0};

	double bearing_lastToCurrent	{0};
	double bearing_lastToNext	{0};
	double bearing_currentToNext	{0};

	friend Autopilot& the_autopilot_instance();
	friend int luaPilotAction(lua_State *L);
	friend void AutopilotHandleEvent(void *arg, ps_event_id_t event);
};

Autopilot& the_autopilot_instance();

extern const char *pilotStateNames[];
extern const char *pilotActionList[];

#endif
