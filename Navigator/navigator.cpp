/*
 * navigator.cpp
 *
 *      Author: martin
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
#include <pthread.h>
#include <errno.h>

#include "robot.h"

#include "nav_debug.h"
#include "navigator.hpp"
#include "AutopilotClass.hpp"
#include "BehaviorClass.hpp"
#include "MotorClass.hpp"

#include "kalman.h"
#include "matrix.h"
#include "mapping/location.hpp"
#include "mapping/waypoints.hpp"

FILE *navDebugFile;

#define NAV_LOOP_MS					100

//reporting criteria
#define REPORT_BEARING_CHANGE 		2			//degrees
#define REPORT_CONFIDENCE_CHANGE 	0.2f	//probability
#define REPORT_MAX_INTERVAL			5			//seconds
#define REPORT_MIN_CONFIDENCE		0.5f
#define REPORT_LOCATION_CHANGE 		5

#define RAW_DATA_TIMEOUT 		5	//seconds

#define VERY_LARGE_COVARIANCE 1000000000.0
#define NORMALIZE_HEADING(x) (x + 360)%360

#define DEGREESTORADIANS(x) (x * M_PI / 180.0)
#define RADIANSTODEGREES(x) ((x * 180.0) / M_PI)

//actual BT leaf node
int luaNavAction(lua_State *L)
{
	NavAction_enum actionCode 	= (NavAction_enum) lua_tointeger(L, 1);

	if (actionCode < NAV_ACTION_COUNT)
	{
		the_behaviors().lastLuaCall = navActionList[actionCode];

		DEBUGPRINT("Nav Action: %s ...", navActionList[actionCode]);

		return actionReply(L, the_navigator_instance().Action(actionCode));
	}
	else
	{
		ERRORPRINT("Nav Action %i invalid", actionCode);

		return fail(L);
	}
}

Navigator::Navigator()
{
	navDebugFile = fopen_logfile("navigator");

	//create navigator thread
	nav_thread = new std::thread([this](){navigator_thread_method();});
}

ActionResult_enum Navigator::Action(NavAction_enum _action)
{
	switch (_action)
	{
	case AssumeHome:
		return assume_location((char*)"Home");
		break;
	case AssumeN:
		return assume_heading(0);
		break;
	case OrientationValid:
		if (isOrientationValid()) return ACTION_SUCCESS;
		else return ACTION_FAIL;
		break;
	case OrientationDR:
		if (isOrientationDR()) return ACTION_SUCCESS;
		else return ACTION_FAIL;
		break;
	case OrientationAbsolute:
		if (isOrientationAbsolute()) return ACTION_SUCCESS;
		else return ACTION_FAIL;
		break;
	case LocationValid:
		if (isLocationValid()) return ACTION_SUCCESS;
		else return ACTION_FAIL;
		break;
	case LocationDGPS:
		if (isLocationDGPS()) return ACTION_SUCCESS;
		else return ACTION_FAIL;
		break;
	case Location10cm:
		if (isLocation10cm()) return ACTION_SUCCESS;
		else return ACTION_FAIL;
		break;
	default:
		return ACTION_FAIL;
		break;
	}
}

ActionResult_enum Navigator::assume_heading(int _heading)
{
	switch (orientationStatus)
	{
	case ORIENT_UNKNOWN:
	case ORIENT_RELATIVE:
	case ORIENT_DEADRECKONED:
		if (imu_online)
		{
			imu_offset = _heading - imu_heading;
			while (imu_offset < -180) imu_offset += 360;
			while (imu_offset >  180) imu_offset -= 360;
		}
		else
		{
			imu_offset = 0;
		}
		heading = _heading;
		using_assumed_heading = true;
		orientationStatus = ORIENT_DEADRECKONED;
		return ACTION_SUCCESS;
		break;
	case ORIENT_ABS:
		using_assumed_heading = false;
		if (abs(heading - _heading) < 20)
		{
			return ACTION_SUCCESS;
		}
		else
		{
			the_behaviors().lastLuaCallReason = "GPS Heading";
			return ACTION_FAIL;
		}
		break;
	}
	return ACTION_FAIL;
}

ActionResult_enum Navigator::assume_location(char *waypoint)
{
	if (locationStatus >= LOCATION_VALID)
	{
		the_behaviors().lastLuaCallReason = "GPS Online";
		return ACTION_FAIL;
	}
	else
	{
		current_location = the_waypoints().GetWaypointByName(waypoint).location();
		if (current_location.type == Location::NAMED_WAYPOINT)
		{
			locationStatus = LOCATION_VALID;
			return ACTION_SUCCESS;
		}
		else
		{
			the_behaviors().lastLuaCallReason = "No waypoint";
			return ACTION_FAIL;
		}
	}
}

void Navigator::navigator_thread_method()
{
	try {
		bool just_started = true;

		//registry entries
		ps_registry_add_new("Pose", "Latitude", PS_REGISTRY_REAL_TYPE, PS_REGISTRY_SRC_WRITE);
		ps_registry_add_new("Pose", "Longitude", PS_REGISTRY_REAL_TYPE, PS_REGISTRY_SRC_WRITE);
		ps_registry_add_new("Pose", "Heading", PS_REGISTRY_INT_TYPE, PS_REGISTRY_SRC_WRITE);
		ps_registry_add_new("Pose", "Status", PS_REGISTRY_TEXT_TYPE, PS_REGISTRY_SRC_WRITE);
		ps_registry_add_new("Waypoint","Fix", PS_REGISTRY_BOOL_TYPE, PS_REGISTRY_SRC_WRITE);
		ps_registry_add_new("Pose", "Accuracy",  PS_REGISTRY_REAL_TYPE, PS_REGISTRY_SRC_WRITE);

		ps_registry_set_real("Pose", "Latitude", 0);
		ps_registry_set_real("Pose", "Longitude", 0);
		ps_registry_set_int("Pose", "Heading", 0);
		ps_registry_set_text("Pose", "Status", "Offline");
		ps_registry_set_bool("Waypoint","Fix", false);

		//register LUA callbacks
		the_behaviors().register_lua_callback("NavigatorAction", &luaNavAction);

		int i;
		for (i=0; i< NAV_ACTION_COUNT; i++)
		{
			the_behaviors().register_lua_global(navActionList[i], i);
		}

		//set up filters
		////////////////////////////////////////////////////////////////////////
		//heading filter - 2 dimensions system (h, dh), 1 dimension measurement
		KalmanFilter HeadingFilter = alloc_filter(2, 1);
		set_identity_matrix(HeadingFilter.state_transition);
#define SET_HEADING_CHANGE(H) HeadingFilter.state_transition.data[0][1] = H;
		//then predict(f)

		/* We only observe (h) each time */
		set_matrix(HeadingFilter.observation_model,
				1.0, 0.0);
#define SET_HEADING_OBSERVATION(H) set_matrix(HeadingFilter.observation, H);
		//then estimate(f)

		/* Noise in the world. */
		double pos = 10.0;
		set_matrix(HeadingFilter.process_noise_covariance,
				pos, 0.0,
				0.0, 1.0);
#define SET_HEADING_PROCESS_NOISE(N) HeadingFilter.process_noise_covariance.data[0][0] = N;

		/* Noise in our observation */
		set_matrix(HeadingFilter.observation_noise_covariance, 4.0);
#define SET_HEADING_OBSERVATION_NOISE(N) set_matrix(HeadingFilter.observation_noise_covariance, N);

		/* The start.heading is unknown, so give a high variance */
		set_matrix(HeadingFilter.state_estimate, 0.0, 0.0);
		set_identity_matrix(HeadingFilter.estimate_covariance);
		scale_matrix(HeadingFilter.estimate_covariance, 100000.0);
#define GET_HEADING NORMALIZE_HEADING((int) HeadingFilter.state_estimate.data[0][0])	//always 0 to 359

		////////////////////////////////////////////////////////////////////////////
		//location filter - 4 dimensions system (n,e,dn,de), 2 dimensions measurement (x,y)
		KalmanFilter LocationFilter = alloc_filter(4, 2);
		set_identity_matrix(LocationFilter.state_transition);
		//PREDICT STEP
#define SET_NORTHING_CHANGE(N) LocationFilter.state_transition.data[0][2] = N;
#define SET_EASTING_CHANGE(E) LocationFilter.state_transition.data[1][3] = E;
		//then predict(f)

		/* We observe (x, y) in each time step */
		set_matrix(LocationFilter.observation_model,
				1.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0);
#define SET_LOCATION_OBSERVATION(N,E) set_matrix(LocationFilter.observation, N, E);
		//then estimate(f)

		/* Noise in the world. */
		set_matrix(LocationFilter.process_noise_covariance,
				pos, 0.0, 0.0, 0.0,
				0.0, pos, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0);
#define SET_LOCATION_PROCESS_NOISE(N) LocationFilter.state_transition.data[0][0] = N;LocationFilter.state_transition.data[1][1] = N;

		/* Noise in our observation */
		set_matrix(LocationFilter.observation_noise_covariance,
				1000.0, 0.0,
				0.0, 1000.0);
#define SET_LOCATION_OBSERVATION_NOISE(N, E) LocationFilter.observation_noise_covariance.data[0][0] = N;LocationFilter.observation_noise_covariance.data[1][1] = E;
#define VERY_LARGE_COVARIANCE 1000000000.0
		/* The start position is unknown, so give a high variance */
		set_matrix(LocationFilter.state_estimate, 19.41467, 154.891664, 0.0, 0.0);
		set_identity_matrix(LocationFilter.estimate_covariance);
		scale_matrix(LocationFilter.estimate_covariance, 100000.0);
#define GET_LATITUDE 	(LocationFilter.state_estimate.data[0][0])
#define GET_LONGITUDE 	(LocationFilter.state_estimate.data[1][0])


		char navStatusBuffer[REGISTRY_TEXT_LENGTH];

		OrientationStatus_enum last_orientationStatus = ORIENT_UNKNOWN;
		LocationStatus_enum last_locationStatus = LOCATION_UNKNOWN;

		auto last_gps_update  = std::chrono::system_clock::now();
		auto last_imu_update  = std::chrono::system_clock::now();

		while (1) {

			if (the_imu().new_imu_data())
			{
				last_imu_update  = std::chrono::system_clock::now();

				imu_heading = the_imu().heading;

				if (!imu_online)
				{
					imu_online = true;
					DEBUGPRINT("nav: IMU online");
				}

				switch (orientationStatus)
				{
				case ORIENT_UNKNOWN:
				case ORIENT_RELATIVE:
					orientationStatus = ORIENT_RELATIVE;
					heading = imu_heading;
					using_imu_heading = true;
					last_heading_update = the_imu().updated;
					break;
				case ORIENT_DEADRECKONED:
					heading = imu_heading + imu_offset;
					if (heading < 0) heading += 360;
					if (heading >= 360) heading -= 360;

					using_imu_heading = true;
					last_heading_update = the_imu().updated;
					break;
				case ORIENT_ABS:
					break;
				}

				if (just_started)
				{
					//first data, assume north
					assume_heading(0);
					just_started = false;
				}

				//			SET_HEADING_OBSERVATION(the_imu().heading);
				//			SET_HEADING_OBSERVATION_NOISE(5.0)
				//			estimate(HeadingFilter);
				//
				//			DEBUGPRINT("IMU: Filtered heading: %i", GET_HEADING);
			}
			else
			{
				if (last_imu_update + std::chrono::seconds((int) imuTimeout) < std::chrono::system_clock::now())
				{
					if (imu_online)
					{
						DEBUGPRINT("nav: IMU timeout");
						imu_online = false;

						switch (orientationStatus)
						{
						case ORIENT_UNKNOWN:
							break;
						case ORIENT_RELATIVE:
						case ORIENT_DEADRECKONED:
							orientationStatus = ORIENT_UNKNOWN;
							break;
						case ORIENT_ABS:
							break;
						}
					}
				}
			}

			if (the_gps().new_gps_data())
			{
				last_gps_update  = std::chrono::system_clock::now();

				if (!gps_online)
				{
					gps_online = true;
					ps_set_condition(GPS_ONLINE);
					DEBUGPRINT("nav: GPS online");
				}

				if (the_gps().fix)
				{
					if (!gps_fix)
					{
						gps_fix = true;
						DEBUGPRINT("nav: GPS fix");
					}

					//update the filter
					//			SET_LOCATION_OBSERVATION(the_gps().latitude, the_gps().longitude);
					//			SET_LOCATION_OBSERVATION_NOISE(the_gps().horizontalAcc * 100, the_gps().horizontalAcc* 100);
					//			predict(LocationFilter);
					//
					//			DEBUGPRINT("GPS: Filtered location %fN, %fE", GET_LATITUDE, GET_LONGITUDE)

					current_location.latitude = the_gps().latitude;
					current_location.longitude = the_gps().longitude;
					gpsLocationValid = the_gps().positionValid;

					gps_heading = the_gps().headMotion;

					groundSpeed = the_gps().groundSpeed;
					gpsSpeedValid  = the_gps().speedValid;

					last_location_update = the_gps().fix_time;

					if (the_gps().positionValid)
					{
						if (the_gps().dgps_data && (the_gps().horizontalAcc < 200))
						{
							if (the_gps().horizontalAcc < 100)
							{
								locationStatus = LOCATION_10CM;
							}
							else
							{
								locationStatus = LOCATION_DGPS;
							}
						}
						else
						{
							locationStatus = LOCATION_VALID;
						}
					}
					else
					{
						locationStatus = LOCATION_UNKNOWN;
					}

					if (the_gps().headingValid)
					{
						if (orientationStatus != ORIENT_ABS)
						{
							orientationStatus = ORIENT_ABS;
							DEBUGPRINT("nav: GPS Heading Valid");
						}

						heading = gps_heading;
						using_imu_heading = false;
						imu_offset = gps_heading - imu_heading;
						while (imu_offset < -180) imu_offset += 360;
						while (imu_offset >  180) imu_offset -= 360;

						last_heading_update = the_gps().update_time;
					}
					else
					{
						if (orientationStatus == ORIENT_ABS)
						{
							orientationStatus = ORIENT_DEADRECKONED;
							DEBUGPRINT("nav: GPS Heading Lost");
						}
					}
				}
				else
				{
					if (gps_fix)
					{
						gps_fix = false;
						DEBUGPRINT("nav: GPS fix lost");

						switch (orientationStatus)
						{
						case ORIENT_UNKNOWN:
						case ORIENT_RELATIVE:
						case ORIENT_DEADRECKONED:
							break;
						case ORIENT_ABS:
							orientationStatus = ORIENT_DEADRECKONED;
							break;
						}

						locationStatus = LOCATION_UNKNOWN;
					}

				}
			}
			else
			{
				if (last_gps_update + std::chrono::seconds((int) gpsTimeout) < std::chrono::system_clock::now())
				{
					if (gps_fix)
					{
						gps_fix = false;
						DEBUGPRINT("nav: GPS fix timeout");

						switch (orientationStatus)
						{
						case ORIENT_UNKNOWN:
						case ORIENT_RELATIVE:
						case ORIENT_DEADRECKONED:
							break;
						case ORIENT_ABS:
							orientationStatus = ORIENT_DEADRECKONED;
							break;
						}
					}
					locationStatus = LOCATION_UNKNOWN;
					gps_online = false;
				}

			}

			int x, y, zRotation;

			if (the_motors().get_movement(&x, &y, &zRotation))
			{
				//got ODO report from motors

				if (!gps_fix) locationStatus = LOCATION_UNKNOWN;

				//update heading belief
				//			float veerAngle = (float) zRotation;
				//			SET_HEADING_CHANGE(veerAngle);
				//			estimate(HeadingFilter);
				//
				//			DEBUGPRINT("ODO: Filtered heading: %i", GET_HEADING);
				//
				//			float hRadians = DEGREESTORADIANS(GET_HEADING);
				//
				//			//location change
				//			SET_NORTHING_CHANGE(x * cosf(hRadians));
				//			SET_EASTING_CHANGE(x * sinf(hRadians));
				//
				//			DEBUGPRINT("ODO: Filtered location: %f, %f", GET_LATITUDE, GET_LONGITUDE);
			}

			//		heading 	= GET_HEADING;
			//		latitude 	= GET_LATITUDE;
			//		longitude 	= GET_LONGITUDE;

			if (current_location.mm_range_to_point(last_location_logged) > 50)
			{
				DEBUGPRINT( "nav: Location %f, %f (+/- %f m)",
						current_location.latitude_in_degrees(),
						current_location.longitude_in_degrees(),
						(float) the_gps().horizontalAcc / 1000);
				last_location_logged = current_location;
			}

			if (last_registry_update + std::chrono::seconds((int)appReportInterval) < std::chrono::system_clock::now())
			{
				if (orientationStatus > ORIENT_UNKNOWN)
				{
					ps_registry_set_int("Pose", "Heading", heading);
				}
				if (locationStatus > LOCATION_UNKNOWN)
				{
					ps_registry_set_real("Pose", "Longitude", current_location.longitude_in_degrees());
					ps_registry_set_real("Pose", "Latitude", current_location.latitude_in_degrees());
					ps_registry_set_real("Pose", "Accuracy", (float) the_gps().horizontalAcc / 1000);		//m

					ps_registry_set_bool("Waypoint","Fix", (locationStatus == LOCATION_10CM));
				}

				const char *os = orientationStatusNames[orientationStatus];
				const char *ls = locationStatusNames[locationStatus];

				snprintf(navStatusBuffer, REGISTRY_TEXT_LENGTH, "%s : %s", ls, os);

				ps_registry_set_text("Pose", "Status", navStatusBuffer);
				DEBUGPRINT( "nav: %s", navStatusBuffer);
				last_registry_update = std::chrono::system_clock::now();
			}


			if (last_orientationStatus > orientationStatus)
			{
				switch (orientationStatus)
				{
				case ORIENT_UNKNOWN:
					ps_cancel_condition(IMU_DATA);
					ps_cancel_condition(HEADING_DR);
					ps_cancel_condition(HEADING_GOOD);
					break;
				case ORIENT_RELATIVE:
					ps_cancel_condition(HEADING_DR);
					ps_cancel_condition(HEADING_GOOD);
					break;
				case ORIENT_DEADRECKONED:
					ps_cancel_condition(HEADING_GOOD);
					break;
				default:
					break;
				}
			}
			else if (last_orientationStatus < orientationStatus)
			{
				switch (orientationStatus)
				{
				case ORIENT_RELATIVE:
					ps_set_condition(IMU_DATA);
					break;
				case ORIENT_DEADRECKONED:
					ps_set_condition(IMU_DATA);
					ps_set_condition(HEADING_DR);
					break;
				case ORIENT_ABS:
					ps_set_condition(IMU_DATA);
					ps_set_condition(HEADING_DR);
					ps_set_condition(HEADING_GOOD);
					break;
				default:
					break;
				}
			}

			last_orientationStatus = orientationStatus;

			if (last_locationStatus > locationStatus)
			{
				switch(locationStatus)
				{
				case LOCATION_UNKNOWN:
					ps_cancel_condition(GPS_FIX);
					ps_cancel_condition(DGPS_FIX);
					ps_cancel_condition(GPS10_FIX);
					break;
				case LOCATION_VALID:
					ps_cancel_condition(DGPS_FIX);
					ps_cancel_condition(GPS10_FIX);
					break;
				case LOCATION_DGPS:
					ps_cancel_condition(GPS10_FIX);
					break;
				default:
					break;
				}
			}
			else if(last_locationStatus < locationStatus)
			{
				switch(locationStatus)
				{
				case LOCATION_UNKNOWN:
					break;
				case LOCATION_VALID:
					ps_set_condition(GPS_FIX);
					break;
				case LOCATION_DGPS:
					ps_set_condition(GPS_FIX);
					ps_set_condition(DGPS_FIX);
					break;
				case LOCATION_10CM:
					ps_set_condition(GPS_FIX);
					ps_set_condition(DGPS_FIX);
					ps_set_condition(GPS10_FIX);
					break;
				default:
					break;
				}
			}

			last_locationStatus = locationStatus;

			usleep(NAV_LOOP_MS * 1000);
		}
	} catch (std::exception &e) {
		PS_ERROR("nav: thread exception: %s", e.what());
	}
}

void Navigator::robot_moving(bool moving)
{
	if (moving)
	{
		start_averaging = current_location;
		measured_heading_total = measured_heading_count = 0;
	}
	else
	{
		average_heading = (int) start_averaging.bearing_to_point(current_location);
		distance_run = (int) start_averaging.mm_range_to_point(current_location);
		if (measured_heading_count) measured_heading = measured_heading_total / measured_heading_count;
		DEBUGPRINT("nav: Run of %icm @ %i (%i)", distance_run/10, average_heading, measured_heading);
	}
}

NavigatorClass& the_navigator()
{
	return the_navigator_instance();
}

Navigator& the_navigator_instance()
{
	static Navigator nav;
	return nav;
}

const char *navActionList[] = NAV_ACTION_LIST;
