/*
 * NavigatorClass.hpp
 *
 *      Author: martin
 */

#ifndef NAVIGATOR_CLASS_HPP
#define NAVIGATOR_CLASS_HPP

#include <chrono>

#include "mapping/location.hpp"
#include "BehaviorClass.hpp"

using namespace std::chrono;

class NavigatorClass
{
public:
	NavigatorClass() {}
	virtual ~NavigatorClass() {}

//BT leafs
	virtual ActionResult_enum assume_heading(int _heading) {return ACTION_FAIL;}
	virtual ActionResult_enum assume_location(char *waypoint) {return ACTION_FAIL;}

//status checks
	bool isOrientationValid() {return (orientationStatus >= ORIENT_RELATIVE);}
	bool isOrientationDR() {return (orientationStatus >= ORIENT_DEADRECKONED);}
	bool isOrientationAbsolute() {return (orientationStatus == ORIENT_ABS);}

	bool isLocationValid() {return (locationStatus >= LOCATION_VALID);}
	bool isLocationDGPS() {return (locationStatus >= LOCATION_DGPS);}
	bool isLocation10cm() {return (locationStatus == LOCATION_10CM);}
	bool isSpeedValid() {return gpsSpeedValid;}
	
//heading infp

	std::chrono::system_clock::time_point last_heading_update;
	
	int heading 				{0};		//degrees
	int groundSpeed 			{0};		// mm/s
	bool gpsSpeedValid			{false};

	typedef enum {
		ORIENT_UNKNOWN, 			//none
		ORIENT_RELATIVE, 			//gyro only
		ORIENT_DEADRECKONED, 		//abs + gyro D/R
		ORIENT_ABS					//from GPS
	} OrientationStatus_enum;

	OrientationStatus_enum orientationStatus = ORIENT_UNKNOWN;

	const char *orientationStatusNames[4] = {"no heading", "gyro-only","heading d/r","heading gps"};

//location info

	std::chrono::system_clock::time_point last_location_update;
	
	Location current_location;

	typedef enum {
		LOCATION_UNKNOWN,
		LOCATION_VALID, 		//GPS fix
		LOCATION_DGPS, 			//DGPS fix
		LOCATION_10CM			//<10cm error
	} LocationStatus_enum;

	LocationStatus_enum locationStatus = LOCATION_UNKNOWN;

	const char *locationStatusNames[4] = {"no fix", "gps fix", "dgps fix", "10cm fix"};

//tracking

	virtual void robot_moving(bool moving) {}	//motion averaging
	
	int average_heading 	{0};
	int distance_run 		{0};
	int measured_heading	{0};
	
};

NavigatorClass& the_navigator();

#endif
