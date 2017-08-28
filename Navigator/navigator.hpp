/*
 * navigator.hpp
 *
 *      Author: martin
 */

#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP

#include <thread>
#include <chrono>

#include "NavigatorClass.hpp"

#include "GPSClass.hpp"
#include "IMUClass.hpp"

using namespace std::chrono;

//LUA leaf actions for navigation

typedef enum {
	AssumeHome,
	AssumeN,
	OrientationValid,
	OrientationDR,
	OrientationAbsolute,
	LocationValid,
	LocationDGPS,
	Location10cm,
	NAV_ACTION_COUNT
} NavAction_enum;

#define NAV_ACTION_LIST {\
		"Navigator_AssumeHome",\
		"Navigator_AssumeN",\
		"Navigator_isOrientationValid",\
		"Navigator_isOrientationDR",\
		"Navigator_isOrientationAbsolute",\
		"Navigator_isLocationValid",\
		"Navigator_isLocationDGPS",\
		"Navigator_isLocation10cm",\
}

class Navigator : public NavigatorClass
{
public:

	void robot_moving(bool moving) override;	//motion averaging

protected:

	ActionResult_enum Action(NavAction_enum _action);

	ActionResult_enum assume_heading(int _heading) override;
	ActionResult_enum assume_location(char *waypoint) override;

	bool gpsLocationValid		{false};
	bool gpsSpeedValid 			{false};

	bool imu_online 			{false};	//geting data
	bool using_imu_heading 		{false};	//no gps heading
	bool using_assumed_heading	{false};

	bool gps_online 			{false};	//getting gps data
	bool gps_fix				{false};	//gps 3D fix

	int imu_heading 			{0};		//degrees
	int gps_heading 			{0};		//degrees
	int imu_offset 				{0};		//degrees

	Location last_location_logged;

	Location start_averaging;
	int measured_heading_total	{0};
	int measured_heading_count	{0};

	uint8_t		last_min {0};

	Navigator();
	~Navigator(){}

	void navigator_thread_method();

	std::thread *nav_thread;

	std::chrono::system_clock::time_point last_registry_update;

	friend Navigator& the_navigator_instance();
	friend int luaNavAction(lua_State *L);
};

Navigator& the_navigator_instance();

extern const char *navActionList[];

#endif
