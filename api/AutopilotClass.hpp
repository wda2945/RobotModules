/*
 * AutopilotClass.hpp
 *
 *  Created on: 2015
 *      Author: martin
 */


#ifndef AUTOPILOT_CLASS_H_
#define AUTOPILOT_CLASS_H_

#include "BehaviorClass.hpp"
#include "mapping/location.hpp"

class AutopilotClass
{
public:
	AutopilotClass() {}
	virtual ~AutopilotClass() {}

	virtual ActionResult_enum pilotSetGoalWaypoint(const char *waypointName) = 0;
	virtual ActionResult_enum pilotSetGoalPosition(Location _goal) = 0;
	virtual ActionResult_enum pilotSetRelativeGoal(int _rangeCM, int _bearingDegrees) = 0;
	virtual ActionResult_enum pilotSetRandomGoal(int _rangeCM) = 0;
	virtual ActionResult_enum pilotSelectRandomWayopint() = 0;
	virtual ActionResult_enum pilotIsAtWaypoint() = 0;

	virtual ActionResult_enum WaitUntilReadyToMove() = 0;
	virtual ActionResult_enum IsReadyToMove() = 0;

};

AutopilotClass& the_autopilot();

#endif
