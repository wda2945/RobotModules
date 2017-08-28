/*
 * MotorClass.hpp
 *
 *  Created on: Jan 29, 2017
 *      Author: martin
 */

#ifndef MOTORS_CLASS_H_
#define MOTORS_CLASS_H_

#include "messages.h"
#include "BehaviorClass.hpp"
#include "ProximityClass.hpp"

class MotorClass
{
protected:
	MotorClass() {}
	virtual ~MotorClass() {}

public:
	
	bool commandRunning {false};

	virtual ActionResult_enum set_proximity_mask_bits(ProxSectorMask_enum mask) = 0;
	virtual ActionResult_enum clear_proximity_mask_bits(ProxSectorMask_enum mask) = 0;

	virtual ActionResult_enum enable_system_stop() = 0;
	virtual ActionResult_enum disable_system_stop() = 0;

	virtual ActionResult_enum Turn(int degrees, bool adjust) = 0;
	virtual ActionResult_enum TurnToHeading(unsigned int bearing) = 0;

	virtual ActionResult_enum Move(int mm, bool adjust) = 0;
	virtual ActionResult_enum Move_towards(unsigned int heading, unsigned int mm, bool adjust) = 0;

	virtual ActionResult_enum set_motor_speeds(unsigned int x, unsigned int zRotation) = 0;

	virtual ActionResult_enum cancel_action() = 0;

	virtual motorResponseFlags_enum get_motor_status() = 0;
	
	virtual ActionResult_enum is_motor_ready() = 0;
	virtual ActionResult_enum test_fail_reason(motorResponseFlags_enum flag) = 0;

	virtual char * fail_reason(motorResponseFlags_enum flag) = 0;

	//odometry

	virtual int get_movement(int *x, int *y, int *z) = 0;	//since last call

	int heading_error {0};
	
};

MotorClass& the_motors();

#endif	// MOTOR_CLASS_H_
