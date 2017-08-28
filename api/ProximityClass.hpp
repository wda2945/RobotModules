/*
 * ProximityClass.hpp
 *
 *  Created on: Jan 29, 2017
 *      Author: martin
 */

#ifndef PROXIMITY_CLASS_H_
#define PROXIMITY_CLASS_H_

#include "BehaviorClass.hpp"

typedef uint8_t ProxSectorMask_enum;
const ProxSectorMask_enum PROX_FRONT_CENTER_MASK = 0x1;
const ProxSectorMask_enum PROX_FRONT_RIGHT_MASK = 0x2;
const ProxSectorMask_enum PROX_RIGHT_MASK = 0x4;
const ProxSectorMask_enum PROX_REAR_RIGHT_MASK = 0x8;
const ProxSectorMask_enum PROX_REAR_CENTER_MASK = 0x10;
const ProxSectorMask_enum PROX_REAR_LEFT_MASK = 0x20;
const ProxSectorMask_enum PROX_LEFT_MASK = 0x40;
const ProxSectorMask_enum PROX_FRONT_LEFT_MASK = 0x80;

const ProxSectorMask_enum PROX_FRONT_MASK = 0x83;
const ProxSectorMask_enum PROX_REAR_MASK = 0x38;

typedef uint8_t ProxStatusMask_enum;
const ProxStatusMask_enum 	PROX_OFFLINE_MASK = 0x1;
const ProxStatusMask_enum 	PROX_ACTIVE_MASK = 0x2;
const ProxStatusMask_enum 	PROX_ERRORS_MASK = 0x4;
const ProxStatusMask_enum 	PROX_FAR_MASK = 0x10;
const ProxStatusMask_enum 	PROX_CLOSE_MASK = 0x20;

class ProximityClass
{
public:
	ProximityClass() {}
	virtual ~ProximityClass() {}

	virtual ActionResult_enum proximityStatus(ProxSectorMask_enum _sectors,  ProxStatusMask_enum _status) {return ACTION_FAIL;}

};

ProximityClass& proximity();

#endif	// PROXIMITY_H_
