/*
 * BehaviorClass.h
 *
 *  Created on: Jul 11, 2014
 *      Author: martin
 */

#ifndef BEHAVIOR_CLASS_H_
#define BEHAVIOR_CLASS_H_

#include <string>

#include "lua.hpp"
#include "ScriptLoaderClass.hpp"
#include "ScriptPlayerClass.hpp"

#include "mapping/location.hpp"

//current behavior status
typedef enum {BEHAVIOR_INVALID, BEHAVIOR_ACTIVE, BEHAVIOR_FAIL, BEHAVIOR_RUNNING, BEHAVIOR_SUCCESS, BEHAVIOR_STATUS_COUNT} BehaviorStatus_enum;

#define BEHAVIOR_STATUS_NAMES {"invalid", "active", "fail", "running", "success"}

extern const char *behaviorStatusNames[];

//result of a BT leaf call
typedef enum {ACTION_SUCCESS, ACTION_FAIL, ACTION_RUNNING} ActionResult_enum;

#define ACTION_RESULT_NAMES {"Success", "Fail", "Running"}

extern const char *actionResultNames[];

//convenience functions to return status to lua
int success(lua_State *L);
int running(lua_State *L);
int fail(lua_State *L);

//call success/running/fail based on ActionResult_enum
int actionReply(lua_State *L, ActionResult_enum result);

class BehaviorClass
{
public:
	BehaviorClass() {}
	virtual ~BehaviorClass() {}

	std::string  behaviorName = "Idle";
	BehaviorStatus_enum behaviorStatus = BEHAVIOR_INVALID;

	std::string lastLuaCall = "none";
	std::string lastLuaCallFail = "none";
	std::string lastLuaCallReason = "none";
	std::string lastActivityName = 	"none";
	std::string lastActivityStatus = "invalid";
	
	ScriptLoaderClass *loader {nullptr};
	ScriptPlayerClass *player {nullptr};
	
	//register lua global
	virtual int register_lua_global(const char *name, int value) = 0;
	virtual int register_lua_global(const char *name, float value) = 0;

	//register lua callback function
	virtual int register_lua_callback(const char *function_name, luaCallback_t callback) = 0;

	Location heel_location;

	lua_State	*btLuaState {nullptr};
	
};

BehaviorClass& the_behaviors();

#endif /* BEHAVIOR_CLASS_H_ */
