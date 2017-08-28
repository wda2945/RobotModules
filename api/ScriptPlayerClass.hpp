/*
 * scriptplayer.hpp
 *
 *      Author: martin
 */

#ifndef SCRIPTPLAYERCLASS_H_
#define SCRIPTPLAYERCLASS_H_

#include "lua.hpp"

// #include "robot.h"
// #include "behavior_enums.h"

//callback for new lua function
typedef int (*luaCallback_t)(lua_State *L);

//callback for script enumeration
typedef void (*scriptCallback_t)(const char *scriptName);

class ScriptPlayerClass
{
public:

	ScriptPlayerClass() {}
	virtual ~ScriptPlayerClass() {}
	
	//register lua callback function - action enum
	virtual int register_lua_global(const char *name, int value) = 0;
	virtual int register_lua_global(const char *name, float value) = 0;

	//register lua callback function - generic
	virtual int register_lua_callback(const char *function_name, luaCallback_t callback) = 0;

	//qctivate behavior
	virtual int ActivateBehavior(const char *scriptName) = 0;
	
	//invoke update() lua function
	virtual int InvokePeriodicUpdate() = 0;

	//enumerate available behaviors
	virtual int AvailableBehaviors(scriptCallback_t callback) {return 0;}
	
};


#endif
