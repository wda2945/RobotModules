/*
 * ScriptPlayer.c
 *
 *  Created on: Aug 10, 2014
 *      Author: martin
 */
// Controller of the LUA subsystem

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <dirent.h>
#include <string.h>
#include <string>
#include <mutex>

#include "lua.hpp"

#include "robot.h"

#include "ScriptPlayerClass.hpp"

#include "BehaviorClass.hpp"

#include "AutopilotClass.hpp"
#include "mapping/waypoints.hpp"

using namespace std;

class ScriptPlayer : public ScriptPlayerClass
{
public:
	ScriptPlayer(lua_State *L);

	//register lua global
	int register_lua_global(const char *name, int value) override;
	int register_lua_global(const char *name, float value) override;

	//register lua callback function
	int register_lua_callback(const char *function_name, luaCallback_t callback) override;

	//qctivate behavior
	int ActivateBehavior(const char *scriptName) override;

	//invoke update() lua function
	int InvokePeriodicUpdate() override;

	//enumerate available behaviors
	int AvailableBehaviors(scriptCallback_t callback) override;

private:

	void status_update(BehaviorStatus_enum stat, const char *failAt, const char *failReason);

	lua_State *btLuaState;

	std::mutex lua_state_mutex;
};

