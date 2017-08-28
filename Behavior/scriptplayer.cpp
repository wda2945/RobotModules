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

#include "ScriptPlayer.hpp"

#include "BehaviorClass.hpp"

#include "behavior/behavior_debug.h"

#include "AutopilotClass.hpp"
#include "mapping/waypoints.hpp"

using namespace std;

//registry
static int GetRegistry(lua_State *L);
static int SetRegistry(lua_State *L);
static int ChangeOption(lua_State *L);
static int ChangeSetting(lua_State *L);

//logging
static int Print(lua_State *L);				//Print("...")
static int Debug(lua_State *L);				//Print("...")
static int Alert(lua_State *L);
static int Fail(lua_State *L);

const char *actionResultNames[] = ACTION_RESULT_NAMES;
const char *behaviorStatusNames[] = BEHAVIOR_STATUS_NAMES;

ScriptPlayer::ScriptPlayer(lua_State *L)
{
	btLuaState = L;

	ps_registry_add_new("Behavior Status", "Status", PS_REGISTRY_TEXT_TYPE, PS_REGISTRY_SRC_WRITE);
	ps_registry_set_text("Behavior Status", "Status", "idle");

	luaopen_math(btLuaState);
	lua_setglobal(btLuaState, "math");
	luaopen_string(btLuaState);
	lua_setglobal(btLuaState, "string");

	//registry callbacks
	lua_pushcfunction(btLuaState, GetRegistry);			//access registry
	lua_setglobal(btLuaState, "GetRegistry");
	lua_pushcfunction(btLuaState, SetRegistry);			//access registry
	lua_setglobal(btLuaState, "SetRegistry");
	lua_pushcfunction(btLuaState, ChangeOption);
	lua_setglobal(btLuaState, "ChangeOption");
	lua_pushcfunction(btLuaState, ChangeSetting);
	lua_setglobal(btLuaState, "ChangeSetting");

	//register basic call-backs
	lua_pushcfunction(btLuaState, Alert);				//Alert Message to App
	lua_setglobal(btLuaState, "Alert");
	lua_pushcfunction(btLuaState, Print);				//Print Message
	lua_setglobal(btLuaState, "Print");
	lua_pushcfunction(btLuaState, Debug);				//Debug Message
	lua_setglobal(btLuaState, "Debug");
	lua_pushcfunction(btLuaState, Fail);				//Note fail() - Fail('name')
	lua_setglobal(btLuaState, "Fail");

	lua_pop(btLuaState, lua_gettop( btLuaState));		//clean stack
}

int ScriptPlayer::register_lua_global(const char *name, int value)
{
	lua_pushinteger(btLuaState, value);
	lua_setglobal(btLuaState, name);
	return 0;
}

int ScriptPlayer::register_lua_global(const char *name, float value)
{
	lua_pushnumber(btLuaState, value);
	lua_setglobal(btLuaState, name);
	return 0;
}

//register lua callback function - generic
int ScriptPlayer::register_lua_callback(const char *function_name, luaCallback_t callback)
{
	lua_pushcfunction(btLuaState, callback);
	lua_setglobal(btLuaState, function_name);
	return 0;
}

void ScriptPlayer::status_update(BehaviorStatus_enum stat, const char *failAt, const char *failReason)
{
	char status[REGISTRY_TEXT_LENGTH];

	snprintf(status, REGISTRY_TEXT_LENGTH, "%s %s %s %s",
			the_behaviors().behaviorName.c_str(),
			behaviorStatusNames[stat],
			failAt,
			failReason);

	ps_registry_set_text("Behavior Status", "Status", status);
}

//messages processed by scripting system
int ScriptPlayer::ActivateBehavior(const char *scriptName)
{
	if (!btLuaState) return -1;

	//critical section
	std::unique_lock<std::mutex> lck {lua_state_mutex};

#define GOTO "Go To "

		//start BT activity
		DEBUGPRINT("Activate: %s", scriptName);
		const char *BTname = scriptName;

		if (strstr(scriptName, GOTO) != NULL)
		{
			if (the_autopilot().pilotSetGoalWaypoint(scriptName + strlen(GOTO)) != ACTION_SUCCESS)
			{
				ERRORPRINT("script: waypoint %s not found", scriptName + strlen(GOTO));
				return 0;
			}
			BTname = (const char*)"GoTo";
		}


		lua_getglobal(btLuaState, "activate");						//reference to 'activate(...)
		if (lua_isfunction(btLuaState,lua_gettop(btLuaState)))
		{
			//activate() exists
			lua_pushstring(btLuaState, BTname);		//name of target BT
			lua_getglobal(btLuaState, BTname);		//reference to the tree itself
			if (lua_istable(btLuaState,lua_gettop(btLuaState)))
			{
				//target table exists - call activate()
				int status = lua_pcall(btLuaState, 2, 0, 0);
				if (status)
				{
					const char *errormsg = lua_tostring(btLuaState, -1);
					LogError("%s, Error: %s",scriptName,errormsg);
				}
				else
				{
					the_behaviors().behaviorName = string(scriptName);
					the_behaviors().behaviorStatus = BEHAVIOR_ACTIVE;
					status_update(BEHAVIOR_ACTIVE, "", "");
					LogInfo("Activate: %s", scriptName);

					SPEAK("Activating script %s", scriptName);
				}
			}
			else
			{
				//target table does not exist
				ERRORPRINT("%s is type %s",BTname,lua_typename(btLuaState,lua_type(btLuaState,lua_gettop(btLuaState))) );
			}
		}
		else
		{
			//activate() does not exist
			ERRORPRINT("activate is type %s",lua_typename(btLuaState,lua_type(btLuaState,lua_gettop(btLuaState))) );
		}
		lua_pop(btLuaState, lua_gettop( btLuaState));

	return 0;
}

//periodic behavior tree update invocation
int ScriptPlayer::InvokePeriodicUpdate()
{
	if (!btLuaState) return -1;

	//critical section
	std::unique_lock<std::mutex> lck {lua_state_mutex};

	switch (the_behaviors().behaviorStatus)
	{
	case BEHAVIOR_ACTIVE:
	case BEHAVIOR_RUNNING:

		lua_getglobal(btLuaState, "update");

		if (lua_isfunction(btLuaState,lua_gettop(btLuaState)))
		{
			//		DEBUGPRINT("LUA: calling update");
			int reply = lua_pcall(btLuaState, 0, 1, 0);
			if (reply)
			{
				const char *errormsg = lua_tostring(btLuaState, -1);
				ERRORPRINT("Script update, Error: %s",errormsg);
				lua_pop(btLuaState, lua_gettop( btLuaState));

				status_update(BEHAVIOR_INVALID, "update", "");
				the_behaviors().behaviorStatus = BEHAVIOR_INVALID;

				return -1;
			}
			else
			{
				int i;
				size_t len;
				const char *status = lua_tolstring(btLuaState,1, &len);

//				DEBUGPRINT("LUA: returned from update (%s)", status);

				BehaviorStatus_enum statusCode = BEHAVIOR_INVALID;
				for ( i=0; i<BEHAVIOR_STATUS_COUNT; i++)
				{
					if (strncmp(behaviorStatusNames[i], status, 4) == 0)
					{
						statusCode = (BehaviorStatus_enum) i;
						break;
					}
				}

				if ((statusCode != the_behaviors().behaviorStatus) || (the_behaviors().behaviorName.compare(the_behaviors().lastActivityName) != 0))
				{
					//change in activity or status

//					status_update(statusCode, "update", "");

					switch (statusCode)
					{
					case BEHAVIOR_SUCCESS:
						LogInfo("%s SUCCESS", the_behaviors().behaviorName.c_str());
						DEBUGPRINT("%s SUCCESS", the_behaviors().behaviorName.c_str());
						status_update(BEHAVIOR_SUCCESS, "", "");

						SPEAK("Behavior %s completed successfully.", the_behaviors().behaviorName.c_str());

						break;
					case BEHAVIOR_RUNNING:
						status_update(BEHAVIOR_RUNNING, "", "");
						break;
					case BEHAVIOR_FAIL:
						LogInfo("%s FAIL @ %s - %s", the_behaviors().behaviorName.c_str(), the_behaviors().lastLuaCallFail.c_str(), the_behaviors().lastLuaCallReason.c_str());
						DEBUGPRINT("%s FAIL @ %s - %s", the_behaviors().behaviorName.c_str(), the_behaviors().lastLuaCallFail.c_str(), the_behaviors().lastLuaCallReason.c_str());
						status_update(BEHAVIOR_FAIL,
								the_behaviors().lastLuaCallFail.c_str(),
								the_behaviors().lastLuaCallReason.c_str());

						std::this_thread::sleep_for(std::chrono::milliseconds(500));
						SPEAK("Script %s failed. Sorry.", the_behaviors().behaviorName.c_str());

						break;
					case BEHAVIOR_ACTIVE:
					case BEHAVIOR_INVALID:
					default:
						LogError("%s Bad Update Response", the_behaviors().behaviorName.c_str());
						ERRORPRINT("%s Bad Update Response", the_behaviors().behaviorName.c_str());
						status_update(statusCode, "update", "");
						break;
					}

					the_behaviors().behaviorStatus = statusCode;
				}

				lua_pop(btLuaState, lua_gettop( btLuaState));

				the_behaviors().lastActivityName = the_behaviors().behaviorName;
				return 0;
			}
		}
		else
		{
			ERRORPRINT("Global update is of type %s",lua_typename(btLuaState,lua_type(btLuaState,lua_gettop(btLuaState))) );
			lua_pop(btLuaState, lua_gettop( btLuaState));
			return -1;
		}
		break;
	default:
		break;
	}
	return 0;
}

//report available activity scripts
int ScriptPlayer::AvailableBehaviors(scriptCallback_t callback)
{
	if (!btLuaState) return -1;

	//critical section
	std::unique_lock<std::mutex> lck {lua_state_mutex};

	//look up activities table
	lua_getglobal(btLuaState, "ActivityList");
	int table = lua_gettop( btLuaState);

	if (lua_istable(btLuaState, table) == 0)
	{
		//not a table
		LogError("No Activities table" );
		lua_pop(btLuaState, lua_gettop( btLuaState));
	}

	int index = 0;

    lua_pushnil(btLuaState);  /* first key */
    while (lua_next(btLuaState, table) != 0) {
      /* uses 'key' (at index -2) and 'value' (at index -1) */

    	const char *scriptName =  lua_tostring(btLuaState, -1);

    	(*callback)(scriptName);

    	index++;

      /* removes 'value'; keeps 'key' for next iteration */
      lua_pop(btLuaState, 1);
    }
	lua_pop(btLuaState, lua_gettop( btLuaState));

	//now add all waypoints

	int count = the_waypoints().waypoints_size();
	char script[50];
	for (int wp = 0; wp < count; wp++)
	{
		string name = the_waypoints()[wp].name();
		if (name[0] != '_')
		{
			snprintf(script, 50, "%s%s", GOTO, name.c_str());
	    	ps_registry_add_new("Behaviors", script, PS_REGISTRY_TEXT_TYPE, PS_REGISTRY_READ_ONLY);
	    	ps_registry_set_text("Behaviors", script, script);
		}
	}

	LogRoutine("%i activities, %i waypoints", index, count);
	return 0;
}

//call-backs

static int GetRegistry(lua_State *L)		//GetRegistry('domain', 'name')
{
	const char *domain = lua_tostring(L,1);
	const char *name = lua_tostring(L,2);

    ps_registry_datatype_t type = ps_registry_get_type(domain, name);

    switch(type)
    {
    case PS_REGISTRY_ANY_TYPE:
    default:
    	return 0;
    	break;
    case PS_REGISTRY_INT_TYPE:
    {
    	int value;
    	if (ps_registry_get_int(domain, name, &value) != PS_OK) return 0;
    	lua_pushinteger(L, value);
    	return 1;
    }
    break;
    case PS_REGISTRY_REAL_TYPE:
    {
    	float value;
    	if (ps_registry_get_real(domain, name, &value) != PS_OK) return 0;
    	lua_pushnumber(L, value);
    	return 1;
    }
    break;
    case PS_REGISTRY_TEXT_TYPE:
    {
    	char value[100];
    	if (ps_registry_get_text(domain, name, value, 100) != PS_OK) return 0;
    	lua_pushstring(L, value);
    	return 1;
    }
    break;
    case PS_REGISTRY_BOOL_TYPE:
    {
    	bool value;
    	if (ps_registry_get_bool(domain, name, &value) != PS_OK) return 0;
    	lua_pushboolean(L, value);
    	return 1;
    }
    break;
    }
}

static int SetRegistry(lua_State *L)		//SetRegistry('domain', 'name', value)
{
	const char *domain = lua_tostring(L,1);
	const char *name = lua_tostring(L,2);

    ps_registry_datatype_t type = ps_registry_get_type(domain, name);

    switch(type)
    {
    case PS_REGISTRY_ANY_TYPE:
    default:
    	return 0;
    	break;
    case PS_REGISTRY_INT_TYPE:
    {
    	int value = lua_tointeger(L,3);
    	ps_registry_set_int(domain, name, value);
    	return 0;
    }
    break;
    case PS_REGISTRY_REAL_TYPE:
    {
    	float value = lua_tonumber(L,3);
    	ps_registry_set_real(domain, name, value);
    	return 0;
    }
    break;
    case PS_REGISTRY_TEXT_TYPE:
    {
    	const char *value = lua_tostring(L, 3);;
    	ps_registry_set_text(domain, name, value);
    	return 0;
    }
    break;
    case PS_REGISTRY_BOOL_TYPE:
    {
    	bool value = lua_toboolean(L,3);
    	ps_registry_set_bool(domain, name, value);
    	return 0;
    }
    break;
    }
}


static int ChangeOption(lua_State *L)
{
	const char *name = lua_tostring(L,1);
	int value = lua_tointeger(L,2);

	if (ps_registry_set_bool("Options", name, value) == PS_OK)
	{
		return success(L);
	}
	else
	{
		return fail(L);
	}
}

static int ChangeSetting(lua_State *L)
{
	const char *name = lua_tostring(L,1);
	float value = lua_tonumber(L,2);

	if (ps_registry_set_real("Settings", name, value) == PS_OK)
	{
		return success(L);
	}
	else
	{
		return fail(L);
	}
}

//Alert
static int Alert(lua_State *L)				//Alert("...")
{
	const char *text = lua_tostring(L,1);

	psMessage_t msg;
	psInitPublish(msg, ALERT);
	strncpy(msg.namePayload.name, text, PS_NAME_LENGTH);
	RouteMessage(msg);
	LogInfo("lua: Alert (%s)", text);

	return 0;
}

//print
static int Print(lua_State *L)				//Print("...")
{
	const char *text = lua_tostring(L,1);
	DEBUGPRINT("lua: %s",text);
	return 0;
}
static int Debug(lua_State *L)				//Print("...")
{
	const char *text = lua_tostring(L,1);
	DEBUGPRINT("lua: %s",text);
	return 0;
}

//Fail
char failBuffer[PS_NAME_LENGTH];
static int Fail(lua_State *L)				//Fail('name')
{
	const char *name = lua_tostring(L,1);

	if (name)
	{
		strncpy(failBuffer, name, PS_NAME_LENGTH);
		the_behaviors().lastLuaCallReason = failBuffer;
	}

	LogInfo("lua: fail at %s",name);
	return 0;
}

