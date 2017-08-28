//
//  behavior.c
//
//  Created by Martin Lane-Smith on 6/14/14.
//  Copyright (c) 2014 Martin Lane-Smith. All rights reserved.
//
// Root threads of the scripted behavior subsystem

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#include "behavior.hpp"
#include "behavior_debug.h"

FILE *behDebugFile;

//allocator for lua state
static void *l_alloc (void *ud, void *ptr, size_t osize, size_t nsize)
{
	(void)ud;  (void)osize;  /* not used */
	if (nsize == 0)
	{
		free(ptr);
		return NULL;
	}
	else
		return realloc(ptr, nsize);
}

Behaviors *behaviors {nullptr};
void BehaviorProcessMessage(const void *_msg, int len)
{
	if (behaviors)
		behaviors->ProcessMessage(_msg, len);
}

void script_callback(const char *scriptName)
{
	ps_registry_add_new("Behaviors", scriptName, PS_REGISTRY_TEXT_TYPE, PS_REGISTRY_READ_ONLY);
	ps_registry_set_text("Behaviors", scriptName, scriptName);
}

int Behaviors::init_scripts()
{
	if (player) delete player;
	if (loader) delete loader;

	// delete lua state
	if (btLuaState) lua_close(btLuaState);

	btLuaState 	= nullptr;
	player		= nullptr;
	loader		= nullptr;

	//create a new LUA state
	btLuaState = lua_newstate(l_alloc, NULL);

	if (btLuaState == NULL)
	{
		ERRORPRINT("luaState create fail");
		throw("luaState create fail");
	}

	//open standard libraries
	luaL_openlibs(btLuaState);

	player = new ScriptPlayer(btLuaState);
	loader = new ScriptLoader(btLuaState);

	player->AvailableBehaviors(&script_callback);

	DEBUGPRINT("beh: BT ready");

	return 0;
}

Behaviors::Behaviors()
{
	behDebugFile = fopen_logfile("behavior");

	//lock Lua until init complete
	luaMtx.lock();

	//Create thread
	behavior_tree_thread = new std::thread([this](){ScriptThread();});
}

int Behaviors::register_lua_global(const char *name, int value)
{
	luaMtx.lock();
	int reply = player->register_lua_global(name, value);
	luaMtx.unlock();
	return reply;
}

int Behaviors::register_lua_global(const char *name, float value)
{
	luaMtx.lock();
	int reply = player->register_lua_global(name, value);
	luaMtx.unlock();
	return reply;
}

int Behaviors::register_lua_callback(const char *function_name, luaCallback_t callback)
{
	luaMtx.lock();
	int reply = player->register_lua_callback(function_name, callback);
	luaMtx.unlock();
	return reply;
}

void Behaviors::ProcessMessage(const void *_msg, int len)
{
	psMessage_t *msg = (psMessage_t *) _msg;

	switch (msg->messageType)
	{
	case RELOAD:
		//reload all scripts
		DEBUGPRINT("Reload scripts");
		luaMtx.lock();
		init_scripts();
		luaMtx.unlock();
		break;

	case HEEL_LOCATION:
		heel_location.latitude = (int32_t) msg->positionPayload.latitude;
		heel_location.longitude = (int32_t) msg->positionPayload.longitude;
		DEBUGPRINT("Heel location: %s", heel_location.description().c_str());
		break;

	case ACTIVATE:
		DEBUGPRINT("Activate %s", msg->namePayload.name);
		luaMtx.lock();
		player->ActivateBehavior(msg->namePayload.name);
		luaMtx.unlock();
		break;
	default:
		break;
	}
}

//thread to run scripts periodically
void Behaviors::ScriptThread()
{
	behaviors = this;
	ps_subscribe(SYS_ACTION_TOPIC, BehaviorProcessMessage);

	try
	{
		//create a new LUA state
		init_scripts();

		luaMtx.unlock();

		DEBUGPRINT("BT thread ready");

		sleep(5);

		while (1)
		{
			luaMtx.lock();
			player->InvokePeriodicUpdate();
			luaMtx.unlock();

			//delay
			usleep(behLoopDelay * 1000);
		}
	} catch (std::exception &e) {
		PS_ERROR("beh: thread exception: %s", e.what());
	}
}

//BT call-back result codes
int success(lua_State *L)
{
	lua_pushstring(L, "success");
	return 1;
}
int running(lua_State *L)
{
	lua_pushstring(L, "running");
	return 1;
}
int fail(lua_State *L)
{
	lua_pushstring(L, "fail");
	the_behaviors().lastLuaCallFail = the_behaviors().lastLuaCall;
	return 1;
}

//enum to BT response
int actionReply(lua_State *L, ActionResult_enum result)
{
	switch (result)
	{
	case ACTION_SUCCESS:
		DEBUGPRINT(".. success")
		return success(L);
		break;
	case ACTION_RUNNING:
		DEBUGPRINT(".. running")
		return running(L);
		break;
	default:
		DEBUGPRINT(".. fail")
		return fail(L);
		break;
	}
}

BehaviorClass& the_behaviors()
{
	return the_behaviors_instance();
}

Behaviors& the_behaviors_instance()
{
	static Behaviors me;
	return me;
}

