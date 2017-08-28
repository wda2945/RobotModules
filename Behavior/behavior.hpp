/*
 * behavior.h
 *
 *  Created on: Jul 11, 2014
 *      Author: martin
 */

#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <string>
#include <thread>
#include <mutex>

#include "robot.h"

#include "BehaviorClass.hpp"

#include "scriptplayer.hpp"
#include "scriptloader.hpp"

#include "mapping/location.hpp"

#include "lua.h"

class Behaviors : public BehaviorClass
{

	Behaviors();
	virtual ~Behaviors() {}

public:

	//register lua globals & callback functions
	int register_lua_global(const char *name, int value) override;
	int register_lua_global(const char *name, float value) override;
	int register_lua_callback(const char *function_name, luaCallback_t callback) override;

private:
	std::thread *behavior_tree_thread;
	std::mutex luaMtx;

	bool APPonline;
	time_t lastAPPresponseTime;

	int init_scripts();

	void ScriptThread();

	void ProcessMessage(const void *_msg, int len);

	friend void BehaviorProcessMessage(const void *_msg, int len);
	friend Behaviors& the_behaviors_instance();
};

Behaviors& the_behaviors_instance();

#endif /* BEHAVIOR_H_ */
