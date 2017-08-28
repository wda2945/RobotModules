/*
 * scriptloader.cpp
 *
 *  Created on: Aug 13, 2014
 *      Author: martin
 */
//loads scripts into the LUA environment

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

#include <dirent.h>
#include <string.h>
#include <mutex>

#include "ScriptLoaderClass.hpp"

#include "robot.h"

class ScriptLoader : public ScriptLoaderClass
{
public:
	ScriptLoader(lua_State *L);	//auto loads BT framework

	int LoadFromFolder(const char *scriptFolder) override;

private:

	lua_State *luaState;
	int LoadBehaviorTree();

	int LoadFromFile(const char *name, const char *path, int nargs);
	int LoadChunk(const char *name, const char *path);
};

