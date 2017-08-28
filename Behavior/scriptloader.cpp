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

#include "ScriptLoader.hpp"

#include "robot.h"

#include "behavior/behavior_debug.h"

//private
const char * ChunkReader(lua_State *L, void *data, size_t *size);

ScriptLoader::ScriptLoader(lua_State *L)
{
	luaState = L;
	LoadBehaviorTree();

	LoadFromFolder(INIT_SCRIPT_PATH);	//load init/default scripts first
	LoadFromFolder(BT_LEAF_PATH);
	LoadFromFolder(BT_ACTIVITY_PATH);
	LoadFromFolder(HOOK_SCRIPT_PATH);
	LoadFromFolder(GENERAL_SCRIPT_PATH);
}

//----------------------------------------LOADING LUA SCRIPTS
int ScriptLoader::LoadBehaviorTree()
{
	int reply;

	reply = LoadFromFile("btclass",  BEHAVIOR_TREE_CLASS, 1);
	if (reply == 0)
	{
		 lua_setglobal(luaState, "BT");
	}
	return reply;
}

//load all lua scripts in a folder
int ScriptLoader::LoadFromFolder(const char *scriptFolder)	//load all scripts in folder
{
	DIR *dp;
	struct dirent *ep;

	DEBUGPRINT("Script Folder: %s", scriptFolder);

	dp = opendir (scriptFolder);
	if (dp != NULL) {
		while ((ep = readdir (dp))) {

			DEBUGPRINT("File: %s", ep->d_name);

			//check whether the file is a lua script
			int len = strlen(ep->d_name);
			if (len < 5) continue;						//name too short

			char *suffix = ep->d_name + len - 4;
			if (strcmp(suffix, ".lua") != 0)
			{
				continue;	//wrong extension
			}

			char path[99];
			sprintf(path, "%s/%s", scriptFolder, ep->d_name);

			char *name = ep->d_name;
			name[len - 4] = '\0'; 						//chop off .lua suffix for name

			if (LoadFromFile(name, path, 0) < 0)
			{
				(void) closedir (dp);
				return -1;
			}
		}
		(void) closedir (dp);
	}
	DEBUGPRINT("Script Folder: %s done", scriptFolder);

	return 0;
}

//load script from a file
int ScriptLoader::LoadFromFile(const char *name, const char *path, int nargs)
{
	int loadReply = LoadChunk(name, path);

	if (loadReply == LUA_OK)
	{
		//now call the chunk to initialize itself
		int status = lua_pcall(luaState, 0, nargs, 0);

		if (status != LUA_OK)
		{
			const char *errormsg = lua_tostring(luaState, -1);
			ERRORPRINT("Error: %s",errormsg);
			return -1;
		}
		else
		{
			DEBUGPRINT("loaded %s", name);
			return 0;
		}
	}
	else
	{
		ERRORPRINT("failed to load %s - %i", name, loadReply);
		return -1;
	}
}

//load a lua chunk from a file
std::mutex chunkFileMtx;
FILE *chunkFile = nullptr;

int ScriptLoader::LoadChunk(const char *name, const char *path)
{
	int loadReply;

	std::unique_lock<std::mutex> lck {chunkFileMtx};

	chunkFile = fopen(path, "r");
	DEBUGPRINT("Loading %s: %s (0x%x)",name, path, (uint32_t) chunkFile);

	if (!chunkFile)
	{
		ERRORPRINT("Failed to open %s",path);
		return -1;
	}

	loadReply = lua_load(luaState, ChunkReader, (void*) 0, name, "bt");

	fclose(chunkFile);
	chunkFile = nullptr;

	if  (loadReply == LUA_ERRSYNTAX)
	{
		const char *errormsg = lua_tostring(luaState, -1);
		ERRORPRINT("lua: %s syntax error: %s", name, errormsg);
		return -1;
	}
	else if (loadReply != LUA_OK)
	{
		ERRORPRINT("lua_load of %s fail: %i", path, loadReply);
		return -1;
	}
	return 0;
}

//file reader call out
char buffer[100];
const char * ChunkReader(lua_State *luaState, void *data, size_t *size)
{
	int c;
	char *next = buffer;
	*size = 0;
	do {
		c = getc(chunkFile);
		if (c != EOF)
		{
			*next++ = c;
			(*size)++;
		}
		else
		{
			if (*size == 0) return NULL;
		}
	} while  (c != EOF && *size < 100);
	return buffer;
}
