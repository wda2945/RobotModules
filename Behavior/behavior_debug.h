/*
 * behaviorDebug.h
 *
 *      Author: martin
 */

#ifndef BEHAVIORDEBUG_H_
#define BEHAVIORDEBUG_H_

#include "debug.h"

extern FILE *behDebugFile;

#ifdef BEHAVIOR_DEBUG
#define DEBUGPRINT(...) tprintf( __VA_ARGS__);tfprintf(behDebugFile, __VA_ARGS__);
#else
#define DEBUGPRINT(...) tfprintf(behDebugFile, __VA_ARGS__);
#endif

#define ERRORPRINT(...) tprintf( __VA_ARGS__);tfprintf(behDebugFile, __VA_ARGS__);

#define ASSERT_LUA_TABLE(L, x, s) if (!lua_istable(L, x)) {ERRORPRINT("Not a LUA Table : %s", s); abort();}

#define ASSERT_LUA_STRING(L, x, s) if (!lua_isstring(L, x)) {ERRORPRINT("Not a LUA String : %s", s); abort();}

#define ASSERT_LUA_NUMBER(L, x, s) if (!lua_isnumber(L, x)) {ERRORPRINT("Not a LUA Number : %s", s); abort();}





#endif
