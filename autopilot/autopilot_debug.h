/*
 * autopilot_common.h
 *
 *  Created on: 2015
 *      Author: martin
 */

#ifndef AUTOPILOT_COMMON_H_
#define AUTOPILOT_COMMON_H_

#include "debug.h"

extern FILE *pilotDebugFile;

#ifdef AUTOPILOT_DEBUG
#define DEBUGPRINT(...) tprintf( __VA_ARGS__);tfprintf(pilotDebugFile, __VA_ARGS__);
#else
#define DEBUGPRINT(...) tfprintf(pilotDebugFile, __VA_ARGS__);
#endif

#define ERRORPRINT(...) tprintf( __VA_ARGS__);tfprintf(pilotDebugFile, __VA_ARGS__);


#endif
