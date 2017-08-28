//
//  nav_debug.h
//
//  Created by Martin Lane-Smith on 5/18/16.
//  Copyright © 2016 Martin Lane-Smith. All rights reserved.
//

#ifndef nav_debug_h
#define nav_debug_h

#include "debug.h"

extern FILE *navDebugFile;

#ifdef NAVIGATOR_DEBUG
#define DEBUGPRINT(...) tprintf( __VA_ARGS__);tfprintf(navDebugFile, __VA_ARGS__);
#else
#define DEBUGPRINT(...) tfprintf(navDebugFile, __VA_ARGS__);
#endif

#define ERRORPRINT(...) tprintf(__VA_ARGS__);tfprintf(navDebugFile, __VA_ARGS__);

#endif /* nav_debug_h */
