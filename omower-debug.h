// Debug routines for Omower
// $Id$

#ifndef _DEBUG_OMOWER_H
#define _DEBUG_OMOWER_H

#include <Arduino.h>
#include <stdint.h>

#define L_DEBUG 0
#define L_NOTICE 1
#define L_INFO 2
#define L_WARNING 3
#define L_ERROR 4

// debug routine is sending something
extern volatile boolean debugBusy;

// current debug level
extern uint8_t debugLevel;

void debug(uint8_t level, const char *fmt, ...);

#endif
