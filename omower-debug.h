// Debug and console control (UART) routines for Omower
// $Id$

#ifndef _DEBUG_OMOWER_H
#define _DEBUG_OMOWER_H

#include <Arduino.h>
#include <stdint.h>

// Size of receive buffer
#define _READ_BUFFER_SIZE 256
// Size of transmit buffer
#define _WRITE_BUFFER_SIZE 2048

#define L_DEBUG 0
#define L_NOTICE 1
#define L_INFO 2
#define L_WARNING 3
#define L_ERROR 4

// disable debug routine
extern volatile boolean debugDisable;

// current debug level
extern uint8_t debugLevel;

void debug(uint8_t level, const char *fmt, ...);

// Console read/write functions
// Returns number of available characters in receive buffer
uint16_t consAvailable();

// Returns status of transfer (true - busy)
boolean consSendBusy();

// Non-blocking read of receive buffer (returns number of characters read)
uint16_t consRead(uint8_t *buf, uint16_t length);

// Non-blocking write to sending buffer (returns number of characters wrote)
uint16_t consWrite(uint8_t *buf, uint16_t length);

// Initialize and set baud speed for the console
void consInit(uint32_t baud);

#endif
