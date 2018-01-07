// Modbus stuff for OMower
// $Id$

#ifndef _MODBUS_OMOWER
#define _MODBUS_OMOWER

#include <stdint.h>

// Calculate CRC for modbus frame                                                                             
uint16_t modbusCrc(char *buf, int len);

// Retrieve 16-bit value from modbus frame
uint16_t getModbus16(char *addr);

// Retrieve 32-bit value from modbus frame (two 16bit registers joined)
uint32_t getModbus32(char *addr);

#endif
