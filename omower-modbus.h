// Modbus stuff for OMower
// $Id$

#ifndef _MODBUS_OMOWER
#define _MODBUS_OMOWER

#include <stdint.h>

// Calculate CRC for modbus frame                                                                             
uint16_t modbusCrc(uint8_t *buf, int len);

// Retrieve 16-bit value from modbus frame
uint16_t getModbus16(uint8_t *addr);

// Retrieve 32-bit value from modbus frame (two 16bit registers joined)
uint32_t getModbus32(uint8_t *addr);

// Put 16-bit value to modbus frame buffer (must increase pointer +2 after)
void putModbus16(uint8_t *buf, uint16_t val);

// Put 32-bit value to modbus frame buffer (must increase pointer +4 after)
void putModbus32(uint8_t *buf, uint32_t val);

#endif
