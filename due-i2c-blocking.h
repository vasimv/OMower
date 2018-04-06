// I2C for ATSAM3X8E without interrupts/DMA (blocking!)
// $Id$

#ifndef _DUE_I2C_BLOCKING_H
#define _DUE_I2C_BLOCKING_H

#include <Arduino.h>
#include <stdint.h>

// Timeout (in cyles) for next byte wait
#define TWI_TIMEOUT_COUNTER 16383

// Initialize I2C bus (speed = 100000 or 400000)                                                              
void i2cInit(uint8_t bus, uint32_t speed);

// Read from i2c device (returns number of bytes read)
uint16_t i2cRead(uint8_t bus, uint8_t device, uint8_t address, uint8_t *data, uint8_t count);

// Read from i2c device without internal address (returns number of bytes read)
uint16_t i2cReadNa(uint8_t bus, uint8_t device, uint8_t *data, uint8_t count);

// Write data to i2c device with internal address
uint16_t i2cWrite(uint8_t bus, uint8_t device, uint8_t address, uint8_t *data, uint16_t count);

// Write data to i2c device without internal address
uint16_t i2cWriteNa(uint8_t bus, uint8_t device, uint8_t *data, uint16_t count);

// Write one byte to i2c device with internal address
uint16_t i2cWriteOne(uint8_t bus, uint8_t device, uint8_t address, uint8_t data);

// Write one byte to i2c device without internal address
uint16_t i2cWriteOneNa(uint8_t bus, uint8_t device, uint8_t data);

#endif
