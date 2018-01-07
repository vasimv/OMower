// NV-memory (eeprom or F-RAM) class for OMower
// $Id$

#ifndef OMOWER_NVMEM_H
#define OMOWER_NVMEM_H

#include <omower-root.h>
#include <omower-defs.h>
#include <stdint.h>
#include <Arduino.h>

// NVMEM  object (note, first bytes will be used by the OMower objects for calibration values, etc)
class nvmem : public thing {
public:
  // Current pointer in NVMEM
  // Gets incremented after read/write
  // High-level software must read and save this value after full hardware initialization to
  // set it later for saving its settings and stuff
  uint32_t curAddr;

  // Software revision number (other revision numbers will be ignored during search)
  uint32_t revision;

  // We do have valid stuff in NVMEM (otherwise the software must set default values)
  boolean haveValid;

  // Read data from NVMEM
  void readMem(uint8_t &data);
  void readMem(int8_t &data);
  void readMem(uint16_t &data);
  void readMem(int16_t &data);
  void readMem(uint32_t &data);
  void readMem(int32_t &data);
  void readMem(float &data);
  void readMem(boolean &data);

  void writeMem(uint8_t data);
  void writeMem(int8_t data);
  void writeMem(uint16_t data);
  void writeMem(int16_t data);
  void writeMem(uint32_t data);
  void writeMem(int32_t data);
  void writeMem(float data);
  void writeMem(boolean data);

  // Commit write to the flash memory
  void commit();

  // Hardware initialization (must be called before any use of the NVMEM)
  _hwstatus begin();
private:
  // RAM buffer for flash memory
  uint8_t buffer[NVMEM_SIZE];

  // current version ID of the last flash record (will be increased with every commit)
  uint32_t curVersion;

  // Internal read function
  void readMemByte(uint8_t *data);
};

#endif
