// NV memory class for OMower (using internal flash storage)
// $Id$

#include <omower-nvmem.h>
#include <string.h>

#ifdef NVMEM_INTERNAL_FLASH
// Atmel flash stuff (uses at least one full page size of RAM)
#include <flash_efc.h>
#include <efc.h>

#define NVMEM_START ((uint8_t *) IFLASH1_ADDR + IFLASH1_SIZE - NVMEM_SIZE * NVMEM_BLOCKS)
#define NVMEM_END ((uint8_t *) IFLASH1_ADDR + IFLASH1_SIZE - 1)
#endif

// Internal read function
void nvmem::readMemByte(uint8_t *data) {
  if (curAddr >= NVMEM_SIZE)
    return;
  *data = buffer[curAddr];
  curAddr++;
} // void nvmem::readMemByte(uint8_t *data)

// Read unsigned byte (with address check)
void nvmem::readMem(uint8_t &data) {
  readMemByte(&data);
} // void nvmem::readMem(uint8_t &data)

// Read boolean
void nvmem::readMem(boolean &data) {
  uint8_t byte;

  readMemByte(&byte);
  data = *((boolean *) &byte);
} // void nvmem::readMem(int8_t &data)

// Read signed byte
void nvmem::readMem(int8_t &data) {
  uint8_t byte;

  readMemByte(&byte);
  data = *((int8_t *) &byte);
} // void nvmem::readMem(int8_t &data)

// Read unsigned two bytes word
void nvmem::readMem(uint16_t &data) {
  uint16_t word;

  readMemByte((uint8_t *) &word);
  readMemByte(((uint8_t *) &word) + 1);
  data = word;
} // void nvmem::readMem(uint16_t &data)

// Read signed two bytes word
void nvmem::readMem(int16_t &data) {
  int16_t word;

  readMemByte((uint8_t *) &word);
  readMemByte(((uint8_t *) &word) + 1);
  data = word;
} // void nvmem::readMem(int16_t &data)

// Read unsigned four bytes word
void nvmem::readMem(uint32_t &data) {
  uint32_t word;

  readMemByte((uint8_t *) &word);
  readMemByte(((uint8_t *) &word) + 1);
  readMemByte(((uint8_t *) &word) + 2);
  readMemByte(((uint8_t *) &word) + 3);
  data = word;
} // void nvmem::readMem(uint32_t &data)

// Read signed four bytes word
void nvmem::readMem(int32_t &data) {
  int32_t word;

  readMemByte((uint8_t *) &word);
  readMemByte(((uint8_t *) &word) + 1);
  readMemByte(((uint8_t *) &word) + 2);
  readMemByte(((uint8_t *) &word) + 3);
  data = word;
} // void nvmem::readMem(int32_t &data)

// Read float
void nvmem::readMem(float &data) {
  float word;

  readMemByte((uint8_t *) &word);
  readMemByte(((uint8_t *) &word) + 1);
  readMemByte(((uint8_t *) &word) + 2);
  readMemByte(((uint8_t *) &word) + 3);
  data = word;
} // void nvmem::readMem(int32_t &data)

// Write unsigned byte to the NVMEM
void nvmem::writeMem(uint8_t data) {
  if (curAddr >= NVMEM_SIZE)
    return;
  buffer[curAddr] = data;
  curAddr++;
} // void nvmem::writeMem(uint8_t data)

// Write signed byte to the NVMEM
void nvmem::writeMem(int8_t data) {
  writeMem(*((uint8_t *) &data));
} // void nvmem::writeMem(int8_t data)

// Write boolean the NVMEM
void nvmem::writeMem(boolean data) {
  writeMem(*((uint8_t *) &data));
} // void nvmem::writeMem(boolean data)

// Write unsigned two bytes word to NVMEM
void nvmem::writeMem(uint16_t data) {
  writeMem(*((uint8_t *) &data));
  writeMem(*(((uint8_t *) &data) + 1));
} // void nvmem::writeMem(uint16_t data)

// Write signed two bytes word to NVMEM
void nvmem::writeMem(int16_t data) {
  writeMem(*((uint8_t *) &data));
  writeMem(*(((uint8_t *) &data) + 1));
} // void nvmem::writeMem(int16_t data)

// Write unsigned four bytes word to NVMEM
void nvmem::writeMem(uint32_t data) {
  writeMem(*((uint8_t *) &data));
  writeMem(*(((uint8_t *) &data) + 1));
  writeMem(*(((uint8_t *) &data) + 2));
  writeMem(*(((uint8_t *) &data) + 3));
} // void nvmem::writeMem(uint32_t data)

// Write signed four bytes word to NVMEM
void nvmem::writeMem(int32_t data) {
  writeMem(*((uint8_t *) &data));
  writeMem(*(((uint8_t *) &data) + 1));
  writeMem(*(((uint8_t *) &data) + 2));
  writeMem(*(((uint8_t *) &data) + 3));
} // void nvmem::writeMem(int32_t data)

// Write float to NVMEM
void nvmem::writeMem(float data) {
  writeMem(*((uint8_t *) &data));
  writeMem(*(((uint8_t *) &data) + 1));
  writeMem(*(((uint8_t *) &data) + 2));
  writeMem(*(((uint8_t *) &data) + 3));
} // void nvmem::writeMem(float data)

// Hardware initialization (initialize buffer, check NVMEM storage for valid blocks, find last one)
_hwstatus nvmem::begin() {
  uint32_t maxVersion = 0;
  uint8_t *lastBlock;

  curAddr = 0;
#ifdef NVMEM_INTERNAL_FLASH
  flash_init(FLASH_ACCESS_MODE_128, 6);

  // Find valid block with highest version number
  for (uint8_t *p = NVMEM_START; p < NVMEM_END; p += NVMEM_SIZE) {
    if ((*p == 0xAA) && (*(p + 1) == 0x55)) {
      if ((*((uint32_t *) (p + 2)) > maxVersion) && (*((uint32_t *) (p + 6)) == revision)) {
        lastBlock = p;
        maxVersion = *((uint32_t *) (p + 2));
      }
    }
  }
  curVersion = maxVersion;
  if (maxVersion > 0) {
    debug(L_NOTICE, (char *) F("nvmem::begin: found latest block %lx (%lu)\n"), lastBlock, maxVersion);
    memcpy(buffer, lastBlock, sizeof(buffer));
    haveValid = true;
  } else {
    memset(buffer, 0xFF, sizeof(buffer));
    haveValid = false;
    buffer[0] = 0xAA;
    buffer[1] = 0x55;
  }

  // Skip first bytes (signature, version and revision number)
  curAddr += 10;
#endif
  return _hwstatus::ONLINE;
} // _hwstatus nvmem::begin()

// Write modified values into flash
void nvmem::commit() {
  uint8_t *lastBlock;
  uint8_t *freeBlock;
  uint32_t minVersion;
  boolean eraseNeed;

#ifdef NVMEM_INTERNAL_FLASH
  // First find current block and check if we are really needed to write the flash
  if (curVersion > 0) {
    lastBlock = NULL;
    for (uint8_t *p = NVMEM_START; p < NVMEM_END; p += NVMEM_SIZE) {
      if ((*p == 0xAA) && (*(p + 1) == 0x55)) {
        if ((*((uint32_t *) (p + 2)) == curVersion) && (*((uint32_t *) (p + 6)) == revision))
          lastBlock = p;
      }
    }
    // We've found current block, compare it, do nothing if same
    if (lastBlock && (memcmp(lastBlock, buffer, sizeof(buffer)) == 0)) {
      debug(L_NOTICE, (char *) F("nvmem::commit: same data at commit, ignoring\n"));
      return;
    }
  }

  // Find first free block or block with smallest version
  minVersion = 0xffffffff;
  lastBlock = NVMEM_START;
  freeBlock = NULL;
  for (uint8_t *p = NVMEM_START; p < NVMEM_END; p += NVMEM_SIZE) {
    if ((*p == 0xAA) && (*(p + 1) == 0x55)) {
      if (*((uint32_t *) (p + 2)) < minVersion) {
        lastBlock = p;
        minVersion = *((uint32_t *) (p + 2));
      }
    } else {
      // Check if all bytes are 0xFF (clean FLASH)
      if ((*p == 0xFF) && (memcmp(p, p + 1, NVMEM_SIZE - 1) == 0))
        freeBlock = p;
    }
  }
  debug(L_DEBUG, (char *) F("nvmem::commit: lastBlock %lx freeBlock %lx minVersion %lu\n"),
        lastBlock, freeBlock, minVersion);
  
  // Write to free block if available or erase lastBlock page and write to it
  if (freeBlock == NULL) {
    eraseNeed = true;
    freeBlock = lastBlock;
  } else
    eraseNeed = false;

  // Increase version, so we won't read old stuff anymore
  curVersion++;
  *((uint32_t *) (buffer + 2)) = curVersion;

  debug(L_NOTICE, (char *) F("nvmem::commit: writing version %lu to memory at %lx\n"),
        curVersion, freeBlock);
  // Atmel's flash write functions
  flash_unlock((uint32_t) freeBlock, (uint32_t) freeBlock + NVMEM_SIZE - 1, 0, 0);
  flash_write((uint32_t) freeBlock, buffer, sizeof(buffer), eraseNeed ? 1: 0);
  flash_lock((uint32_t) freeBlock, (uint32_t) freeBlock + NVMEM_SIZE - 1, 0, 0);
#endif
} // void nvmem::commit()
