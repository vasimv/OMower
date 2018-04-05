// Modbus stuff for OMower
// $Id$

#include <omower-modbus.h>
#include <string.h>

// Calculate CRC for modbus frame                                                                             
uint16_t modbusCrc(uint8_t *buf, int len) {                                                                   
  uint32_t tmp, tmp2;                                                                                         
  uint8_t Flag;                                                                                               
  uint16_t i, j;                                                                                            
                                                                                                              
  tmp = 0xFFFF;                                                                                             
  for (i = 0; i < len; i++) {                                                                               
    tmp = tmp ^ buf[i];                                                                                   
    for (j = 1; j <= 8; j++) {                                                                            
      Flag = tmp & 0x0001;                                                                              
      tmp >>=1;                                                                                         
      if (Flag)                                                                                         
        tmp ^= 0xA001;                                                                                
    }                                                                                                     
  }                                                                                                         
  tmp2 = tmp >> 8;                                                                                          
  tmp = (tmp << 8) | tmp2;                                                                                  
  tmp &= 0xFFFF;                                                                                            
  return (uint16_t) tmp;                                                                                    
} // uint16_t modbusCrc(uint8_t *buf, int len)    

// Retrieve 16-bit value from modbus frame
uint16_t getModbus16(uint8_t *addr) {
  return (uint16_t) addr[1] | ((uint16_t) addr[0] << 8);
} // uint16_t getModbus16(uint8_t *addr)

// Retrieve 32-bit value from modbus frame (two 16bit registers joined)
uint32_t getModbus32(uint8_t *addr) {
  return (uint32_t) addr[3] | ((uint32_t) addr[2] << 8)
          | ((uint32_t) addr[1] << 16) | ((uint32_t) addr[0] << 24);
} // uint32_t getModbus32(uint8_t *addr)

// Put 16-bit value to modbus frame buffer (must increase pointer +2 after)
void putModbus16(uint8_t *buf, uint16_t val) {
  buf[0] = (val >> 8) & 0xff;
  buf[1] = val & 0xff;
} // void putModbus16(uint8_t *buf, uint16_t val)

// Put 32-bit value to modbus frame buffer (must increase pointer +4 after)
void putModbus32(uint8_t *buf, uint32_t val) {
  buf[0] = (val >> 24) & 0xff;
  buf[1] = (val >> 16) & 0xff;
  buf[2] = (val >> 8) & 0xff;
  buf[3] = val & 0xff;
} // void putModbus32(uint8_t *buf, uint32_t val)

