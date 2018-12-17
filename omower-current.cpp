// Current sensor class for OMower
// $Id$

#include <omower-root.h>
#include <Arduino.h>
#include <due-adc-scan.h>
#include <omower-debug.h>

// stub
_status currentThing::init() {
  return _status::NOERR;
} // _status currentThing::init()
                                                                                                              
// stub
uint16_t currentThing::readRawCurrent(numThing n) {
  return 0;
} // uint16_t currentThing::readRawCurrent(numThing n)

// Current reading, -1 - sum for all things of same class                                                   
float currentThing::readCurrent(numThing n) {
  int32_t sum;
  numThing i;

  if (n != -1) {
    return kCurrent * ((int16_t) readRawCurrent(n) - (int16_t) zeroOffset);
  }

  sum = 0;
  for (i = 0; i < numThings(); i++)
    sum += ((int16_t) readRawCurrent(i) - (int16_t) zeroOffset);
  return kCurrent * sum;
} // float currentThing::readCurrent(numThing n)

// Calibrate ACS725 sensor (when there is sure zero current)                                                
void currentThing::calibCurrent(numThing n) {
  zeroOffset = readRawCurrent(n);
} // void currentThing::calibCurrent(numThing n)
