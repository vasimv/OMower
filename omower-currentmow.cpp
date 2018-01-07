// Mowing motor current sensor
// $Id$

#include <omower-currentmow.h>
#include <omower-defs.h>
#include <due-adc-scan.h>
#include <max11617-adc-scan.h>
#include <omower-debug.h>

// Constructor
currentMow::currentMow() {
  #ifdef MOW_CURRENT_INT
  kCurrent = K_ADC_CURRENT_ACS725_30;
  zeroOffset = 2048;
  #endif
  #ifdef MOW_CURRENT_EXT1
  kCurrent= K_ADC_CURRENT_MOTORS
  zeroOffset = 0;
  #endif
  prevValue = zeroOffset;
} // currentMotors::currentMotors()

// Returns raw current values
uint16_t currentMow::readRawCurrent(numThing n) {
  uint16_t value;

  #ifdef MOW_CURRENT_INT
  value = adcArr[MOW_CURRENT_INT].lastRead;
  #endif

  #ifdef MOW_CURRENT_EXT1
  value = adc11617Arr[MOW_CURRENT_EXT1];
  #ifdef MOW_CURRENT_EXT2
  value += adc11617Arr[MOW_CURRENT_EXT2];
  #endif
  #endif

  // Do filtering (0.2 koefficient)
  value = (value + 4 * prevValue) / 5;
  prevValue = value;
  return value;
} // uint16_t currentMow::readRawCurrent(numThing n)
