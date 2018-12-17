// Motors (wheel) current sensors
// $Id$

#include <omower-currentmotors.h>
#include <max11617-adc-scan.h>
#include <omower-defs.h>
#include <omower-ros.h>

// Constructor
currentMotors::currentMotors() {
  kCurrent = K_ADC_CURRENT_MOTORS;
  zeroOffset = 0;
  for (int i = 0; i < numThings(); i++)
    prevRaw[i] = 0;
} // currentMotors::currentMotors()

// Returns number of sensors
numThing currentMotors::numThings() {
#ifdef MOT_QUAD_WHEELS
  return 4;
#endif
  return 2;
} // numThing currentMotors::numThings()

// Returns raw current values
uint16_t currentMotors::readRawCurrent(numThing n) {
  uint16_t rawCurrent;

  switch (n) {
    case 0:
      rawCurrent = adc11617Arr[CH_ADC_LEFTFORW];
      break;
    case 1:
      rawCurrent = adc11617Arr[CH_ADC_RIGHTFORW];
      break;
    case 2:
      rawCurrent = adc11617Arr[CH_ADC_LEFTBACK];
      break;
    case 3:
      rawCurrent = adc11617Arr[CH_ADC_RIGHTBACK];
      break;
    default:
      rawCurrent = zeroOffset;
      break;
  }

  // Additional smoothing
  rawCurrent = (rawCurrent + 8 * prevRaw[n]) / 9;
  prevRaw[n] = rawCurrent;
  return rawCurrent;
} // uint16_t currentMotors::readRawCurrent(numThing n)

#ifdef USE_ROS
float rosMotCurrents[4] = {0, 0, 0, 0};
#endif

// Force report to ROS
void currentMotors::reportToROS() {
#ifdef USE_ROS
  for (numThing i = 0; i < numThings(); i++)
    rosMotCurrents[i] = readCurrent(i);
#ifdef MOT_QUAD_WHEELS
  oROS.reportToROS(reportSensor::MOTORCURRENT, (uint8_t *) rosMotCurrents, 4);
#else
  oROS.reportToROS(reportSensor::MOTORCURRENT, (uint8_t *) rosMotCurrents, 2);
#endif
#endif
} // void currentMotors::reportToROS()
