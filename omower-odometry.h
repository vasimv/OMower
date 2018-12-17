// Wheel motors odometry sensors class for OMower
// $Id$

#ifndef _ODOMETRY_OMOWER_H
#define _ODOMETRY_OMOWER_H

#include "omower-root.h"
#include "omower-defs.h"

// Wheel motors odometry sensors
class odometryMotors : public navThing {
public:
  // Settings variables:
  // Pulses by full turn
  uint16_t ticksRev;
  // Pulses by cm
  float ticksCm;

  // Hardware init (setting interrupts)
  _hwstatus begin();

  // Resetting odometry variables (can be re-called for resetting)
  _status init();

  // Reads pulses counter (negative - reverse rotation)
  long readTicks(numThing n);

  // Resets ticks counters
  _status resetTicks(numThing n);

  // Sets "course", in pulses (offset of current value)
  void setCourse(long leftTicks, long rightTicks);

  // Error (for motor drivers), -PI .. PI
  float readCourseError();

  // Ticks per minute (about, calculated by two times per second)
  int32_t readTPM(numThing n);

  // Revolutions per minute (about, calculated by two times per second)
  int16_t readRPM(numThing n);

  void poll10();

  // Force report odometers readings to ROS
  void reportToROS();

private:
  long prevTicks[_NUM_ODOMETERS_WHEELS];
  uint8_t pollsNum;
  long targTicks[2];
  int32_t TPM[_NUM_ODOMETERS_WHEELS];
};

#endif
