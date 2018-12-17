// Motors (wheel) current sensors
// $Id$

#ifndef _OMOWER_CURRENTMOTORS_H
#define _OMOWER_CURRENTMOTORS_H

#include <omower-root.h>

class currentMotors : public currentThing {
public:
  // Maximum current (must be checked in top-level class) per motor
  float currentMax;

  // Constructor
  currentMotors();

  // Number of current sensors
  numThing numThings();

  // Force report to ROS
  void reportToROS();
private:
#ifdef MOT_QUAD_WHEELS
  uint16_t prevRaw[4];
#else
  uint16_t prevRaw[2];
#endif
  uint16_t readRawCurrent(numThing n);
};

#endif
