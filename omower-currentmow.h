// Mowing motor current sensor
// $Id$

#ifndef _OMOWER_CURRENTMOW_H
#define _OMOWER_CURRENTMOW_H

#include <omower-root.h>

class currentMow : public currentThing {
public:
  // Maximum current for mowing motor
  float currentMax;

  // Constructor
  currentMow();

private:
  // Previous value for filter
  volatile uint16_t prevValue;

  uint16_t readRawCurrent(numThing n);
};

#endif
