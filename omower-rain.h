// Rain sensor support for OMower
// $Id$

#ifndef _RAIN_OMOWER_H
#define _RAIN_OMOWER_H

#include <omower-root.h>
#include <Arduino.h>

class rain : public thing {
public:
  // Read status of lawn sensor
  boolean readSensor(numThing n);

  // Software initialization stuff
  _status init();

  void poll10();
};
#endif

