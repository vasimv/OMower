// Lawn sensors support for OMower
// $Id$

#ifndef _LAWN_OMOWER_H
#define _LAWN_OMOWER_H

#include <omower-root.h>
#include <Arduino.h>

class lawn : public thing {
public:
  // Read status of lawn sensor
  boolean readSensor(numThing n);

  // Software initialization stuff
  _status init();

  void poll10();

  // Not really needed stuff
  _locationThings locThings() { return _locationThings::FORW_BACK; }
  numThing numThings() { return 2; }
};
#endif

