// Drop sensors support for OMower
// $Id$

#ifndef _DROP_OMOWER_H
#define _DROP_OMOWER_H

#include <omower-root.h>
#include <Arduino.h>

class drop : public thing {
public:
  // Read status of drop sensor
  uint16_t readSensor(numThing n);

  // Software initialization stuff
  _status init();

  void poll10();

  // Not really needed stuff
  _locationThings locThings() { return _locationThings::FORW_BACK; }
  numThing numThings() { return 2; }
};
#endif

