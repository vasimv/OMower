// Bumper sensors support for OMower
// $Id$

#ifndef _BUMPER_OMOWER_H
#define _BUMPER_OMOWER_H

#include <omower-root.h>
#include <omower-defs.h>
#include <Arduino.h>

// How long (in 1/10 of second) keep bumper's signal active
#define BUMPER_TIMEOUT 5

class bumper : public thing {
public:
  // Read status of bumper sensor
  boolean readSensor(numThing n);

  // Software initialization stuff
  _status init();

  void poll10();

  // Not really needed stuff
  _locationThings locThings() { return _locationThings::LEFTFORW_RIGHTFORW_LEFTBACK_RIGHTBACK; }
  numThing numThings() { return BUMPERS_NUM; }

private:
#if (BUMPERS_NUM > 0)
  // Bumper counters
  uint8_t bumperCount[BUMPERS_NUM];

  // Enable/disable bumper sensor (checked at init)
  boolean bumperDisable[BUMPERS_NUM];
#endif
};
#endif

