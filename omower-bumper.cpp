// Bumper sensors support for OMower                                                                          
// $Id$                     

#include <omower-bumper.h>

#ifdef BUMPERS_INVERT
#define BUMPERS_ACTIVE LOW
#else
#define BUMPERS_ACTIVE HIGH
#endif

// Read bumper status
boolean bumper::readSensor(numThing n) {
  if (bumperCount[n] > 0)
    return true;
  return false;
} // boolean bumper::readSensor(numThing n)

// Software initialization stuff
_status bumper::init() {
#if (BUMPERS_NUM > 0)
  // Set pins mode
  pinMode(PIN_BUMP_LEFTFORW, INPUT_PULLUP);
  pinMode(PIN_BUMP_RIGHTFORW, INPUT_PULLUP);
  pinMode(PIN_BUMP_LEFTFORW, INPUT_PULLUP);
  pinMode(PIN_BUMP_LEFTFORW, INPUT_PULLUP);

  for (int i = 0; i < BUMPERS_NUM; i++) {
    bumperCount[i] = 0;
    bumperDisable[i] = false;
  }

  // Check if bumper sensor is in active status at start (not connected/not working)
  if (digitalRead(PIN_BUMP_LEFTFORW) == BUMPERS_ACTIVE)
    bumperDisable[0] = true;
  if (digitalRead(PIN_BUMP_RIGHTFORW) == BUMPERS_ACTIVE)
    bumperDisable[1] = true;
  if (digitalRead(PIN_BUMP_LEFTBACK) == BUMPERS_ACTIVE)
    bumperDisable[2] = true;
  if (digitalRead(PIN_BUMP_RIGHTBACK) == BUMPERS_ACTIVE)
    bumperDisable[3] = true;
#endif
  return _status::NOERR;
} // _status bumper::init()

// Check bumpers, decrease trigger counters
void bumper::poll10() {
#if (BUMPERS_NUM > 0)
  // Decrease bumper counter every cycle
  for (int i = 0; i < BUMPERS_NUM; i++) {
    if (bumperCount[i] > 0)
      bumperCount[i]--;
  }

  // Check bumpers sensors
  if (!bumperDisable[0] && (digitalRead(PIN_BUMP_LEFTFORW) == BUMPERS_ACTIVE))
    bumperCount[0] = BUMPER_TIMEOUT;
  if (!bumperDisable[1] && (digitalRead(PIN_BUMP_RIGHTFORW) == BUMPERS_ACTIVE))
    bumperCount[1] = BUMPER_TIMEOUT;
  if (!bumperDisable[2] && (digitalRead(PIN_BUMP_LEFTBACK) == BUMPERS_ACTIVE))
    bumperCount[2] = BUMPER_TIMEOUT;
  if (!bumperDisable[3] && (digitalRead(PIN_BUMP_RIGHTBACK) == BUMPERS_ACTIVE))
    bumperCount[3] = BUMPER_TIMEOUT;
  debug(L_DEBUG, (char *) F("bumpers::poll10: %hd %hd %hd %hd\n"),
        bumperCount[0], bumperCount[1], bumperCount[2], bumperCount[3]);
#endif
} // void bumper::poll10()

