// Bumper sensors support for OMower                                                                          
// $Id$                     

#include <omower-bumper.h>
#include <omower-ros.h>

#ifdef BUMPERS_INVERT
#define BUMPERS_ACTIVE LOW
#else
#define BUMPERS_ACTIVE HIGH
#endif

// Read bumper status
boolean bumper::readSensor(numThing n) {
#if (BUMPERS_NUM > 0)
  if (bumperCount[n] > 0)
    return true;
#endif
  return false;
} // boolean bumper::readSensor(numThing n)

// Software initialization stuff
_status bumper::init() {
  debug(L_INFO, (char *) F("bumper::init\n"));
#if (BUMPERS_NUM > 0)
  // Set pins mode
  pinMode(PIN_BUMP_LEFTFORW, INPUT_PULLUP);
  pinMode(PIN_BUMP_RIGHTFORW, INPUT_PULLUP);
  pinMode(PIN_BUMP_LEFTBACK, INPUT_PULLUP);
  pinMode(PIN_BUMP_RIGHTBACK, INPUT_PULLUP);

  for (int i = 0; i < BUMPERS_NUM; i++) {
    bumperCount[i] = 0;
    bumperDisable[i] = false;
  }
  delay(1);

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
  reportToROS();
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

// Force report to ROS
void bumper::reportToROS() {
#if defined(USE_ROS) && (BUMPERS_NUM > 0)
  oROS.reportToROS(reportSensor::BUMPER, &bumperCount[0], BUMPERS_NUM);
#endif
} // void bumper::reportToROS()
