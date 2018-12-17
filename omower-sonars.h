// Sonars sensors for OMower
// $Id$

#ifndef _OMOWER_SONARS_H
#define _OMOWER_SONARS_H

#include <omower-root.h>
#include <Arduino.h>
#include <stdint.h>
#include <omower-defs.h>
#include <omower-ros.h>

class sonars : public thing {
public:
  // Settings variables:
  // Distance for an obstacle stop (must be checked outside)
  uint16_t triggerDist;

  // Software (re)initialization
  _status init();

  // Read current distance, in cms
  // 0 - no value (infinity)
  uint16_t readSonar(numThing n);

  // Number and location constants
  _locationThings locThings();
  numThing numThings();

  // Disable/enable sonars (to save power)
  _status enableThings();
  _status disableThings();

  void poll20();

  // Force report sonars to ROS
  void reportToROS();

private:
  // Current sonar pinged
  uint8_t currentSonar;

  // Calculated distance of all sonars
  uint16_t sonarDist[SONARS_NUM];

  // Filtered distance of all sonars
  uint16_t sonarFilt[SONARS_NUM];

  // Enable sonars pings
  boolean enableSonars;

  // Sonars pairs pins
  const uint8_t sonarTrigs[6] = {PIN_SONAR_TRIG1, PIN_SONAR_TRIG2,
                                 PIN_SONAR_TRIG3, PIN_SONAR_TRIG4,
                                 PIN_SONAR_TRIG5, PIN_SONAR_TRIG6};
  const uint8_t sonarEchos[6] = {PIN_SONAR_ECHO1, PIN_SONAR_ECHO2,
                                 PIN_SONAR_ECHO3, PIN_SONAR_ECHO4,
                                 PIN_SONAR_ECHO5, PIN_SONAR_ECHO6};

  // Send ping for sonar
  void sendPing(numThing n, boolean forward);
  
  // Process data from sonar and put it in sonarDist array
  void processEcho(numThing n);
};

#endif
