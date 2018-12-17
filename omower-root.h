// OMower basic classes
// $Id$

#ifndef _OMOWER_ROOT_H
#define _OMOWER_ROOT_H

#include <Arduino.h>
#include <stddef.h>
#include <omower-enums.h>

// Problems with Os optimization!
#pragma GCC optimize ("O3")

// Base class for all devices
class thing {
public:
  // Hardware initialization (once)
  virtual _hwstatus begin() { return _hwstatus::ONLINE; }

  // (re)Initializing hardware of all devices of same class
  virtual _status init() { return _status::NOERR;}

  // Enable/disable for all devices of same class (sleep mode if supported)
  virtual _status enableThings() { return _status::NOERR; }
  virtual _status disableThings() { return _status::NOERR; }

  // Returns number of devices of the class
  inline virtual numThing numThings() { return 1; }

  // Returns placement of devices
  inline virtual _locationThings locThings() { return _locationThings::JUSTONE; }

  // Checks status of device, if thingNum = -1 - status of all devices
  // (their bus/master controller)
  virtual _hwstatus checkThing(numThing n) { return _hwstatus::ONLINE; }

  virtual _hwstatus softError() { return checkThing(-1); }

  // Returns error counts number for the device (-1 - sum for all of them)
  virtual int errorThing(numThing n) { return 0; }

  // Increments errors counter for the device (-1 - for all of them)
  virtual void addError(numThing n) {}

  // Resets errors counter
  virtual void resetError(numThing n) {}

  // Force to report sensor 
  virtual void reportToROS() { return; }

  // This method must be called 10 times per second
   inline virtual void poll10() { return; }

  // This method must be called 20 times per second
  inline virtual void poll20() { return; }

  // This method must be called 50 times per second
  inline virtual void poll50() { return; }
};

// Subclass for devices with navigation (like GPS or IMU)
class navThing : public thing {
public:
  // Returns error for PID-controller, -PI..PI range
  // (if > 999 - temporary stop, if < -999 - emergency stop)
  virtual float readCourseError() { return -999; }

  // Returs true if reached the destination
  virtual boolean reachedDest() { return destReached; }

  // Constructor
  navThing() {};
protected:
  boolean destReached;
};

// Sub-class for current sensors
class currentThing : public thing {
public:
  // Settings variables:
  // ADC->Current koefficient
  float kCurrent;
  // Offset for zero current
  uint16_t zeroOffset;

  virtual _status init();

  // Current reading, -1 - sum for all things of same class
  virtual float readCurrent(numThing n);

  // Calibrate ACS725 sensor (when there is sure zero current)
  virtual void calibCurrent(numThing n);
protected:

  // Read raw current value
  virtual uint16_t readRawCurrent(numThing n);
};

#endif
