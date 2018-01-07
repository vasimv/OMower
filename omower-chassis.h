// Chassis data, common hardware initialization
// $Id$

#ifndef _CHASSIS_OMOWER_H
#define _CHASSIS_OMOWER_H

#include <omower-root.h>
#include <Arduino.h>
#include <stdint.h>

class chassis : public thing {
public:
  // chassis dimension, in mm
  uint16_t length;
  uint16_t width;
  uint16_t height;

  // Distance between ground and bottom cover, in mm
  uint16_t clearance;

  // Distance between forward and back axis, in mm
  uint16_t distAxis;

  // Wheels diameter, in mm
  uint16_t diameterWheel;

  // Mowing cut width, in mm
  uint16_t mowSize;

  // Hardware revision number
  uint32_t revisionNum;

  // Pause after power-on for full internal controllers initialization
  // (before starting init())
  uint16_t pauseTime;

  // Must be called before all other begin()'s
  _hwstatus begin();

  // Set hooks functions for poll*()
  void setPollHooks(void (*hookPoll50)(), void (*hookPoll20)(), void (*hookPoll10)());

  // Software (re)initialization
  _status init();
};

extern volatile uint32_t cyclesPollNum;
#endif
