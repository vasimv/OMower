// Digital input-output class for OMower
// $Id$

#ifndef _BUTTONLEDS_OMOWER_H
#define _BUTTONLEDS_OMOWER_H

#include <omower-root.h>
#include <Arduino.h>

#define T_BEEPER 0
#define T_BUTTON 1
#define T_BOARDLED 2
#define T_EXTLED1 3
#define T_EXTLED2 4
#define T_EXTLED3 5

// Buttons, LEDs, beeper
// n -> 0 - beeper, 1 - button
class buttonsLeds : public thing {
public:
  // Read status of button/LED
  boolean readSensor(numThing n);

  // Set status of LED/beeper
  void setLED(numThing n, boolean state);

  // Hardware initialization stuff
  _hwstatus begin();

  // Not really needed stuff
  _locationThings locThings() { return _locationThings::SPECIAL; }
  numThing numThings() { return 6; }

  // Force report things to ROS
  void reportToROS();
};
#endif
