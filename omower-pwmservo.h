// PWM/Servo output support for OMower
// $Id$

#ifndef _PWMSERVO_OMOWER_H
#define _PWMSERVO_OMOWER_H

#include <omower-root.h>
#include <Arduino.h>
#include <stdint.h>

// Don't use these PWM outputs if you need following TC counters:
// PWM-F uses Timer0
// PWM-G uses Timer6
// PWM-H uses Timer8

// Note:
// PWM-A, PWM-B are used by motors class
// PWM-C, PWM-D also are used by motors class when 4 wheel motors
// PWM-E is used by mowing motor class when external brushless motor ESC connected
// PWM-FAN, PWM-BOOST, PWM-CHARGE are used by power class

// Numbers constants (numThing)
#define PWM_A 0
#define PWM_B 1
#define PWM_C 2
#define PWM_D 3
#define PWM_E 4
#define PWM_F 5
#define PWM_G 6
#define PWM_H 7
#define PWM_FAN 8
#define PWM_BOOST 9
#define PWM_CC 10

class pwmServo : public thing {
public:
  // Start/set RC servo with duty 0..999 (1..2 ms) with refresh rate (hertz)
  void servoStart(numThing n, uint16_t duty, uint16_t refresh = 100);

  // Set duty cycle 0..999 (1..2 ms)
  void servoSet(numThing n, uint16_t duty);

  // Start/set PWM output with duty/period in hundreths of microsecond
  // (actual resolution is a bit lower as CPU frequency is 84 MHz only)
  void pwmStart(numThing n, uint32_t period, uint32_t duty);

  // Set PWM output duty in microseconds
  void pwmSet(numThing n, uint32_t duty);

  // Constants
  numThing numThings();
  _locationThings locThings();

private:
  // Convert hundreths of microsecond to 1/2 CPU freq cycles (for TC*)
  inline uint32_t husToCycles(uint32_t hus);
};
#endif
