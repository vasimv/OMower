// Mowing motor class for OMower
// $Id$

#ifndef _MOW_OMOWER_H
#define _MOW_OMOWER_H

#include <omower-root.h>
#include <omower-currentmow.h>
#include <stdint.h>

class motorMow : public thing {
public:
  // Odometry sensors
//  odometryMow *odometrySens;
  // Current sensors
  currentMow *currentSens;

  // Settings variables:
  // Acceleration (maximum change per 1/10 second), 1..255
  uint8_t accel;
  // Emergency stop acceleration, 1..255
  uint8_t accelStop;
  // Hard maximum PWM for motor controllers, 1..255
  uint8_t maxPWM;
  // Soft maximum (speed) of maxPWM, 1..255
  uint8_t maxSpeed;
  // Modulation (saw pattern) of mowing motor speed
  // 0 - no modulation, 255 - from zero to maximum
  uint8_t modulateSpeed;
  // Modulation period (in milliseconds)
  uint16_t modulatePeriod;
  // Vertical position of mowing motor (0 - most top position), in mm
  uint8_t heightMow;
  // Lowest vertical position of mowing motor, in mm
  uint8_t heightMowMin;

  // Constructor
  motorMow();

  // Hardware initialization
  _hwstatus begin();

  // Reset/initialization of mowing motors drivers and calibrate height
  _status init();

  // Set height position of mower motor (will take few seconds)
  _status setHeight(uint8_t height);

  // Reversing mowing motor(s)
  void reverseMow();

  // Soft error status, requires reinitialization
  _hwstatus softError();

  // Disable motor
  _status disableThings();

  // Enable motor
  _status enableThings();

  void poll10();

  // Force report PWM to ROS
  void reportToROS();

private:
  // Current speed
  uint8_t currentSpeed;
  // Rotate in reverse direction
  boolean reverseDir;

  // error status
  uint8_t errorStatus;

  // Last time when mowing motor was on
  uint32_t lastOn;

  // Modulation variables
  uint8_t cycle;
  uint32_t lastCycle;

  // Set speed of mowing motor
  void setSpeed(uint8_t speed);

  // Move stepper for few steps (with mowing motor off)
  // Returns number of actual steps before fault or end switch
  int stepMove(int steps);
};
#endif
