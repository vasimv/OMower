// Wheel motors class for test car chassis
// $Id$

#ifndef _MOTORS_OMOWER_H
#define _MOTORS_OMOWER_H

#include "omower-root.h"
#include "omower-odometry.h"
#include "omower-currentmotors.h"

// Zero PWM for at least one second (robot is not moving)
extern boolean _notMoving;

class motors : public thing {
public:
  // Encoders sensors
  odometryMotors *odometrySens;

  // Motor current sensors (in case of overcurrent - emergency stop,
  // status through softError(), requires re-initialization)
  currentMotors *currentSens;

  // Settings variables:
  // acceleration per 1/10 second, 1..255
  uint8_t accel;
  // Emergency stop acceleration, 1..255
  uint8_t accelStop;
  // Hard maximum PWM for motor controllers, 1..255
  uint8_t maxPWM;
  // Minimum PWM for brushed motor start 1..255
  uint8_t minPWM;
  // Soft maximum (speed) of maxPWM, 1..255
  uint8_t maxSpeed;
  // Reverse all motors direction
  boolean reverseAll;

  // PID values for movement
  float pidMoveP;
  float pidMoveI;
  float pidMoveD;

  // PID values for roll
  float pidRollP;
  float pidRollI;
  float pidRollD;

  // Constructor
  motors();

  // Hardware initialization (called at setup)
  _hwstatus begin();

  // Resets/initializes motors driver and sensors
  _status init();

  // Enable/disable for all devices of same class (sleep mode if supported)
  _status enableThings();
  _status disableThings();

  // Emergency stop (zero'ing speed with accelStop acceleration)
  void emergStop();

  // Turn on a place, with timeout (milliseconds)
  void roll(_dir dirRoll, unsigned long ms);

  // Movement, with timeout (milliseconds)
  void move(_dir dirMove, unsigned long ms);

  // Starts turn on a place by a sensor reading (compass, perimeter, GPS, etc)
  // ms - timeout in milliseconds.
  // Sensor must return error of needed course by readCourseError()
  void rollCourse(navThing *sensor, unsigned long ms);

  // Starts movement by a sensor reading
  // ms - timeout in milliseconds.
  // Sensor must return error of needed course by readCourseError()
  void moveCourse(navThing *sensor, unsigned long ms);

  // Soft (recoverable) error status (overcurrent, odometry, etc),
  // requires re-initialization
  _hwstatus softError();

  // Current PWM value on motor(s)
  int16_t curPWM(numThing n);

  void poll10();

private:
  // Current PWM values (negative values means reverse)
  int16_t leftPWM = 0;
  int16_t rightPWM = 0;

  uint8_t errorStatus;

  // When we should end movement
  unsigned long endTime = 0;

  // When we've moved last time
  unsigned long lastMoveTime;

  // Pointer to navigation things (NULL if not used)
  navThing *courseThing = NULL;

  // Speed set for non-navigation mode
  int16_t leftSpeed, rightSpeed;

  // Current status of movement
  enum class _moveStatus : int8_t { EMERG_STOP = -1, STOP = 0, MOVE = 1, ROLL = 2, MOVE_NAV = 3, ROLL_NAV = 4};

  // Current status, after finishing (by timeout) - set to "STOP"
  _moveStatus curStatus = _moveStatus::STOP;

  // Set PWM levels on motor driver
  void setPWM(int16_t left, int16_t right);

  // Update PWM with speeds (acceleration in effect)
  void updateSpeed(int16_t left, int16_t right, uint8_t accel);

  // Calculate PWM change based on maximum acceleration
  int16_t getPWMAccel(int16_t needed, int16_t current, uint8_t accelCur);

  // PID variables
  float sumErr, lastErr;

  // PID controller reset
  void pidReset();

  // PID controller output (-1.0 ... 1.0)
  float pidCalc(float error, float P, float I, float D);

  // Check for faults (overcurrent, driver fault)
  uint8_t checkFault();
};
#endif
