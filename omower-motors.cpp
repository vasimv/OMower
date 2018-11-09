// OMower motors class
// $Id$

// Problems with Os optimization!
#pragma GCC optimize ("O2")

#include <omower-defs.h>
#include <Arduino.h>
#include "omower-constants.h"
#include "omower-debug.h"
#include "omower-motors.h"
#include <pwm_lib.h>

using namespace arduino_due::pwm_lib;

boolean _notMoving;

#ifndef MOT_DRIVER_DRV8825
pwm<pwm_pin::CH_MOT_LEFTFORW_PWM> pwm_leftforw;
pwm<pwm_pin::CH_MOT_RIGHTFORW_PWM> pwm_rightforw;

#ifdef MOT_QUAD_WHEELS
pwm<pwm_pin::CH_MOT_LEFTBACK_PWM> pwm_leftback;
pwm<pwm_pin::CH_MOT_RIGHTBACK_PWM> pwm_rightback;
#endif
#else
#include "due-adc-scan.h"

volatile uint32_t _leftStepsDivider = 0;
volatile uint32_t _rightStepsDivider = 0;
volatile boolean _lowerLeftStep = false;
volatile boolean _lowerRightStep = false;
volatile int32_t _leftStepsCount;
volatile int32_t _rightStepsCount;
volatile uint8_t _motorsWakeUp = 0;

// Calculate divider ratio for stepping
#define DRV8825_DIV(x) ((x == 0) ? 0 : (((uint32_t) (_ADC_SAMPLE_RATE / 600) * 255L) / (uint32_t) abs(x)))
#endif

// Constructor
motors::motors() {
  currentSens = NULL;
} // motors::motors()

// Called 10 times per second
void motors::poll10() {
  float courseError;
  float pidOut;
  uint8_t speedCorr;

  debug(L_DEBUG, "motors::poll10: curStatus = %d, endtime= %lu, leftPWM = %d, rightPWM = %d\n",
       curStatus, endTime, leftPWM, rightPWM);

  // Check if there is a fault
  if (checkFault()) {
    disableThings();
    setPWM(0, 0);
    return;
  }

  // Check if we've reached timeout
  if (endTime > 0) {
    if (endTime < millis()) {
      if (curStatus != _moveStatus::EMERG_STOP)
        curStatus = _moveStatus::STOP;
      endTime = 0;
    }
  }

  // Calculate PWMs
  switch (curStatus) {
    // We're in emergency stop mode
    case _moveStatus::EMERG_STOP:
      if ((leftPWM == 0) && (rightPWM == 0)) {
        debug(L_INFO, "motors::poll10: emergency stop complete\n");
        curStatus = _moveStatus::STOP;
        break;
      }
      updateSpeed(0, 0, accelStop);
      break;

    // We're in stop mode
    case _moveStatus::STOP:
      // We've stopped already, do nothing
      if ((leftPWM == 0) && (rightPWM == 0)) {
        debug(L_DEBUG, "motors::poll10: stop complete\n");
        break;
      }
      updateSpeed(0, 0, accel);
      break;

    // We're moving or rolling with constant speed
    case _moveStatus::MOVE:
    case _moveStatus::ROLL:
      updateSpeed(leftSpeed, rightSpeed, accel);
      break;

    // We're moving or rolling with navigation
    case _moveStatus::MOVE_NAV:
    case _moveStatus::ROLL_NAV:
      // Safeguard check
      if (!courseThing) {
        curStatus = _moveStatus::STOP;
        break;
      }
      // Get course error for PID controller
      courseError = courseThing->readCourseError();

      // Check if courseThing called for stop
      if (courseError < -999.0) {
        debug(L_INFO, "motors::poll10: courseThing called for emergency stop\n");
        curStatus = _moveStatus::EMERG_STOP;
        poll10();
        break;
      }
      if (courseError > 999.0) {
        if ((leftPWM != 0) || (rightPWM != 0))
          debug(L_INFO, (char *) F("motors::poll10: courseThing called for temporary stop\n"));
        leftSpeed = 0;
        rightSpeed = 0;
      } else {
        // Calculate speed
        if (curStatus == _moveStatus::ROLL_NAV) {
          // Turning at one place
          pidOut = pidCalc(courseError, pidRollP, pidRollI, pidRollD);
          leftSpeed = pidOut * maxSpeed;
          rightSpeed = -pidOut * maxSpeed;
        } else {
          // Moving
          pidOut = pidCalc(courseError, pidMoveP, pidMoveI, pidMoveD);
          // Calculate speed correction (up to 1.5 times at 15 degrees)
          if (abs(courseError) > 0.2618f)
            speedCorr = maxSpeed / 1.3f;
          else {
            if (abs(courseError) < 0.017f)
              speedCorr = maxSpeed;
            else
              speedCorr = maxSpeed / (1 + abs(courseError) / (3.0f * 0.2618f));
          }
          leftSpeed = speedCorr;
          rightSpeed = speedCorr;
          if (pidOut > 0)
            rightSpeed = speedCorr - (int16_t) (pidOut * (float) speedCorr);
          else
            leftSpeed = speedCorr + (int16_t) (pidOut * (float) speedCorr);
        }
        debug(L_DEBUG, (char *) F("motors::poll10: courseError %g, pidOut %g, leftSpeed %d, rightSpeed %d\n"),
              courseError, pidOut, leftSpeed, rightSpeed);
      	// Do not allow to stop one of side wheels while other is rotating
      	if ((leftSpeed == 0) && (rightSpeed !=0))
          leftSpeed = -rightSpeed * 255 / 160;
        if ((rightSpeed == 0) && (leftSpeed != 0))
          rightSpeed = -leftSpeed * 255 / 160;
      }

      // Range check
      if (leftSpeed > maxSpeed)
        leftSpeed = maxSpeed;
      if (rightSpeed > maxSpeed)
        rightSpeed = maxSpeed;
      if (leftSpeed < -maxSpeed)
        leftSpeed = -maxSpeed;
      if (rightSpeed < -maxSpeed)
        rightSpeed = -maxSpeed;

      updateSpeed(leftSpeed, rightSpeed, accel);
      break;

    // Shouldn't happen
    default:
      curStatus = _moveStatus::STOP;
      break;
  }

  // Set _notMoving flag for external stuff
  if ((leftPWM == 0) && (rightPWM == 0)) {
    if ((millis() - lastMoveTime) > 1000) {
      _notMoving = true;
      #ifdef MOT_DRIVER_DRV8825
      disableThings();
      #endif
    }
  } else {
    _notMoving = false;
    lastMoveTime = millis();
  }
} // void motors::poll10()

// Calculate updated PWM with acceleration
int16_t motors::getPWMAccel(int16_t needed, int16_t current, uint8_t accelCur) {
  int16_t res;

  // Calculate PWM change
  if (needed > current) {
    res = current + (int16_t) accelCur;
    if (res > needed)
      res = needed;
  } else {
    res = current - (int16_t) accelCur;
    if (res < needed) 
      res = needed;
  }
  return res;
} // int16_t motors::getPWMAccel(int16_t needed, int16_t current, uint8_t accelCur)

// Update PWM of motors with speed->PWM conversion
void motors::updateSpeed(int16_t left, int16_t right, uint8_t accelCur) {
  int16_t leftSet, rightSet;

  // Calculate PWM values based on speed, maxSpeed and maxPWM
  leftSet = (int32_t) left * (int32_t) maxSpeed * (int32_t) maxPWM / 65025;
  rightSet = (int32_t) right * (int32_t) maxSpeed * (int32_t) maxPWM / 65025;

  // Set to minimum PWM if too low
  if ((leftSet != 0) && (abs(leftSet) < minPWM)) {
    if (leftSet > 0)
      leftSet = minPWM;
    else
      leftSet = -minPWM;
  }
  if ((rightSet != 0) && (abs(rightSet) < minPWM)) {
    if (rightSet > 0)
      rightSet = minPWM;
    else
      rightSet = -minPWM;
  }

  setPWM(getPWMAccel(leftSet, leftPWM, accelCur), getPWMAccel(rightSet, rightPWM, accelCur));
} // void motors::updateSpeed(int16_t left, int16_t right, uint8_t accel)

// PID controller output calculation -1..1
float motors::pidCalc(float error, float P, float I, float D) {
  float dErr;
  float res;
  float iErr;

  // Normalize input value to -1.0 .. 1.0 range
  dErr = error / M_PI;

  // Calculate PID output
  iErr = sumErr;
  // Safeguards for integral part overflow
  if (iErr > 0.5) {
    sumErr = iErr = 0.5;
  } else {
    if (iErr < -0.5) {
      sumErr = iErr = -0.5; 
    } else {
      sumErr += (dErr * I) / 10.0; 
      iErr = sumErr;
    }
  }
  res = dErr * P + iErr - D * (dErr - lastErr);
  debug(L_NOTICE, "motors::pidCalc: error %1.3f, dErr %1.3f, sumErr %1.3f, lasterr %1.3f, res %1.3f\n", error, dErr, sumErr, lastErr, res);
  lastErr = dErr;

  // Check output range
  if (res > 1.0)
    res = 1.0;
  if (res < -1.0)
    res = -1.0;
  return res;
} // float motors::pidCalc(float error, float P, float I, float D) 

// PID controller reset
void motors::pidReset() {
  lastErr = sumErr = 0.0;
} // void motors::pidReset()

// Emergency stop (zero'ing speed with accelStop acceleration)
void motors::emergStop() {
  curStatus = _moveStatus::EMERG_STOP;
  debug(L_INFO, "motors::emergStop\n");
} // void motors::emergStop()

// Turn at a place, with timeout (milliseconds)
void motors::roll(_dir dirRoll, unsigned long ms) {
  courseThing = NULL;
  endTime = ms + millis();
  switch(dirRoll) {
    // Left turn
    case _dir::LEFT:
      leftSpeed = -maxSpeed;
      rightSpeed = maxSpeed;
      curStatus = _moveStatus::ROLL;
      break;

    // Right turn
    case _dir::RIGHT:
      leftSpeed = maxSpeed;
      rightSpeed = -maxSpeed;
      curStatus = _moveStatus::ROLL;
      break;

    // Wrong direction for turn
    default:
      leftSpeed = 0;
      rightSpeed = 0;
      curStatus = _moveStatus::STOP;
      break;
  }
  debug(L_INFO, "motors::roll: dirRoll %d, ms %lu\n", dirRoll, ms);
} // void motors::roll(_dir dirRoll, unsigned long ms)

// Movement, with timeout (milliseconds)
void motors::move(_dir dirMove, unsigned long ms) {
  courseThing = NULL;
  endTime = ms + millis();
  switch(dirMove) {
    // Moving back
    case _dir::BACKWARD:
      leftSpeed = rightSpeed = -maxSpeed;
      curStatus = _moveStatus::MOVE;
      break; 

    // Moving forward
    case _dir::FORWARD:
      leftSpeed = rightSpeed = maxSpeed;
      curStatus = _moveStatus::MOVE;
      break; 

    // Turn with left motor stop
    case _dir::LEFT:
      leftSpeed = 0;
      rightSpeed = maxSpeed;
      curStatus = _moveStatus::MOVE;
      break;

    // Turn with right motor stop 
    case _dir::RIGHT:
      rightSpeed = 0;
      leftSpeed = maxSpeed;
      curStatus = _moveStatus::MOVE;
      break;
  
    // Stop or wrong _dir
    default:
      curStatus = _moveStatus::STOP;
      return;
  }
  debug(L_INFO, "motors::move: dirMove %d, ms %lu\n", dirMove, ms);
} // void motors::move(_dir dirMove, unsigned long ms)

// Starts turn on a place by a sensor reading (compass, perimeter, GPS, etc)
// ms - timeout in milliseconds.
// Sensor must return error of needed course by readCourseError()
void motors::rollCourse(navThing *sensor, unsigned long ms) {
  courseThing = sensor;
  pidReset();
  endTime = ms + millis();
  curStatus = _moveStatus::ROLL_NAV;
  debug(L_INFO, "motors::rollCourse: ms %lu\n", ms);
} // void motors::rollCourse(navThing *sensor, unsigned long ms)

// Starts movement by a sensor reading
// ms - timeout in milliseconds.
// Sensor must return error of needed course by readCourseError()
void motors::moveCourse(navThing *sensor, unsigned long ms) {
  courseThing = sensor;
  pidReset();
  endTime = ms + millis();
  curStatus = _moveStatus::MOVE_NAV;
  debug(L_INFO, "motors::moveCourse: ms %lu\n", ms);
} // void motors::moveCourse(navThing *sensor, unsigned long ms)

// Current PWM of motors
int16_t motors::curPWM(numThing n) {
  if (n == 0)
    return leftPWM;
  else
    return rightPWM;
} // int16_t curPWM(numThing n)

// Hardware initialization
_hwstatus motors::begin() {
  debug(L_DEBUG, (char *) F("motors::begin\n"));
  reverseAll = false;
  #ifdef MOT_DRIVER_DUAL_MC33926
  pinMode(PIN_MOT_LEFTFORW_DIR, OUTPUT);
  digitalWrite(PIN_MOT_LEFTFORW_DIR, LOW);
  pinMode(PIN_MOT_RIGHTFORW_DIR, OUTPUT);
  digitalWrite(PIN_MOT_RIGHTFORW_DIR, LOW);
  pinMode(PIN_MOT_LEFTFORW_FAULT, INPUT_PULLUP);
  pinMode(PIN_MOT_RIGHTFORW_FAULT, INPUT_PULLUP);
  pinMode(PIN_MOT_FORW_ENABLE, OUTPUT);
  digitalWrite(PIN_MOT_FORW_ENABLE, LOW);
  #ifdef MOT_QUAD_WHEELS
  pinMode(PIN_MOT_LEFTBACK_DIR, OUTPUT);
  digitalWrite(PIN_MOT_LEFTBACK_DIR, LOW);
  pinMode(PIN_MOT_RIGHTBACK_DIR, OUTPUT);
  digitalWrite(PIN_MOT_RIGHTBACK_DIR, LOW);
  pinMode(PIN_MOT_LEFTBACK_FAULT, INPUT_PULLUP);
  pinMode(PIN_MOT_RIGHTBACK_FAULT, INPUT_PULLUP);
  pinMode(PIN_MOT_BACK_ENABLE, OUTPUT);
  digitalWrite(PIN_MOT_BACK_ENABLE, LOW);
  #endif
  #endif
  #ifdef MOT_DRIVER_IHM12A1
  pinMode(PIN_MOT_ENABLE, OUTPUT);
  digitalWrite(PIN_MOT_ENABLE, LOW);
  pinMode(PIN_MOT_FAULT, INPUT_PULLUP);
  pinMode(PIN_MOT_REF_SELECT, OUTPUT);
  digitalWrite(PIN_MOT_REF_SELECT, HIGH);
  pinMode(PIN_MOT_LEFT_PWM, OUTPUT);
  digitalWrite(PIN_MOT_LEFT_PWM, LOW);
  pinMode(PIN_MOT_RIGHT_PWM, OUTPUT);
  digitalWrite(PIN_MOT_RIGHT_PWM, LOW);
  pinMode(PIN_MOT_LEFT_DIR, OUTPUT);
  digitalWrite(PIN_MOT_LEFT_DIR, LOW);
  pinMode(PIN_MOT_RIGHT_DIR, OUTPUT);
  digitalWrite(PIN_MOT_RIGHT_DIR, LOW);
  #endif
  #ifdef MOT_DRIVER_DRV8825
  pinMode(PIN_MOT_FORW_ENABLE, OUTPUT);
  digitalWrite(PIN_MOT_FORW_ENABLE, LOW);
  pinMode(PIN_MOT_LEFTFORW_FAULT, INPUT_PULLUP);
  pinMode(PIN_MOT_RIGHTFORW_FAULT, INPUT_PULLUP);
  pinMode(PIN_MOT_LEFTFORW_DIR, OUTPUT);
  digitalWrite(PIN_MOT_LEFTFORW_DIR, LOW);
  pinMode(PIN_MOT_RIGHTFORW_DIR, OUTPUT);
  digitalWrite(PIN_MOT_RIGHTFORW_DIR, LOW);
  pinMode(PIN_MOT_LEFTFORW_STEP, OUTPUT);
  digitalWrite(PIN_MOT_LEFTFORW_STEP, LOW);
  pinMode(PIN_MOT_RIGHTFORW_STEP, OUTPUT);
  digitalWrite(PIN_MOT_RIGHTFORW_STEP, LOW);
  #endif
} // motors::motors()

void motors::setPWM(int16_t left, int16_t right) {
#ifdef MOT_DRIVER_DRV8825
  // Don't change speed if not needed (will mess stepper cycles)
  if ((leftPWM == left) && (rightPWM == right) && (left != 0) && (right != 0))
    return;
#endif
  leftPWM = left;
  rightPWM = right;
  // Inverse direction for opposite motor
  if (reverseAll)
    left = -left;
  else
    right = -right;
  debug(L_NOTICE, (char *) F("motors::setPWM: %hd %hd\n"), left, right);
  #ifdef MOT_DRIVER_DRV8825
  if ((digitalRead(PIN_MOT_FORW_ENABLE) == LOW) && ((leftPWM != 0) || (rightPWM != 0)))
    _motorsWakeUp = DRV8825_WAKEUP;
  #endif
  if (left <= 0) {
    #if defined(MOT_DRIVER_DUAL_MC33926)
    digitalWrite(PIN_MOT_LEFTFORW_DIR, LOW);
    pwm_leftforw.set_duty(MOT_PWM_TICK_WIDTH * abs(left));
    #ifdef MOT_QUAD_WHEELS
    digitalWrite(PIN_MOT_LEFTBACK_DIR, LOW);
    pwm_leftback.set_duty(MOT_PWM_TICK_WIDTH * abs(left));
    #endif
    #endif
    #ifdef MOT_DRIVER_IHM12A1
    digitalWrite(PIN_MOT_LEFT_DIR, LOW);
    analogWrite(PIN_MOT_LEFT_PWM, abs(left));
    #endif
    #ifdef MOT_DRIVER_DRV8825
    digitalWrite(PIN_MOT_LEFTFORW_DIR, HIGH);
    _leftStepsDivider = DRV8825_DIV(left);
    _leftStepsCount = _leftStepsDivider;
    if (left != 0)
      digitalWrite(PIN_MOT_LEFTFORW_STEP, HIGH);
    else
      digitalWrite(PIN_MOT_LEFTFORW_STEP, LOW);
    #endif
  } else {
    #if defined(MOT_DRIVER_DUAL_MC33926)
    digitalWrite(PIN_MOT_LEFTFORW_DIR, HIGH);
    #ifdef MOT_QUAD_WHEELS
    digitalWrite(PIN_MOT_LEFTBACK_DIR, HIGH);
    #endif
    pwm_leftforw.set_duty(MOT_PWM_TICK_WIDTH * (255 - left));
    #ifdef MOT_QUAD_WHEELS
    pwm_leftback.set_duty(MOT_PWM_TICK_WIDTH * (255 - left));
    #endif
    #endif
    #ifdef MOT_DRIVER_IHM12A1
    digitalWrite(PIN_MOT_LEFT_DIR, HIGH);
    analogWrite(PIN_MOT_LEFT_PWM, abs(left));
    #endif
    #ifdef MOT_DRIVER_DRV8825
    digitalWrite(PIN_MOT_LEFTFORW_DIR, LOW);
    _leftStepsDivider = DRV8825_DIV(left);
    _leftStepsCount = _leftStepsDivider;
    digitalWrite(PIN_MOT_LEFTFORW_STEP, HIGH);
    #endif
  }
  if (right <= 0) {
    #if defined(MOT_DRIVER_DUAL_MC33926)
    digitalWrite(PIN_MOT_RIGHTFORW_DIR, LOW);
    pwm_rightforw.set_duty(MOT_PWM_TICK_WIDTH * abs(right));
    #ifdef MOT_QUAD_WHEELS
    digitalWrite(PIN_MOT_RIGHTBACK_DIR, LOW);
    pwm_rightback.set_duty(MOT_PWM_TICK_WIDTH * abs(right));
    #endif
    #endif
    #ifdef MOT_DRIVER_IHM12A1
    digitalWrite(PIN_MOT_RIGHT_DIR, HIGH);
    analogWrite(PIN_MOT_RIGHT_PWM, abs(right));
    #endif
    #ifdef MOT_DRIVER_DRV8825
    digitalWrite(PIN_MOT_RIGHTFORW_DIR, HIGH);
    _rightStepsDivider = DRV8825_DIV(right);
    _rightStepsCount = _leftStepsDivider;
    if (right != 0)
      digitalWrite(PIN_MOT_RIGHTFORW_STEP, HIGH);
    else
      digitalWrite(PIN_MOT_RIGHTFORW_STEP, LOW);
    #endif
  } else {
    #ifdef MOT_DRIVER_DUAL_MC33926
    digitalWrite(PIN_MOT_RIGHTFORW_DIR, HIGH);
    #ifdef MOT_QUAD_WHEELS
    digitalWrite(PIN_MOT_RIGHTBACK_DIR, HIGH);
    #endif
    pwm_rightforw.set_duty(MOT_PWM_TICK_WIDTH * (255 - right));
    #ifdef MOT_QUAD_WHEELS
    pwm_rightback.set_duty(MOT_PWM_TICK_WIDTH * (255 - right));
    #endif
    #endif  
    #ifdef MOT_DRIVER_IHM12A1
    digitalWrite(PIN_MOT_RIGHT_DIR, LOW);
    analogWrite(PIN_MOT_RIGHT_PWM, abs(right));
    #endif
    #ifdef MOT_DRIVER_DRV8825
    digitalWrite(PIN_MOT_RIGHTFORW_DIR, LOW);
    _rightStepsDivider = DRV8825_DIV(right);
    _rightStepsCount = _leftStepsDivider;
    digitalWrite(PIN_MOT_RIGHTFORW_STEP, HIGH);
    #endif
  }
} // void motors::setPWM(int16_t left, int16_t right)

// motors initialization
_status motors::init() {
  // Reset motor driver with STDBY/RST pin
  disableThings();
  delay(1);

  errorStatus = 0;
  #ifdef MOT_DRIVER_DUAL_MC33926
  pwm_leftforw.start(255 * MOT_PWM_TICK_WIDTH, 0);
  pwm_rightforw.start(255 * MOT_PWM_TICK_WIDTH, 0);
  #ifdef MOT_QUAD_WHEELS
  pwm_leftback.start(255 * MOT_PWM_TICK_WIDTH, 0);
  pwm_rightback.start(255 * MOT_PWM_TICK_WIDTH, 0);
  #endif
  #endif
  #ifdef MOT_DRIVER_IHM12A1
  analogWrite(PIN_MOT_LEFT_PWM, 0);
  analogWrite(PIN_MOT_RIGHT_PWM, 0);
  #endif
  #ifdef MOT_DRIVER_DRV8825
  _leftStepsDivider = _rightStepsDivider = 0;
  _motorsWakeUp = 0;
  #endif
  setPWM(0, 0);
  _notMoving = true;
  lastMoveTime = 0;
  return enableThings();
} // _status motors::init()

// Enable motor driver
_status motors::enableThings() { 
  #ifdef MOT_DRIVER_DUAL_MC33926
  digitalWrite(PIN_MOT_FORW_ENABLE, HIGH);
  #ifdef MOT_QUAD_WHEELS
  digitalWrite(PIN_MOT_BACK_ENABLE, HIGH);
  #endif
  #endif
  #ifdef MOT_DRIVER_IHM12A1
  digitalWrite(PIN_MOT_ENABLE, HIGH);
  #endif
  #ifdef MOT_DRIVER_DRV8825
  digitalWrite(PIN_MOT_FORW_ENABLE, HIGH);
  #endif
  return _status::NOERR;
} // _status motors::enableThings()

// Disable motor driver
_status motors::disableThings() {
  #ifdef MOT_DRIVER_DUAL_MC33926
  digitalWrite(PIN_MOT_FORW_ENABLE, LOW);
  #ifdef MOT_QUAD_WHEELS
  digitalWrite(PIN_MOT_BACK_ENABLE, LOW);
  #endif
  #endif
  #ifdef MOT_DRIVER_IHM12A1
  digitalWrite(PIN_MOT_ENABLE, LOW);
  #endif
  #ifdef MOT_DRIVER_DRV8825
  digitalWrite(PIN_MOT_FORW_ENABLE, LOW);
  #endif
  return _status::NOERR;
} // _status motors::enableThings()

// Check for motor faults (overcurrent, driver fault), returns 0 if no errors
uint8_t motors::checkFault() {
  float sumCurrent, current;

  if (errorStatus)
    return errorStatus;
  // Check for motor driver's fault pins
  #ifdef MOT_DRIVER_DUAL_MC33926
  #ifndef MOT_QUAD_WHEELS
  if ((digitalRead(PIN_MOT_LEFTFORW_FAULT) == LOW) || (digitalRead(PIN_MOT_RIGHTFORW_FAULT) == LOW)) {
  #else
  if ((digitalRead(PIN_MOT_LEFTFORW_FAULT) == LOW) || (digitalRead(PIN_MOT_RIGHTFORW_FAULT) == LOW)
      || (digitalRead(PIN_MOT_LEFTBACK_FAULT) == LOW) || (digitalRead(PIN_MOT_RIGHTBACK_FAULT) == LOW)) {
  #endif
  #endif
  #ifdef MOT_DRIVER_IHM12A1
  if (digitalRead(PIN_MOT_FAULT) == LOW) {
  #endif
  #ifdef MOT_DRIVER_DRV8825
  if ((digitalRead(PIN_MOT_LEFTFORW_FAULT) == LOW) || (digitalRead(PIN_MOT_RIGHTFORW_FAULT) == LOW)) {
  #endif
    debug(L_WARNING, (char *) F("motors::checkFault: detected driver fault\n"));
    addError(2);

    errorStatus = 1;
    return errorStatus;
  }
  
  // Check for over-current
  for (numThing i = 0; i < currentSens->numThings(); i++) {
    current = currentSens->readCurrent(i);
    if (current > currentSens->currentMax) {
      debug(L_WARNING, (char *) F("motors::checkFault: detected overcurrent on motor %hu (%f)\n"),
            i, current);
      errorStatus = 2;
      addError(i);
    }
    sumCurrent += current;
  }
  return errorStatus;
} //  uint8_t motors::checkFault()

// Returns soft error status
_hwstatus motors::softError() {
  if (errorStatus)
    return _hwstatus::ERROR;
  #ifdef MOT_DRIVER_DUAL_MC33926
  #ifndef MOT_QUAD_WHEELS
  if (digitalRead(PIN_MOT_FORW_ENABLE) == LOW)
  #else
  if ((digitalRead(PIN_MOT_FORW_ENABLE) == LOW) || (digitalRead(PIN_MOT_BACK_ENABLE) == LOW))
  #endif
  #endif
  #ifdef MOT_DRIVER_IHM12A1
  if (digitalRead(PIN_MOT_ENABLE) == LOW)
  #endif
  #ifdef MOT_DRIVER_DRV8825
  if (digitalRead(PIN_MOT_FORW_ENABLE) == LOW)
  #endif
    return _hwstatus::DISABLED;
  return _hwstatus::ONLINE;
} // _hwstatus softError()

#ifdef MOT_DRIVER_DRV8825
// Pulse steppers (called from due-adc-scan.cpp interrupt 38412Hz routine)
void pulseSteppers() {
  if (_motorsWakeUp) {
    digitalWrite(PIN_MOT_FORW_ENABLE, HIGH);
    _motorsWakeUp--;
    return;
  }
  if (_leftStepsDivider) {
    if (_lowerLeftStep) {
      digitalWrite(PIN_MOT_LEFTFORW_STEP, LOW);
      _lowerLeftStep = false;
    }
    if ((_leftStepsCount--) <= 0) {
      digitalWrite(PIN_MOT_LEFTFORW_STEP, HIGH);
      _lowerLeftStep = true;
      _leftStepsCount = _leftStepsDivider;
    }
  }
  if (_rightStepsDivider) {
    if (_lowerRightStep) {
      digitalWrite(PIN_MOT_RIGHTFORW_STEP, LOW);
      _lowerRightStep = false;
    }
    if ((_rightStepsCount--) <= 0) {
      digitalWrite(PIN_MOT_RIGHTFORW_STEP, HIGH);
      _lowerRightStep = true;
      _rightStepsCount = _rightStepsDivider;
    }
  }
} // void pulseSteppers()
#endif
