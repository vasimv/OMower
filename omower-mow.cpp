// OMower mowing motor class
// $Id$

#include <omower-defs.h>
#include <Arduino.h>
#include "omower-constants.h"
#include "omower-debug.h"
#include "omower-mow.h"
#include <pwm_lib.h>

using namespace arduino_due::pwm_lib;

#ifdef CH_MOW_PPM
pwm<pwm_pin::CH_MOW_PPM> pwm_mow;
#endif

// Constructor
motorMow::motorMow() {
  currentSens = NULL;
  // odometryMow = NULL;
} // motorMow::motorMow()

// Called 10 times per second (modulation, overcurrent check)
void motorMow::poll10() {
  uint16_t speedSet;

  if (errorStatus)
    return;

  // No modulation, just set the speed
  if (modulateSpeed == 0)
    speedSet = maxSpeed;
  else {
    // Variable speed
    uint8_t diffCycle = modulateSpeed / 8;
    uint8_t minSpeed = maxSpeed - modulateSpeed;

    speedSet = minSpeed + cycle * diffCycle;
    if (millis() > (lastCycle + modulatePeriod / 8)) {
      lastCycle = millis();     
      cycle++;
    }
  }

  // Check if we have current sensor (overcurrent protection, calibration)
  if (currentSens) { 
    float current = currentSens->readCurrent(0);
    
    // Overcurrent protection
    if (current > currentSens->currentMax) {
      errorStatus = 1;
      disableThings();

      debug(L_WARNING, (char *) F("motorMow::poll10: Overcurrent %f > %f, turning off\n"),
            current, currentSens->currentMax);
      return;
    }
    // Calibrate mowing motor current sensor if motor is not rotating long time
    if (maxSpeed != 0)
      lastOn = millis();
    else {
      if ((lastOn - millis()) > 20000)
        currentSens->calibCurrent(0);
    }
  }

  if (currentSpeed == speedSet)
    return;

  // Calculate current speed with acceleration
  if (currentSpeed > speedSet) {
    if (currentSpeed < accelStop)
      currentSpeed = 0;
    else
      currentSpeed = currentSpeed - accelStop;
  } else {
    if (((uint16_t) currentSpeed + (uint16_t) accel) > speedSet)
      currentSpeed = speedSet;
    else
      currentSpeed = currentSpeed + accel;
  }
  setSpeed(currentSpeed);
} // void motorMow::poll10()

// Reverse direction of mowing motor
void motorMow::reverseMow() {
  reverseDir = !reverseDir;
  setSpeed(currentSpeed);
} // void motorMow::reverseMow()

void motorMow::setSpeed(uint8_t speed) {
  int32_t duty = 1;

  if (reverseDir)
    duty = -1;
  // Calculate duty with integer sizes in mind
  duty = MOW_PWM_ZEROMIDDLE
         + ((duty * (int32_t) MOW_PWM_OFFSET * (int32_t) speed) / 65535) * (int32_t) maxPWM;
#ifdef CH_MOW_PPM
  pwm_mow.set_duty(duty);
#endif
} // void motorMow::setSpeed(int8_t speed)

_status motorMow::setHeight(uint8_t height) {
  int steps, res;

  if (height == heightMow)
    return _status::NOERR;
  // Move stepper
  steps = (heightMow - height) * STEPS_PER_MM;
  res = stepMove(steps);
  // Calculate real height based on steps number
  if (steps > 0)
    heightMow = heightMow - res / STEPS_PER_MM;
  else 
    heightMow = heightMow + res / STEPS_PER_MM;
  return _status::NOERR;
} // _status motorMow::setHeight(uint8_t height)


// Hardware initialization
_hwstatus motorMow::begin() {
  debug(L_DEBUG, (char *) F("motorMow::begin\n"));
  pinMode(PIN_MOW_RESET, OUTPUT);
  digitalWrite(PIN_MOW_RESET, LOW);
  pinMode(PIN_MOW_STEP, OUTPUT);
  digitalWrite(PIN_MOW_STEP, LOW);
  pinMode(PIN_MOW_DIR, OUTPUT);
  pinMode(PIN_MOW_FAULT1, INPUT_PULLUP);
  pinMode(PIN_MOW_FAULT2, INPUT_PULLUP);
  pinMode(PIN_MOW_S_END, INPUT_PULLUP);
#ifdef CH_MOW_PPM
  pwm_mow.start(MOW_PWM_PERIOD, MOW_PWM_ZEROMIDDLE);
#endif
  maxSpeed = 0;
  currentSpeed = 0;
  return _hwstatus::ONLINE;
} // _hwstatus mow::begin()

// Send n pulses to stepper driver (with fault checks and end switch)
// Returns number of pulses before fault or end switch
int motorMow::stepMove(int steps) {
  int i;
  uint8_t tmpSpeed;
  boolean ignoreEnd;

  debug(L_NOTICE, (char *) F("Moving stepper for %d steps, initial pos %hu\n"), steps, heightMow);

  // Turn off mowing motor
  maxSpeed = 0;
  currentSpeed = 0;
  setSpeed(0);
  delay(5000);

  // Enable stepper driver and pause few milliseconds
  digitalWrite(PIN_MOW_RESET, HIGH);
  delay(5);

  // Set direction
  if (steps < 0) {
    digitalWrite(PIN_MOW_DIR, HIGH);
    steps = -steps;
    ignoreEnd = false;
  } else {
    digitalWrite(PIN_MOW_DIR, LOW);
    ignoreEnd = true;
  }

  for (i = 0; i < steps; i++) {
    // up to 1000 pulses per second
    digitalWrite(PIN_MOW_STEP, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_MOW_STEP, LOW);
    delayMicroseconds(1990);
    if ((digitalRead(PIN_MOW_FAULT1) == LOW) || (digitalRead(PIN_MOW_FAULT2) == LOW)
        || (ignoreEnd && (digitalRead(PIN_MOW_S_END) == LOW))) {
      debug(L_NOTICE, (char *) F("Detected fault or end condition at step %d (%d %d %d)\n"), i,
            digitalRead(PIN_MOW_FAULT1), digitalRead(PIN_MOW_FAULT2), digitalRead(PIN_MOW_S_END));
      break;
    }
  }

  // Disable stepper driver
  delay(50);
  digitalWrite(PIN_MOW_RESET, LOW);
  maxSpeed = tmpSpeed;
  return i;
} // int motorMow::stepMove(int steps)

// Software (re)initialization
_status motorMow::init() {
  uint8_t tmpHeight = heightMow;

  // reset mowing motor speed
  setSpeed(0);
  reverseDir = false;
  currentSpeed = 0;

  // Move mowing motor up to find zero height
  stepMove(STEPS_PER_MM * heightMowMin * 1.2);
  heightMow = 0;
  setHeight(tmpHeight);

  lastOn = millis();
  errorStatus = 0;

  return _status::NOERR;
} // _status motorMow::init()

// Returns soft error status
_hwstatus motorMow::softError() {
  if (errorStatus)
    return _hwstatus::ERROR;
  return _hwstatus::ONLINE;
} // _hwstatus motorMow::softError()

// Disable motor driver
_status motorMow::disableThings() {
  maxSpeed = 0;
  setSpeed(0);
} // _status motorMow::disableThings()

// Enable motor driver
_status motorMow::enableThings() {

} // _status motorMow::enableThings()
