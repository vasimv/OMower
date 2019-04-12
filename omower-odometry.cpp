// Wheel motors odometry sensors class for OMower
// $Id$

#include <omower-odometry.h>
#include <Arduino.h>
#include <omower-debug.h>
#include <omower-enums.h>
#include <omower-ros.h>
#include <string.h>
#include <stdint.h>

#if _NUM_ODOMETERS_WHEELS > 0
volatile int32_t ticksMotors[_NUM_ODOMETERS_WHEELS];
#endif

// Callbacks for pin FALLING state
#ifdef PIN_ODO_A2
void leftForwInt() {
#if _NUM_ODOMETERS_WHEELS > 0
  if (digitalRead(PIN_ODO_A2) == LOW)
    ticksMotors[0]++;
  else
    ticksMotors[0]--;
#endif
} // void leftForwInt()
#endif
#ifdef PIN_ODO_B2
void rightForwInt() {
#if _NUM_ODOMETERS_WHEELS > 0
  // Opposite direction for right motor
  if (digitalRead(PIN_ODO_B2) == HIGH) 
    ticksMotors[1]++;
  else
    ticksMotors[1]--;
#endif
} // void rightForwInt()
#endif
#ifdef PIN_ODO_C2
void leftBackInt() {
#if _NUM_ODOMETERS_WHEELS > 0
  if (digitalRead(PIN_ODO_C2) == LOW)
    ticksMotors[2]++;
  else
    ticksMotors[2]--;
#endif
} // void leftBackInt()
#endif
#ifdef PIN_ODO_D2
void rightBackInt() {
#if _NUM_ODOMETERS_WHEELS > 0
  // Opposite direction for right motor
  if (digitalRead(PIN_ODO_D2) == HIGH)
    ticksMotors[3]++;
  else
    ticksMotors[3]--;
#endif
} // void rightBakcInt()
#endif

// Sets interrupts callbacks
_hwstatus odometryMotors::begin() {
#ifdef PIN_ODO_A1
  pinMode(PIN_ODO_A1, INPUT);
  pinMode(PIN_ODO_A2, INPUT);
  attachInterrupt(PIN_ODO_A1, leftForwInt, RISING);
#endif
#ifdef PIN_ODO_B1
  pinMode(PIN_ODO_B1, INPUT);
  pinMode(PIN_ODO_B2, INPUT);
  attachInterrupt(PIN_ODO_B1, rightForwInt, RISING);
#endif
#ifdef PIN_ODO_C1
  pinMode(PIN_ODO_C1, INPUT);
  pinMode(PIN_ODO_C2, INPUT);
  attachInterrupt(PIN_ODO_C1, leftBackInt, RISING);
#endif
#ifdef PIN_ODO_D1
  pinMode(PIN_ODO_D1, INPUT);
  pinMode(PIN_ODO_D2, INPUT);
  attachInterrupt(PIN_ODO_D1, rightBackInt, RISING);
#endif
  // Lyxmotion 20RPM motor with YC2010 encoder (4 ticks per revolution, 99.4:1 reductor)
  ticksRev = 1980;
  // 10" wheel
  ticksCm = 0.05065937;
  return _hwstatus::ONLINE;
} // _hwstatus odometryMotors::begin()

_status odometryMotors::resetTicks(numThing n) {
#if _NUM_ODOMETERS_WHEELS > 0
  ticksMotors[n] = 0;
  prevTicks[n] = 0;
#endif
  return _status::NOERR;
} // _status odometryMotors::resetTicks(numThing n)

// Clear odometry variables
_status odometryMotors::init() {
  debug(L_INFO, (char *) F("odometryMotors::init\n"));
#if _NUM_ODOMETERS_WHEELS > 0
  memset((void *) ticksMotors, 0, sizeof(ticksMotors));
  memset((void *) prevTicks, 0, sizeof(prevTicks));
  memset((void *) TPM, 0, sizeof(TPM));
#endif
  memset((void *) targTicks, 0, sizeof(targTicks));
  pollsNum = 0;
  reportToROS();
  return _status::NOERR;
} // _status odometryMotors::init()

// Read current number of ticks (accumulative)
long odometryMotors::readTicks(numThing n) {
#if _NUM_ODOMETERS_WHEELS > 0
  return ticksMotors[n];
#else
  return 0;
#endif
} // long odometryMotors::readTicks(numThing n)

// Read current ticks per minute
int32_t odometryMotors::readTPM(numThing n) {
#if _NUM_ODOMETERS_WHEELS > 0
  return TPM[n];
#else
  return 0;
#endif
} // int16_t odometryMotors::readTPM(numThing n)

// Read current RPM
int16_t odometryMotors::readRPM(numThing n) {
#if _NUM_ODOMETERS_WHEELS > 0
  return TPM[n] / ticksRev;
#else
  return 0;
#endif
} // int16_t odometryMotors::readRPM(numThing n)

// Calculate TPM for each motor
void odometryMotors::poll10() {
  long curTicks;

#if _NUM_ODOMETERS_WHEELS > 0
  pollsNum++;
  // Calculate two times per second
  if (pollsNum >= 5) {
    debug(L_NOTICE, (char *) F("odometryMotors::poll10(/5): "));
    for (uint8_t i = 0; i < _NUM_ODOMETERS_WHEELS; i++) {
      curTicks = ticksMotors[i];
      TPM[i] = ((curTicks - prevTicks[i]) / pollsNum) * 600;
      prevTicks[i] = curTicks;
      debug(L_NOTICE, (char *) F("%ld (%ld)  "), curTicks, TPM[i]);
    }
    pollsNum = 0;
    debug(L_NOTICE, "\n");
  }
#endif
} // void odometryMotors::poll10()

// Not implemented yet
float odometryMotors::readCourseError() {
  return 1000;
} // float odometryMotors::readCourseError()

// Force report odometers readings to ROS
void odometryMotors::reportToROS() {
#ifdef USE_ROS
#if _NUM_ODOMETERS_WHEELS > 0
  oROS.reportToROS(reportSensor::ODOMETRY, (uint8_t *) ticksMotors, _NUM_ODOMETERS_WHEELS);
#endif
#endif
} // void odometryMotors::reportToROS()

numThing odometryMotors::numThings() {
  return _NUM_ODOMETERS_WHEELS;
} // numThing odometeryMotors::numThings()
