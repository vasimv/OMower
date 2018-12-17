// Wheel motors odometry sensors class for OMower
// $Id$

#include <omower-odometry.h>
#include <Arduino.h>
#include <omower-debug.h>
#include <omower-enums.h>
#include <omower-ros.h>
#include <string.h>
#include <stdint.h>

volatile int32_t ticksMotors[_NUM_ODOMETERS_WHEELS];

// Callbacks for pin FALLING state
#ifdef PIN_ODO_A2
void leftForwInt() {
  if (digitalRead(PIN_ODO_A2) == LOW)
    ticksMotors[0]++;
  else
    ticksMotors[0]--;
} // void leftForwInt()
#endif
#ifdef PIN_ODO_B2
void rightForwInt() {
  // Opposite direction for right motor
  if (digitalRead(PIN_ODO_B2) == HIGH) 
    ticksMotors[1]++;
  else
    ticksMotors[1]--;
} // void rightForwInt()
#endif
#ifdef PIN_ODO_C2
void leftBackInt() {
  if (digitalRead(PIN_ODO_C2) == LOW)
    ticksMotors[2]++;
  else
    ticksMotors[2]--;
} // void leftBackInt()
#endif
#ifdef PIN_ODO_D2
void rightBackInt() {
  // Opposite direction for right motor
  if (digitalRead(PIN_ODO_D2) == HIGH)
    ticksMotors[3]++;
  else
    ticksMotors[3]--;
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
  ticksMotors[n] = 0;
  prevTicks[n] = 0;
  return _status::NOERR;
} // _status odometryMotors::resetTicks(numThing n)

// Clear odometry variables
_status odometryMotors::init() {
  debug(L_INFO, (char *) F("odometryMotors::init\n"));
  memset((void *) ticksMotors, 0, sizeof(ticksMotors));
  memset((void *) prevTicks, 0, sizeof(prevTicks));
  memset((void *) targTicks, 0, sizeof(targTicks));
  memset((void *) TPM, 0, sizeof(TPM));
  pollsNum = 0;
  reportToROS();
  return _status::NOERR;
} // _status odometryMotors::init()

// Read current number of ticks (accumulative)
long odometryMotors::readTicks(numThing n) {
  return ticksMotors[n];
} // long odometryMotors::readTicks(numThing n)

// Read current ticks per minute
int32_t odometryMotors::readTPM(numThing n) {
  return TPM[n];
} // int16_t odometryMotors::readTPM(numThing n)

// Read current RPM
int16_t odometryMotors::readRPM(numThing n) {
  return TPM[n] / ticksRev;
} // int16_t odometryMotors::readRPM(numThing n)

// Calculate TPM for each motor
void odometryMotors::poll10() {
  long curTicks;

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
} // void odometryMotors::poll10()

float odometryMotors::readCourseError() {
  return 1000;
} // float odometryMotors::readCourseError()

// Force report odometers readings to ROS
void odometryMotors::reportToROS() {
#ifdef USE_ROS
  oROS.reportToROS(reportSensor::ODOMETRY, (uint8_t *) ticksMotors, _NUM_ODOMETERS_WHEELS);
#endif
} // void odometryMotors::reportToROS()
