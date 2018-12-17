// Digital input-output stuff for buttons, leds, etc
// $Id$

#include <omower-defs.h>
#include <omower-buttonsleds.h>
#include <omower-debug.h>
#include <omower-ros.h>
#include <stdint.h>

// Hardware init
_hwstatus buttonsLeds::begin() {
  debug(L_DEBUG, (char *) F("buttonLeds::begin\n"));
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BOARDLED, OUTPUT);
  digitalWrite(PIN_BOARDLED, LOW);
#ifndef EXTLED1_INVERSE
  pinMode(PIN_EXTLED1, OUTPUT);
  digitalWrite(PIN_EXTLED1, LOW);
#else
  digitalWrite(PIN_EXTLED1, HIGH);
  pinMode(PIN_EXTLED1, OUTPUT);
  digitalWrite(PIN_EXTLED1, HIGH);
#endif
  pinMode(PIN_EXTLED2, OUTPUT);
  digitalWrite(PIN_EXTLED2, LOW);
  pinMode(PIN_EXTLED3, OUTPUT);
  digitalWrite(PIN_EXTLED3, LOW);
  pinMode(PIN_BEEPER, OUTPUT);
  digitalWrite(PIN_BEEPER, LOW);
  reportToROS();
  return _hwstatus::ONLINE;
} // _hwstatus buttonsLeds::begin()

#ifdef USE_ROS
uint8_t rosButtonLeds[6];
#endif

// Read button/led state
boolean buttonsLeds::readSensor(numThing n) {
  boolean res;

  switch (n) {
    case T_BEEPER: 
      res = (digitalRead(PIN_BEEPER) == HIGH);
      break;
    case T_BUTTON:
      res = (digitalRead(PIN_BUTTON) == LOW);
      break;
    case T_BOARDLED:
      res = (digitalRead(PIN_BOARDLED) == HIGH);
      break;
#ifndef EXTLED1_INVERSE
    case T_EXTLED1:
      res =  (digitalRead(PIN_EXTLED1) == HIGH);
#else
    case T_EXTLED1:
      res = (digitalRead(PIN_EXTLED1) == LOW);
#endif
      break;
    case T_EXTLED2:
      res = (digitalRead(PIN_EXTLED2) == HIGH);
      break;
    case T_EXTLED3:
      res = (digitalRead(PIN_EXTLED3) == HIGH);
      break;
    default:
      return false;
  }
#ifdef USE_ROS
  rosButtonLeds[n] = (uint8_t) res;
#endif
  return res;
} // boolean buttonsLeds::readSensor(numThing n)

// Set LED/beeper state
void buttonsLeds::setLED(numThing n, boolean state) {
  switch (n) {
    case T_BEEPER:
      digitalWrite(PIN_BEEPER, state ? HIGH : LOW);
      break;
    case T_BOARDLED:
      digitalWrite(PIN_BOARDLED, state ? HIGH : LOW);
      break;
#ifndef EXTLED1_INVERSE
    case T_EXTLED1:
      digitalWrite(PIN_EXTLED1, state ? HIGH : LOW);
#else
    case T_EXTLED1:
      digitalWrite(PIN_EXTLED1, state ? LOW : HIGH);
#endif
      break;
    case T_EXTLED2:
      digitalWrite(PIN_EXTLED2, state ? HIGH : LOW);
      break;
    case T_EXTLED3:
      digitalWrite(PIN_EXTLED3, state ? HIGH : LOW);
      break;
    default:
      return;
  }
#ifdef USE_ROS
  rosButtonLeds[n] = (uint8_t) state;
#endif
  reportToROS();
} // void buttonsLeds::setLED(numThing n, boolean state)

// Force report things to ROS
void buttonsLeds::reportToROS() {
#ifdef USE_ROS
  oROS.reportToROS(reportSensor::BUTTONS, rosButtonLeds, 6);
#endif
} // void buttonsLeds::reportToROS()
