// Digital input-output stuff for buttons, leds, etc
// $Id$

#include <omower-defs.h>
#include <omower-buttonsleds.h>
#include <omower-debug.h>

// Hardware init
_hwstatus buttonsLeds::begin() {
  debug(L_DEBUG, (char *) F("buttonLeds::begin\n"));
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BOARDLED, OUTPUT);
  digitalWrite(PIN_BOARDLED, LOW);
  pinMode(PIN_EXTLED1, OUTPUT);
  digitalWrite(PIN_EXTLED1, LOW);
  pinMode(PIN_EXTLED2, OUTPUT);
  digitalWrite(PIN_EXTLED2, LOW);
  pinMode(PIN_EXTLED3, OUTPUT);
  digitalWrite(PIN_EXTLED3, LOW);
  pinMode(PIN_BEEPER, OUTPUT);
  digitalWrite(PIN_BEEPER, LOW);
  return _hwstatus::ONLINE;
} // _hwstatus buttonsLeds::begin()

// Read button/led state
boolean buttonsLeds::readSensor(numThing n) {
  switch (n) {
    case T_BEEPER:
      return (digitalRead(PIN_BEEPER) == HIGH);
    case T_BUTTON:
      return (digitalRead(PIN_BUTTON) == LOW);
    case T_BOARDLED:
      return (digitalRead(PIN_BOARDLED) == HIGH);
    case T_EXTLED1:
      return (digitalRead(PIN_EXTLED1) == HIGH);
    case T_EXTLED2:
      return (digitalRead(PIN_EXTLED2) == HIGH);
    case T_EXTLED3:
      return (digitalRead(PIN_EXTLED3) == HIGH);
    default:
      return false;
  }
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
    case T_EXTLED1:
      digitalWrite(PIN_EXTLED1, state ? HIGH : LOW);
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
} // void buttonsLeds::setLED(numThing n, boolean state)
