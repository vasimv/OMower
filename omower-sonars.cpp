// Sonars sensors for OMower
// $Id$

#include <omower-sonars.h>
#include <Arduino.h>
#include <string.h>

numThing sonars::numThings() {
  return SONARS_NUM;
} // numThing sonars::numThings()

_locationThings sonars::locThings() {
#if SONARS_NUM == 1
  return _locationThings::JUSTONE;
#elif SONARS_NUM == 2
  return _locationThings::LEFT_RIGHT;
#elif SONARS_NUM == 3
  return _locationThings::LEFT_CENTER_RIGHT;
#elif SONARS_NUM == 4
  return _locationThings::LEFTFORW_RIGHTFORW_LEFTBACK_RIGHTBACK;
#endif
  return _locationThings::SPECIAL;
} // _locationThings sonars::locThings()

uint16_t sonars::readSonar(numThing n) {
  return sonarDist[n];
} // uint16_t sonars::readSonar(numThing n)

// Sonars data from interrupt
struct {
  uint32_t startMillis;
  uint32_t startTicks;
  uint32_t endMillis;
  uint32_t endTicks;
} dataSonars[SONARS_NUM];

// Current sonars pinged (number in array and pin number)
uint8_t nSonarForw = 0;
uint8_t pSonarForw = 0;
uint8_t nSonarBack = 0;
uint8_t pSonarBack = 0;

// Sonars interrupt function (on change) for forward sonar
static void sonarsIntForw() {
  if (digitalRead(pSonarForw)) {
    // Fill start timestamp fields on raise
    if (dataSonars[nSonarForw].startMillis == 0) {
      dataSonars[nSonarForw].startTicks = SysTick->VAL;
      dataSonars[nSonarForw].startMillis = millis();
    }
  } else {
    // Fill start timestamp fields on fall
    dataSonars[nSonarForw].endTicks = SysTick->VAL;
    dataSonars[nSonarForw].endMillis = millis();
  }
} // static void sonarsIntForw()

// Sonars interrupt function (on change) for backward sonar
static void sonarsIntBack() {
  if (digitalRead(pSonarBack)) {
    // Fill start timestamp fields on raise
    if (dataSonars[nSonarForw].startMillis == 0) {
      dataSonars[nSonarBack].startTicks = SysTick->VAL;
      dataSonars[nSonarBack].startMillis = millis();
    }
  } else {
    // Fill start timestamp fields on fall
    dataSonars[nSonarBack].endTicks = SysTick->VAL;
    dataSonars[nSonarBack].endMillis = millis();
  }
} // static void sonarsIntBack()


// Software (re)initialization
_status sonars::init() {
  // Set pins
  pinMode(PIN_SONAR_TRIG1, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG1, LOW);
  pinMode(PIN_SONAR_TRIG2, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG2, LOW);
#if (SONARS_NUM > 2)
  pinMode(PIN_SONAR_TRIG3, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG3, LOW);
  pinMode(PIN_SONAR_TRIG4, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG4, LOW);
  pinMode(PIN_SONAR_TRIG5, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG5, LOW);
  pinMode(PIN_SONAR_TRIG6, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG6, LOW);
  pinMode(PIN_SONAR_ECHO1, INPUT);
  pinMode(PIN_SONAR_ECHO2, INPUT);
  pinMode(PIN_SONAR_ECHO3, INPUT);
  pinMode(PIN_SONAR_ECHO4, INPUT);
  pinMode(PIN_SONAR_ECHO5, INPUT);
  pinMode(PIN_SONAR_ECHO6, INPUT);
#endif

  // Clear data
  memset((uint8_t *) dataSonars, 0, sizeof(dataSonars));
  memset((uint8_t *) sonarDist, 0, sizeof(sonarDist));

  currentSonar = 0;
  enableSonars = true;
  return _status::NOERR;
} // _status sonars::init() {

// Poll routine - ping another pair, process data from previous ping
void sonars::poll20() {
  if (!enableSonars)
    return;

  // Get data from dataSonars array and calculate distances
  processEcho(currentSonar);
  processEcho(currentSonar + SONARS_NUM / 2);

  // Increment to next sonar
  currentSonar = (currentSonar + 1) % (SONARS_NUM / 2);

  // Send pings for one of forward and backward sonars
  sendPing(currentSonar, true);
  sendPing(currentSonar + (SONARS_NUM / 2), false);
} // void sonars::poll20()

void sonars::sendPing(numThing n, boolean forward) {
  // Clear end data
  dataSonars[n].endMillis = 0;
  dataSonars[n].endTicks = 0;
  dataSonars[n].startMillis = 0;
  dataSonars[n].startTicks = 0;

  // Attach interrupt on change
  if (forward) {
    nSonarForw = n;
    pSonarForw = sonarEchos[n];
    attachInterrupt(sonarEchos[n], &sonarsIntForw, CHANGE);
  } else {
    nSonarBack = n;
    pSonarBack = sonarEchos[n];
    attachInterrupt(sonarEchos[n], &sonarsIntBack, CHANGE);
  }

  // Start ping
  digitalWrite(sonarTrigs[n], HIGH);
  delayMicroseconds(30);
  digitalWrite(sonarTrigs[n], LOW);
} // void sonars::sendPing()

void sonars::processEcho(numThing n) {
  uint32_t delay;
  uint16_t dist;

  // Detach interrupt
  detachInterrupt(sonarEchos[n]);

  // Check if we have valid data in
  if (dataSonars[n].endMillis == 0) {
    sonarDist[n] = 0;
    return;
  }
  
  // Calculate raw data (84000 ticks per ms)
  delay = (dataSonars[n].endMillis - dataSonars[n].startMillis) * (SysTick->LOAD + 1)
          + dataSonars[n].startTicks - dataSonars[n].endTicks;

  // 58 cm per uS, 84 ticks per uS
  dist = delay / (HCSR04_CM_PER_US * MCU_FREQ_MHZ);

  // Too long distance (more than 15 meters) for the sonar, perhaps bad data
  if (dist > SONAR_MAX_DIST) 
    dist = 0;

  // Need two measurements to confirm the obstacle
  if ((dist > 0) && (dist < triggerDist)) {
    if ((sonarDist[n] > 0) && (sonarDist[n] < triggerDist))
      sonarFilt[n] = dist;
    else
      sonarFilt[n] = sonarDist[n];
  } else
    sonarFilt[n] = dist;
  sonarDist[n] = dist;
} // void sonars::processEcho(numThing n)


_status sonars::enableThings() {
  enableSonars = true;
  return _status::NOERR;
} // _status sonars::enableThings() 

_status sonars::disableThings() {
  enableSonars = false;
  return _status::NOERR;
} // _status sonars::disableThings()

