// Power control class for OMower
// $Id$

#include <omower-defs.h>
#include <omower-power.h>
#include <string.h>
#include <Arduino.h>
#include <due-adc-scan.h>
#include <max11617-adc-scan.h>
#include <omower-debug.h>
#include <pwm_lib.h>
#include <omower-ros.h>

// PWM objects for fan and charge/boost control
using namespace arduino_due::pwm_lib;
#ifndef NO_FAN_CONTROL
pwm<pwm_pin::CH_PWM_FAN> pwmPow_fan;
#endif
#ifndef NO_CHARGE_CONTROL
pwm<pwm_pin::CH_PWM_BOOST> pwmPow_boost;
pwm<pwm_pin::CH_PWM_CC> pwmPow_charge;
#endif

#define MAX_CHARGE_PWM (CHARGE_PWM_PERIOD - CHARGE_PWM_PERIOD/8)

// Set solar panel duty output
void power::setSolar(int duty) {
  if (duty > MAX_CHARGE_PWM)
    duty = MAX_CHARGE_PWM;
  if (duty < 0)
    duty = 0;
#ifndef NO_CHARGE_CONTROL
  pwmPow_boost.set_duty(duty);
#endif
  lastSolarDuty = duty;
  debug(L_DEBUG, (char *) F("P: s %d\n"), duty);
} // void power::setSolar(int duty)

// Set main charger duty output
void power::setMain(int duty) {
  if (duty > MAX_CHARGE_PWM)
    duty = MAX_CHARGE_PWM;
  if (duty < 0)
    duty = 0;
#ifndef NO_CHARGE_CONTROL
  pwmPow_charge.set_duty(duty);
#endif
  lastMainDuty = duty;

  // Set charge flag
  if (duty == 0) {
    if ((millis() - lastCharge)  > 120000)
      chargingOn = false;
  } else {
    chargingOn = true;
    lastCharge = millis();
  }
  debug(L_DEBUG, (char *) F("P: c %d\n"), duty);
} // void power::setMain(int duty)

// We make them static to use in 
float volts[_NUM_VOLTAGE_SENSORS];
float currBatt, currBoost;

// Reads all voltages, updates PWM on boost and charge
void power::poll50() {
  float maxCurrent;
  float diffVolts;
  boolean canUseMain = false;
  boolean canUseSolar = false;
  boolean storeProbe = false;
  uint32_t diffProbe;
  boolean probing = false;

  // Check if we have to turn off charging to probe voltage
  if (chargingOn) {
    diffProbe = millis() - lastProbe;
    // Do not initiate probing right after charge is on
    if (diffProbe > 180000) {
      lastProbe = millis();
      diffProbe = 0;
    }
    // Do probing every 60 seconds for 2 seconds
    if (diffProbe > 60000)
      probing = true;
    // Check if we can safely store probed voltage
    if (diffProbe > 60500) {
      storeProbe = true;
      probing = false;
      lastProbe = millis();
    }
  }

  // Read voltages and currents to update filtered values
  for (numThing i = 0; i < _NUM_VOLTAGE_SENSORS; i++) {
    readRawVoltage(i);
    if ((i == P_BATTERY) && (!chargingOn || storeProbe))
      battProbe = readRawSensor(i);
    volts[i] = readRawSensor(i);
  }

  // Read current sensors
  if (currentSens) {
    currBatt = currentSens->readCurrent(P_BATTERY);
    currBoost = currentSens->readCurrent(P_BOOST);
  }

  debug(L_DEBUG, (char *) F("P: %g (%g) %g %g %g %g %g\n"), volts[P_BATTERY], battProbe, volts[P_CHARGE],
        volts[P_BOOST], volts[P_SOLAR], currBoost, currBatt);

  canUseMain = false;
  canUseSolar = false;

  // Check if battery need charging, then that we have charge port voltage - use it then instead solar panel
  if ((millis() > 30000) && !probing && (volts[P_BATTERY] < maxBatteryVoltage)) {
    if (volts[P_CHARGE] > volts[P_BATTERY])
      canUseMain = true;
    else {
      // Check if solar voltage is enough
      if (volts[P_SOLAR] > (minSolarVoltage - 2.0f))
        canUseSolar = true;
    }

    // Lower maximum charge current when we're almost done
    maxCurrent = maxChargeCurrent;
    diffVolts = (maxBatteryVoltage - volts[P_BATTERY]) / maxBatteryVoltage;
    if (diffVolts < 0.013f)
      maxCurrent = maxChargeCurrent / 8;
    else {
      if (diffVolts < 0.027f)
        maxCurrent = maxChargeCurrent / 4;
      else
        if (diffVolts < 0.041f) 
          maxCurrent = maxChargeCurrent / 2;
    }
  }

  // Reset main duty timer if we're off
  if ((!enableMain || !canUseMain) && (!enableSolar || !canUseSolar))
    setMain(0);
  
  // Reset solar booster duty timer
  if (!enableSolar || !canUseSolar)
    setSolar(0);

  // Charging from charge port, just regulate current output
  if (canUseMain) {
    if ((volts[P_CHARGE] > (volts[P_BATTERY] + minDiffVoltage))
        && (chargingOn || (volts[P_BATTERY] < maxBatteryChargeStart))) {
      // Charge current protection
      if (currBoost > (maxChargeCurrent * 1.5)) {
        debug(L_INFO, (char *) F("power::poll50: Too high charge curret %f, switching off charge\n"),
              currBoost);
        setMain(0);
      } else {
        // Regulate output
        if (currBoost < maxCurrent)
          lastMainDuty++;
        else
          lastMainDuty--;
        setMain(lastMainDuty);
      }
    } else
      setMain(0);
  }

  // Charge from solar panel, regulate booster and current
  if (canUseSolar && (chargingOn || (volts[P_BATTERY] < maxBatteryChargeStart))) {
    // Boost voltage more if needed
    if (volts[P_BOOST] < (volts[P_BATTERY] + minDiffVoltage) * 1.1)
      lastSolarDuty++;
    else
      lastSolarDuty--;
    // Input solar voltage protection
    if (volts[P_SOLAR] > (volts[P_BATTERY] * 3)) {
      setSolar(0);
      setMain(0);
    } else
      setSolar(lastSolarDuty);

    // Check if the booster at the maximum, decrease charging current
    if ((lastSolarDuty >= MAX_CHARGE_PWM)
        || (volts[P_SOLAR] < minSolarVoltage))
      lastMainDuty--;
    else {
      // Booster output is sufficient, decrease current output otherwise
      if (volts[P_BOOST] > (volts[P_BATTERY] + minDiffVoltage)) {
        // Solar panel current is not more than half of maximum charge current to prevent booster overload
        if (currBoost > (maxCurrent / 2))
          lastMainDuty--;
        else
          lastMainDuty++;
      } else
        lastMainDuty--;
    }

    // Charge current peak protection
    if (currBoost > (maxChargeCurrent * 1.5)) {
      debug(L_INFO, (char *) F("power::poll50: Too high charge curret %f, switching off charge\n"),
            currBoost);
      setMain(0);
    } else 
      setMain(lastMainDuty);
  }

#ifndef NO_FAN_CONTROL
  // Check power save mode and disable/enable fan when no charge
  if (powerSave) {
    if (chargingOn)
      pwmPow_fan.set_duty(FAN_PWM_PERIOD / 2);
    else
      pwmPow_fan.set_duty(0);
  }
#endif
} // void power::poll50()

float readPowerLeft(numThing n) {
  return 10;
} // float readPowerLeft(numThing n)

// Constructor, initialization
power::power() {
  // Clear variables
  memset((uint8_t *) prevValue, 0, sizeof(prevValue));

  // Set variables
  kVoltage = K_ADC_VOLTAGE_INT;
  kVoltageExt = K_ADC_VOLTAGE_EXT;
  currentSens = NULL;
  enableMain = false;
  enableSolar = false;

  minDiffVoltage = 5.0;
  maxBatteryVoltage = 14.6;
  maxBatteryChargeStart = 14.2;
  minBatteryVoltage = 10.0;
  minSolarVoltage = 12.0;
  maxChargeCurrent = 0.5;
  powerSave = false;
  lastProbe = 0;
  chargingOn = false;
} // power::power()

// Enable charging from main charge port
_status power::setMainCharge(boolean charge) {
  enableMain = charge;
  return _status::NOERR;
} // _status power::setMainCharge(boolean charge)

// Enable charging from solar panel (enabling booster)
_status power::setSolarCharge(boolean charge) {
  enableSolar = charge;
  return _status::NOERR;
} // _status power::setSolarCharge(boolean charge)

// Hardware init
_hwstatus power::begin() {
  debug(L_DEBUG, (char *) F("power::begin\n"));
  // Init pwm channels (50% fan, 0 at charge and boost)
#ifndef NO_FAN_CONTROL
  pwmPow_fan.start(FAN_PWM_PERIOD, FAN_PWM_PERIOD / 2);
#endif
#ifndef NO_CHARGE_CONTROL
  pwmPow_boost.start(CHARGE_PWM_PERIOD, 0);
  pwmPow_charge.start(CHARGE_PWM_PERIOD, 0);
#endif
  pinMode(PIN_SHUTDOWN, OUTPUT);
  digitalWrite(PIN_SHUTDOWN, LOW);
  powerSave = false;
  reportToROS();
  if (currentSens)
    currentSens->reportToROS();
  return _hwstatus::ONLINE;
} // _hwstatus power::begin()

// Emergency shutdown
_status power::emergShutdown() {
  boolean tmpEnableMain = enableMain;
  boolean tmpEnableSolar = enableSolar;

  enableMain = false;
  enableSolar = false;
  delay(100);
  debug(L_NOTICE, (char *) F("Turning off power\n"));
  pinMode(PIN_SHUTDOWN, OUTPUT);
  digitalWrite(PIN_SHUTDOWN, HIGH);
  delay(1000);
  digitalWrite(PIN_SHUTDOWN, LOW);
  enableMain = tmpEnableMain;
  enableSolar = tmpEnableSolar;
  return _status::NOERR;
} // _status power::emergShutdown()

numThing power::numThings() {
  return 7;
} // numThing power::numThings()

float power::readRawSensor(numThing n) {
  if (n < P_3V3)
    return prevValue[n] * kVoltage;
  else
    return prevValue[n] * kVoltageExt;
} // float power::readRawSensor(numThing n)

// Report true battery voltage when charging is on
float power::readSensor(numThing n) {
  if ((n == P_BATTERY) && chargingOn)
    return battProbe;

  return readRawSensor(n);
} // float power::readSensor(numThing n)

uint16_t power::readRawVoltage(numThing n) {
  uint16_t value;

  switch (n) {
    case P_BATTERY:
      #ifndef DISABLE_BATTERY_CHECK
      value = adcArr[POW_VOLTAGE_BATT].lastRead;
      #else
      value = 12.0f / kVoltage;
      #endif
      break;
    case P_SOLAR:
      value = adcArr[POW_VOLTAGE_SOLAR].lastRead;
      break;
    case P_CHARGE:
      value = adcArr[POW_VOLTAGE_CHARGE].lastRead;
      break;
    case P_BOOST:
      value = adcArr[POW_VOLTAGE_BOOST].lastRead;
      break;
    case P_3V3:
      value = adc11617Arr[POW_VOLTAGE_3V3];
      break;
    case P_5V:
      value = adc11617Arr[POW_VOLTAGE_5V];
      break;
    case P_5VSEC:
      value = adc11617Arr[POW_VOLTAGE_5VSEC];
      break;
    default:
      return 0;
  }
  value = (value + 4 * prevValue[n]) / 5;
  prevValue[n] = value;
  return value;
} // uint16_t power::readRawVoltage(numThing n)

// current sensor constructor (clear prev values)
currentPow::currentPow() {
  prevValueBatt = 2048;
  prevValueBoost = 2048;
  zeroOffset = 2048;
  kCurrent = K_ADC_CURRENT_ACS725_30;
} // currentPow::currentPow()

uint16_t currentPow::readRawCurrent(numThing n) {
  uint16_t value;
  volatile uint16_t *prevValue;

  if (n == P_BATTERY) {
    value = adcArr[POW_CURRENT_TOTAL].lastRead;
    prevValue = &prevValueBatt;
  } else {
    value = adcArr[POW_CURRENT_CHARGE].lastRead;
    prevValue = &prevValueBoost;
  }
  // Filter
  value = (value + 4 * (*prevValue)) / 5;
  *prevValue = value;
  return value;
} // uint16_t currentPow::readRawCurrent(numThing n)

// Disable power save mode
_status power::enableThings() {
  powerSave = false;
  return _status::NOERR;
} // _status power::enableThings()

// Enable power save mode
_status power::disableThings() {
  powerSave = true;
  return _status::NOERR;
} // _status power::disableThings()

#ifdef USE_ROS
float rosCurrents[2];
#endif

// Force report current sensors to ROS
void currentPow::reportToROS() {
#ifdef USE_ROS
  oROS.reportToROS(reportSensor::CURRENT, (uint8_t *) &currBatt, 2);
#endif
} // void currentPow::reportToROS()

// Force report voltages to ROS
void power::reportToROS() {
#ifdef USE_ROS
  oROS.reportToROS(reportSensor::VOLTAGE, (uint8_t *) volts, _NUM_VOLTAGE_SENSORS);
#endif
} // void power::reportToROS()
