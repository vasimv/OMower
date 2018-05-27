// Power control class for OMower
// $Id$

#ifndef _OMOWER_POWER_H
#define _OMOWER_POWER_H

#include <omower-root.h>
#include <Arduino.h>
#include <stdint.h>

// Voltage and current sensors numbers
// Battery voltage and current
#define P_BATTERY 0
// Solar panel voltage output (only voltage sensor)
#define P_SOLAR 1
// Charge port output voltage (only voltage sensor)
#define P_CHARGE 2
// Booster/charge voltage and charge output
#define P_BOOST 3
// Only voltage sensors (main 3.3V and 5V, secondary 5V)
#define P_3V3 4
#define P_5V 5
#define P_5VSEC 6

#define _NUM_VOLTAGE_SENSORS 7

// Current sensors (most of stuff defined in the currentThing)
class currentPow : public currentThing {
public:
  currentPow();

private:
  // Previous value for filter
  volatile uint16_t prevValueBatt;
  volatile uint16_t prevValueBoost;

  uint16_t readRawCurrent(numThing n);
};

// Voltage sensors and controls
class power : public thing {
public:
  // Settings variables:
  // ADC->voltage coefficient for internal ADC
  float kVoltage;
  // ADC->voltage coefficient for MAX11617
  float kVoltageExt;
  // Starts charging if charge voltage is bigger than battery's voltage
  // (also target of solar's battery boost-up regulator)
  float minDiffVoltage;
  // Maximum battery voltage (stops charging at), must be checked at top-level
  float maxBatteryVoltage;
  // Start charging if battery voltage is lower than, must be checked at top-level
  float maxBatteryChargeStart;
  // Emergency shutdown battery level (bistable relay turns off everything),
  // must be checked at top-level
  float minBatteryVoltage;
  // Maximum charge current
  float maxChargeCurrent;
  // Minimum solar battery voltage (Vmp - couple volts)
  float minSolarVoltage;
  

  // Current sensors
  currentPow *currentSens;

  power();

  // Hardware init
  _hwstatus begin();

  // Voltage reading
  float readSensor(numThing n);

  // Approximage power left (in watts)
  float readPowerLeft(numThing n);

  // Enable/disable charge from solar panel
  _status setSolarCharge(boolean charge);

  // Enable/disable charge from charge port
  _status setMainCharge(boolean charge);

  // Emergency shutdown (bistable relay will turn off the robot)
  _status emergShutdown();

  numThing numThings();

  void poll50();

  // Disable power save mode
  _status enableThings();

  // Enable power save mode
  _status disableThings();
  
private:
  // Flag of power save mode
  boolean powerSave;

  // Flags of internal regulators enabled
  boolean enableMain;
  boolean enableSolar;

  // Previous values for the filter
  uint16_t prevValue[7];

  // Charge current regulator status (true - main cycle, false - increasing PWM duty from 0)
  boolean increaseMain;

  // Booster regulator status (true - main cycle, false - increasing PWM duty from 0)
  boolean increaseSolar;

  // Last PWM duty timer values
  int lastSolarDuty;
  int lastMainDuty;

  // read raw value with filter
  uint16_t readRawVoltage(numThing n);

  // Set booster PWM duty
  void setSolar(int duty);

  // Set charger PWM duty
  void setMain(int duty);
};

#endif
