// PINs and other stuff definitions for OMover v3 PCB
// with 2x dual MC33926 motor driver board (4 wheels)
// Brushless mowing motor (ESC on PWM-E channel)

#ifndef _DEFS_OMOWER_H
#define _DEFS_OMOWER_H

#include <omower-debug.h>
#include <pwm_lib.h>

// Frequency in MHz
#define MCU_FREQ_MHZ 84

// Use EMAC/HSMCI/SSC interrupts as software generated interrupts for better multitasking
// Drawbacks - you cannot use secondary I2C and primary I2C bus from poll50() functions only
#define USE_SOFTWARE_INTERRUPTS

// Uncomment for arduino due without 32.768kHz crystal
// #define NO_RTC_CRYSTAL

// Size of nvmem available for writing/reading, for internal flash must be less than 2048 (one
// flash's page size), should be (32 * 2**N)
#define NVMEM_SIZE 512

// Using internal program flash as NVMEM 
#define NVMEM_INTERNAL_FLASH

// How much blocks (NVMEM_SIZE bytes) of the internal flash will be used as NVMEM
// (they will be written sequentially to prevent wear-out)
#define NVMEM_BLOCKS 32

// Invert external LED #1
// #define EXTLED1_INVERSE

// Channel&pin definition for pwm_lib
#define CH_PWM_A PWML0_PC2
#define CH_PWM_B PWML1_PC4
#define CH_PWM_C PWML2_PA20
#define CH_PWM_D PWML3_PC8
#define CH_PWM_E PWML4_PC21
#define CH_PWM_FAN PWML5_PC22
#define CH_PWM_BOOST PWML6_PC23
#define CH_PWM_CC PWML7_PC24

// Flip IMU module around X axis (bottom of module is up)
#define IMU_FLIP

// Rotate IMU module by 180 degrees
// #define IMU_ROTATE_180

// GY-80 module as IMU
// #define IMU_GY80

// MPU-9250 (MPU6500+AK8963 on signle chip) as IMU
#define IMU_MPU9250

// Use I2C passthrough mode to access AK8963, may not work properly (instead it'll use slower I2C master interface if not defined)
#define IMU_MPU6500_PASSTHROUGH

// FXOS8700+FXAS21002 board FRDM-STBC-AGM01
// #define IMU_FXOS8700
// #define IMU_FXAS21002

// Use madgwick filter for IMU data fusion (if disabled, it'll use complementary+kalman)
#define MADGWICK_FILTER 

// Use Kalman filter for IMU data
//#define KALMAN_FILTER

// Invert bumper's signal (LOW - obstacle)
#define BUMPERS_INVERT

// Number of bumper sensors
#define BUMPERS_NUM 4

// Bumper pins definitions
#define PIN_BUMP_LEFTFORW 51
#define PIN_BUMP_RIGHTFORW 50
#define PIN_BUMP_LEFTBACK 49
#define PIN_BUMP_RIGHTBACK 48

// Dual Polulu MC33926
#define MOT_DRIVER_DUAL_MC33926
// STM-Nucleo IHM12A1 driver on test car
// #define MOT_DRIVER_IHM12A1

#define PIN_MOT_LEFTFORW_FAULT 73
#define PIN_MOT_RIGHTFORW_FAULT 44
#define PIN_MOT_FORW_ENABLE 10
#define PIN_MOT_LEFTFORW_DIR 72
#define PIN_MOT_RIGHTFORW_DIR 45

// hundreds of microseconds per PWM tick for motors
// 50 * 255 -> 127.5 uS PWM period
#define MOT_PWM_TICK_WIDTH 50

// Maximum RPM of motors
#define MAXIMUM_RPM 20.0f

// Perimeter of wheel (in meters)
#define WHEEL_PERIMETER 0.798f

// Wheel base (in meters)
#define WHEEL_BASE 0.5f

// 2 dual motor drivers (separate channels for quad wheels)
#define MOT_QUAD_WHEELS
#define PIN_MOT_LEFTBACK_FAULT 46
#define PIN_MOT_RIGHTBACK_FAULT 52
#define PIN_MOT_BACK_ENABLE 109
#define PIN_MOT_LEFTBACK_DIR 47
#define PIN_MOT_RIGHTBACK_DIR 65

// Motors pwm channels
#define CH_MOT_LEFTFORW_PWM CH_PWM_A
#define CH_MOT_RIGHTFORW_PWM CH_PWM_B
#define CH_MOT_LEFTBACK_PWM CH_PWM_C
#define CH_MOT_RIGHTBACK_PWM CH_PWM_D

// MAX11617 ADC channels numbers for FB pins
#define CH_ADC_LEFTFORW 0
#define CH_ADC_RIGHTFORW 1
#define CH_ADC_LEFTBACK 2
#define CH_ADC_RIGHTBACK 3

// Feedback of MC33926 to current coefficient
#define K_ADC_CURRENT_MOTORS 0.0022

// Stepper motor driver DRV8825/A4988 pins and parameters
#define PIN_MOW_STEP 107
#define PIN_MOW_RESET 106
#define PIN_MOW_DIR 4
#define PIN_MOW_FAULT1 22
#define PIN_MOW_FAULT2 29
#define PIN_MOW_S_END 41

// Number of steps per mm of height
#define STEPS_PER_MM 50

// PWM output channel for mowing motor
#define CH_MOW_PPM CH_PWM_E

// Feedback from ACS725-30 to current ratio
#define K_ADC_CURRENT_ACS725_30 0.0175

// Current sensor of mowing motor (it uses reading from AD10 when defined, otherwise - C & D from MAX11617)
#define MOW_CURRENT_INT 12

// Current of mowing motor from C/D motor drivers
// #define MOW_CURRENT_EXT1 2
// #define MOW_CURRENT_EXT2 3

// PPM signal parameters for mowing motor (in hundreths of microsecond)
#define MOW_PWM_ZEROMIDDLE 150000
#define MOW_PWM_OFFSET 49000
#define MOW_PWM_PERIOD 250000

// Odometers (hall sensors)
#define _NUM_ODOMETERS_WHEELS 4
#define _NUM_ODOMETERS_MOW 1
#define PIN_ODO_A1 92
#define PIN_ODO_A2 93
#define PIN_ODO_B1 94
#define PIN_ODO_B2 108
#define PIN_ODO_C1 95
#define PIN_ODO_C2 96
#define PIN_ODO_D1 97
#define PIN_ODO_D2 98
#define PIN_ODO_X1 99
#define PIN_ODO_X2 100

// Test version, just two perimeter sensors
#define _PERIMETERS_NUM 2
// Positions of perimeters sensors
#define _PERIMETERS_POS _locationThings::LEFT_RIGHT

// Pin for perimeter and its adc channel
#define PIN_PERIMETER1 A4
#define PIN_PERIMETER2 A5
#define ADC_PERIMETER_CH1 3
#define ADC_PERIMETER_CH2 2
#define ADC_PERIMETER_CH3 10
#define ADC_PERIMETER_CH4 11

// Switches pins
#define PIN_BEEPER 38
#define PIN_BUTTON 101
#define PIN_BOARDLED 42
#define PIN_EXTLED1 35
#define PIN_EXTLED2 36
#define PIN_EXTLED3 37

// Sonars pins
#define PIN_SONAR_TRIG1 23
#define PIN_SONAR_ECHO1 3
#define PIN_SONAR_TRIG2 24
#define PIN_SONAR_ECHO2 53
#define PIN_SONAR_TRIG3 25
#define PIN_SONAR_ECHO3 105
#define PIN_SONAR_TRIG4 26
#define PIN_SONAR_ECHO4 78
#define PIN_SONAR_TRIG5 27
#define PIN_SONAR_ECHO5 103
#define PIN_SONAR_TRIG6 28
#define PIN_SONAR_ECHO6 2

#define SONARS_NUM 6

// Centimeters per microsecond
#define HCSR04_CM_PER_US 58

// Maximum distance for sonar (ignore bigger one), in centimeters
#define SONAR_MAX_DIST 1500

// MAX11617 I2C bus number, will be not read if not defined
#define BUS_MAX11617 1

// ADC channels on internal ADC
#define POW_CURRENT_TOTAL 1
#define POW_CURRENT_CHARGE 4
#define POW_VOLTAGE_SOLAR 0
#define POW_VOLTAGE_BATT 5
#define POW_VOLTAGE_BOOST 6
#define POW_VOLTAGE_CHARGE 7

// ADC channels on MAX11617
#define POW_VOLTAGE_3V3 8
#define POW_VOLTAGE_5V 7
#define POW_VOLTAGE_5VSEC 6

// ADC->voltage coefficient for internal ADC
#define K_ADC_VOLTAGE_INT 0.00802
// ADC->voltage coefficient for MAX11617
#define K_ADC_VOLTAGE_EXT 0.00833

// Fan pwm period (in hundreths of uS)
#define FAN_PWM_PERIOD 5000

// Boost and charge PWM period (in hundreths of uS)
#define CHARGE_PWM_PERIOD 200

// Shutdown relay pin
#define PIN_SHUTDOWN 32

// SCL pins definition (for force reset at init)
#define PIN_SCL 21
#define PIN_SDA 20
#define PIN_SCL1 71
#define PIN_SDA1 70

#endif
