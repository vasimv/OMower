// IMU class for test smallcar
// $Id$

#ifndef OMOWER_IMU_H
#define OMOWER_IMU_H

#include <omower-root.h>
#include <omower-defs.h>
#include <omower-nvmem.h>
#include <Arduino.h>
#include <math.h>

#ifdef KALMAN_FILTER
#include <Kalman.h>
#endif

// IMU sensor (compass, accelerometer, gyro, baro)
class imu : public navThing {
public:
  // Hardware initialization (once)
  _hwstatus begin();

  // (re)Initialization of the imu (reads calibration values from saveMem object if not NULL)
  _status init(nvmem *saveMem);

  // I2C bus object
  uint8_t bus;

  // Settings variables:
  // Maximum degree by pitch/roll for softError()
  int16_t maxDegree;

  // Magnetic inclination in the region, in degrees
  // (magnetic north pole offset from the geographic one)
  float inclMag;

  // Current course (with magnetic inclination) in radians
  float readCurDegreeRad(numThing n);

  // Current course (with magnetic inclination) in degress
  int16_t readCurDegree(numThing n);

  // Current pitch degree
  int16_t readCurPitchDegree(numThing n);

  // Current roll degree
  int16_t readCurRollDegree(numThing n);

  // Current temperature (in celsius)
  int16_t readCurTemperature(numThing n);

  // Current pressure, not working yet
  int16_t readCurPressure(numThing n);

  // Compass calibration start (the robot must be rotated by all axis slowly)
  _status startCompassCalib();
  // Compass calibration stop
  _status stopCompassCalib();
  // Compass self-test to compensate temperature drift
  // (must be called when robot is not moving!)
  _status performCompassSelfTest();

  // Accelerometer calibration (robot must stay on flat surface)
  _status calibAccel();

  // Gyro calibration (robot must stay on flat surface)
  _status calibGyro();

  // Save calibration values to NVMEM
  _status saveCalib();

  // Load calibration values from NVMEM (gets called at init automatically)
  _status loadCalib();

  // Sets movement/roll course by compass
  void setCourse(int16_t degree, boolean stopWhenReached = false);
  // Error for PID controller
  float readCourseError();

  // returns soft error status if too high degree or calibration missed
  _hwstatus softError();

  void poll50();

  // Static helpers stuff
  // Convert degrees to -M_PI..M_PI range
  static float degreePI(int16_t angleDegree);
  // Convert -M_PI..M_PI range to degrees
  static int16_t piDegree(float anglePI);
  // scale setAngle to same sign as currAngle
  static float scalePIangles(float setAngle, float currAngle);
  // scale to -M_PI .. M_PI range
  static float scalePI(float v);
  // scale to -180..180 degree range
  static int16_t scaleDegree(int16_t angleDegree);

private:
  // Calibration values address in NVMEM
  nvmem *oSave;
  uint32_t nvmemAddr;

  // Calibration statuses
  volatile boolean gyroCalibrated;
  volatile boolean accelCalibrated;
  volatile boolean compassCalibrated;
  volatile boolean baroCalibrated;
  volatile boolean imuLocked;

  // Compass need temperature compensation
  boolean compassNeedComp;
  uint8_t comSelfTest;

  // Issue stop command when reached set course
  volatile boolean stopFinished;

  // float type vector
  struct _vec_float_t {
    float x;
    float y;
    float z;
  };

#ifdef IMU_GY80
  // gyro read buffer
  struct {
    uint8_t xl;
    uint8_t xh;
    uint8_t yl;
    uint8_t yh;
    uint8_t zl;
    uint8_t zh;
  } gyroFifo[4];
#endif

  // Gyro offsets
  volatile struct _vec_float_t gyroOfs;

  // Compass offset
  volatile struct _vec_float_t comOfs;

  // Compass scale
  volatile struct _vec_float_t comScale;

  // Compass min/max for calibration
  volatile struct _vec_float_t comMax;
  volatile struct _vec_float_t comMin;

  // Compass self-test values at init
  volatile struct _vec_float_t comInit;
  volatile float comTempInit;

  // Compass last self-test values
  volatile struct _vec_float_t comRaw;
  volatile float comTempSelf;

  // Current compass temperature offset
  // (calculated based on self-test values)
  volatile struct _vec_float_t comTempScale;

  // Compass tilt
  struct _vec_float_t comTilt;

  volatile struct _vec_float_t acc;

  // Accelerometer scale
  volatile struct _vec_float_t accScale;

  // Accelerometer offset
  volatile struct _vec_float_t accOfs;

  // Sensors data
  volatile struct _vec_float_t com;
  volatile struct _vec_float_t gyro;

  // Calculated values in -M_PI..M_PI range
  volatile float accPitch, accRoll, comYaw, filtPitch, filtRoll, filtYaw, gyroNoise, filtTemp, filtPress;

  // Course set
  volatile float courseCur;

  // Last reading time
  unsigned long lastTime;

  // Error status 0 - no errors, 1 - hardware error, 2 - soft error
  volatile uint8_t errorStatus;

#ifdef KALMAN_FILTER
  Kalman kalmanX, kalmanY;
#endif

#ifdef MADGWICK_FILTER
  // Filter stuff
  volatile float q0, q0Prev;
  volatile float q1, q1Prev;
  volatile float q2, q2Prev;
  volatile float q3, q3Prev;
  volatile float invSampleFreq;

  float invSqrt(float x);

  // Init madgwick filter
  void madgwickInit();

  // Update madgwick filter
  void madgwickUpdate();
#endif

  // Complementary filter
  float complementary(float newAngle, float newRate, float looptime, float angle);

#ifdef IMU_GY80
  // Baro/temperature sensor calibration values
  struct {
    int16_t ac1, ac2, ac3;
    uint16_t ac4, ac5, ac6;
    int16_t b1, b2;
    int16_t mb, mc, md;
  } bmpCalib;

  // Reading temperature (instead of pressure) at next interrupt
  boolean readingTemperature;

  // L3G4200D gyro sensor init
  boolean initL3G4200D();

  // Read data from gyro
  void readL3G4200D();

  // HMC5883L compass sensor init 
  void initHMC5883L();

  // Read data from compass
  void readHMC5883L();

  // Turn on/off compass self test
  void selfHMC5883L(boolean test);

  // Calculate temperature compensation for compass
  void calcTempComp();

  // ADXL345B acceleration sensor init
  void initADXL345B();

  // Read data from accelerometer
  void readADXL345B();

  // BMP085 pressure/temperature sensor init
  void initBMP085();
  
  // Read data from BMP085
  void readBMP085();

  // Calibrate accel of GY80
  _status calibAccelGY80();

  // Calibrate gyro of GY80
  _status calibGyroGY80();
#endif // IMU_GY80

#ifdef IMU_MPU9250
  // Check if MPU6500 and AK8963 working
  boolean checkMPU9250();

  // Accel&gyro sensor MPU6500 init
  void initMPU6500();

  // send data to AK8693 through MPU9250
  void sendAK8963(uint8_t reg, uint8_t val);

  // receive data from AK8693 through MPU9250
  uint8_t recvAK8963(uint8_t reg, uint8_t *buf, uint8_t size);

  // Mag sensor AK8963 init
  void initAK8963();

  // Read from gyro&accel sensor MPU9250 (true if new data arrived)
  boolean readMPU6500();
  
  // Read from magnetometer sensor AK8963
  void readAK8963();

  // Calibration stuff for MPU6500
  void calibMPU6500();
#endif // IMU_MPU9250


#ifdef IMU_FXOS8700
  // Check and init FXOS8700
  boolean initFXOS8700();

  // Read accelerometer and magnetometer sensors
  boolean readFXOS8700();

  // Restart auto calibration of FXOS8700 magnetometer (it has internal max/min registers)
  void restartAutoCalibFXOS8700();

  // read MIN/MAX/OFFSET data from FXOS8700 magnetometer
  void readAutoCalibFXOS8700();

  // Calibrate accelerometer
  void calibFXOS8700();
#endif
#ifdef IMU_FXAS21002
  // Check and init FXAS21002
  boolean initFXAS21002();

  // Read gyroscope sensor
  boolean readFXAS21002();

  // Calibrate gyroscope
  void calibFXAS21002();
#endif

  // Swap two bytes
  void swapBytes(uint8_t *p);
};

#endif
