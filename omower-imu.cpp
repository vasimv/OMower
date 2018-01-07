// IMU class for test smallcar, GY-80 module
// $Id$

#include <Arduino.h>
#include <omower-imu.h>
#include <due-i2c-blocking.h>
#include <omower-motors.h>
#include <omower-debug.h>

// -------------I2C addresses ------------------------
#define ADXL345B (0x53)          // ADXL345B acceleration sensor (GY-80 PCB)
#define HMC5883L (0x1E)          // HMC5883L compass sensor (GY-80 PCB)
#define L3G4200D (0xD2 >> 1)     // L3G4200D gyro sensor (GY-80 PCB)
#define BMP085 (0x77)

#define ADDR 600
#define MAGIC 6

// permanent calibration values
#define DEBUG_IMU_CONSTANT

// Print accelerometer/gyro data on each poll
// #define DEBUG_IMU_FULL

// Poll 50 times per second
void imu::poll50() {
  unsigned int loopTime;
  unsigned long now;
  float cosRoll, cosPitch, sinRoll, sinPitch;

  // Don't perform anything if we have a problem or disabled
  if (errorStatus != 0)
    return;

  debug(L_DEBUG, "imu::poll50\n");
  // Read data from sensors
  readBMP085();
  readL3G4200D();
  readADXL345B();
  readHMC5883L();

  // Calculate time from previous reading
  now = millis();
  loopTime = (now - lastTime);
  lastTime = now;

  // Calculate roll and pitch
  accPitch = atan2(-acc.x, sqrt(sq(acc.y) + sq(acc.z)));
  accRoll = atan2(acc.y, acc.z);
  accPitch = scalePIangles(accPitch, filtPitch);
  accRoll = scalePIangles(accRoll, filtRoll);
//  filtPitch = kalman(accPitch, gyro.x, loopTime, filtPitch);
  filtPitch = accPitch;
  filtPitch = scalePI(filtPitch);
//  filtRoll = kalman(accRoll, gyro.y, loopTime, filtRoll);
  filtRoll = accRoll;
  filtRoll = scalePI(filtRoll);
  cosRoll = cos(filtRoll);
  cosPitch = cos(filtPitch);
  sinRoll = sin(filtRoll);
  sinPitch = sin(filtPitch);

  // Calculate yaw
#if 0
  // Old ardumower code
  comTilt.x = com.x * cosPitch + com.z * sinPitch;
  comTilt.y = com.x * sinRoll * sinPitch + com.y * cosRoll
              - com.z * sinRoll * cosPitch;
  comTilt.z = -com.x * cosRoll * sinPitch + com.y * sinRoll
              + com.z * cosRoll * cosPitch;
  comYaw = scalePI(atan2(comTilt.y, comTilt.x));
  comYaw = scalePIangles(comYaw, filtYaw);
  filtYaw = complementary(comYaw, -gyro.z, loopTime, filtYaw);
  filtYaw = scalePI(filtYaw);
#endif

  if (comSelfTest == 0) {
    comTilt.x = com.x * cosPitch + com.y * sinRoll * sinPitch + com.z * cosRoll * sinPitch;
    comTilt.y = com.y * cosRoll - com.z * sinRoll;
    comYaw = scalePI(atan2(comTilt.y, comTilt.x));
    comYaw = scalePIangles(comYaw, filtYaw);
    filtYaw = complementary(comYaw, -gyro.z, loopTime, filtYaw);
    filtYaw = scalePI(filtYaw);
  }

  debug(L_DEBUG, (char *) F("imu: P %g, R %g, Y %g, T %g, e %d\n"),
        filtPitch, filtRoll, filtYaw, filtTemp, errorStatus);
#ifdef DEBUG_IMU_FULL
  debug(L_DEBUG, (char *) F("IF: %g %g %g  %g %g %g\n"),
	acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z);
#endif

  // End self test and calculate temperature compensation
  if (comSelfTest == 1) {
    comSelfTest = 0;
    selfHMC5883L(false);
    calcTempComp();
    compassNeedComp = false;
  }

  // Initiate real self-test after collecting raw compass data
  if (comSelfTest == 2) {
    comSelfTest = 1;
    selfHMC5883L(true);
    comRaw.x = com.x;
    comRaw.y = com.y;
    comRaw.z = com.z;
    comTempSelf = filtTemp;
  }

  // Check if temperature difference is high enough to make
  // self-test and calculate temperature compensation
  if ((comSelfTest == 0) && compassNeedComp && (abs(filtTemp - comTempInit) >= 5))
    comSelfTest = 2;
} // void imu::poll50()

// ADXL345B acceleration sensor init
void imu::initADXL345B() {
  debug(L_DEBUG, (char *) F("imu::initADXL345B\n"));
  i2cWriteOne(bus, ADXL345B, 0x2D, 0);
  i2cWriteOne(bus, ADXL345B, 0x2D, 16);
  i2cWriteOne(bus, ADXL345B, 0x2D, 8);
  // Full resolution, 16G
  i2cWriteOne(bus, ADXL345B, 0x31, 0x0B);
  // 100Hz data rate
  i2cWriteOne(bus, ADXL345B, 0x2C, 0x0A);
  // FIFO stream
  i2cWriteOne(bus, ADXL345B, 0x38, 0x9F);
  if (!accelCalibrated)
    calibAccel();
} // void imu::initADXL345B()

// ADXL345B acceleration sensor calibration
_status imu::calibAccel() {
  float sumX, sumY, sumZ;

  delay(250);
  for (uint16_t i = 0; i < 50; i++) {
    readADXL345B();
    sumX += acc.x;
    sumY += acc.y;
    sumZ += acc.z;
    delay(20);
  }
  accOfs.x = sumX / 50;
  accOfs.y = sumY / 50;
  // Suppose earth gravity is 1G
  accOfs.z = -(250 - sumZ / 50);
  // Well. not really 250 but around 
  accScale.x = 250.0;
  accScale.y = 250.0;
  accScale.z = 250.0;
  debug(L_DEBUG, (char *) F("imu::calibAccel: offsets %g %g %g, scale %g %g %g\n"), accOfs.x, accOfs.y, accOfs.z, accScale.x, accScale.y, accScale.z);
  accelCalibrated = true;
  return _status::NOERR;
} // void imu::calibAccel()

// Swap two bytes
void imu::swapBytes(uint8_t *p) {
  uint8_t tmp;

  tmp = *p;
  *p = *(p + 1);
  *(p + 1) = tmp;
} // void imu::swapBytes(uint8_t *p)

// read data from BMP085
void imu::readBMP085() {
  uint16_t ut;
  unsigned long up;
  uint16_t res;
  int32_t x1, x2, b5;
  int32_t x3, b3, b6, p;
  uint32_t b4, b7;

  if (readingTemperature) {
    // Read temperature
    res = i2cRead(bus, BMP085, 0xF6, (uint8_t *) &ut, 2);
    if (res != 2) {
      debug(L_NOTICE, (char *) F("imu::readBMP085: res %d\n"), res);
      addError(4);
    }
    swapBytes((uint8_t *) &ut);

    // Calculate temperature
    x1 = (((int32_t) ut - (int32_t) bmpCalib.ac6) * (int32_t) bmpCalib.ac5) >> 15;
    x2 = ((int32_t) bmpCalib.mc << 11) / (x1 + bmpCalib.md);
    filtTemp = ((x1 + x2 + 8) >> 4) / 10.0;

    // Request to measure pressure
    readingTemperature = false;
    i2cWriteOne(bus, BMP085, 0xF4, 0x34 + (2 << 6));
  } else {
    // Read pressure

    // Request temperature conversion
    readingTemperature = true;
    i2cWriteOne(bus, BMP085, 0xF4, 0x2E);
  }
} // void imu::readBMP085()

// BMP085 temperature/pressure sensor init
void imu::initBMP085() {
  uint8_t *p;
  uint16_t res;

  debug(L_DEBUG, (char *) F("imu::initBMP085\n"));

  // Read calibration values to the bmpCalib structure
  p = (uint8_t *) &bmpCalib;
  for (uint8_t addr = 0xAA; addr < 0xC0; addr += 2) {
    res = i2cRead(bus, BMP085, addr, p, 2);
    if (res != 2) {
      debug(L_NOTICE, (char *) F("imu::initBMP085: res %d\n"), res);
      addError(4);
    }
    p += 2;
  }

  // Swap all read bytes
  for (p = (uint8_t *) &bmpCalib; p < ((uint8_t *) &bmpCalib + sizeof(bmpCalib)); p += 2)
    swapBytes(p);

  // Request temperature conversion
  readingTemperature = true;
  i2cWriteOne(bus, BMP085, 0xF4, 0x2E);
} // void imu::initBMP085()

// Turn on/off self test on compass
void imu::selfHMC5883L(boolean test) {
  if (test)
    i2cWriteOne(bus, HMC5883L, 0x00, 0x19);
  else
    i2cWriteOne(bus, HMC5883L, 0x00, 0x18);
} // void imu::selfHMC5883L(boolean test)

// Calculate temperature compensation
void imu::calcTempComp() {
  float tempDiff = filtTemp - comTempInit;

  debug(L_NOTICE, (char *) F("imu::calcTempComp: x/y/z %g %g %g (%g %g %g)\n"), com.x, com.y, com.z, comRaw.x, comRaw.y, comRaw.z);
  comTempScale.x = (comInit.x / (com.x - comRaw.x) - 1) / tempDiff;
  comTempScale.y = (comInit.y / (com.y - comRaw.y) - 1) / tempDiff;
  comTempScale.z = (comInit.z / (com.z - comRaw.z) - 1) / tempDiff;
  debug(L_NOTICE, (char *) F("imu::calcTempComp: compensation x/y/z %g %g %g\n"), comTempScale.x, comTempScale.y, comTempScale.z);
} // void imu::calcTempComp()

// HMC5883L compass sensor init
void imu::initHMC5883L() {
  debug(L_DEBUG, (char *) F("imu::initHMC5883L\n"));
  // 8 samples average, 15Hz, no bias
  // i2cWriteOne(bus, HMC5883L, 0x00, 0x70);
  // no average, 75Hz, no bias
  i2cWriteOne(bus, HMC5883L, 0x00, 0x18);
    
  // gain
  // i2cWriteOne(bus, HMC5883L, 0x01, 0xA0);
  // gain
  i2cWriteOne(bus, HMC5883L, 0x01, 0x40);
  // mode
  i2cWriteOne(bus, HMC5883L, 0x02, 0);


  // Save self-test and temperature values at init
  delay(30);
  readHMC5883L();
  comRaw.x = com.x;
  comRaw.y = com.y;
  comRaw.z = com.z;
  comSelfTest = 1;
  selfHMC5883L(true);
  delay(30);
  readHMC5883L();
  readBMP085();
  comInit.x = com.x - comRaw.x;
  comInit.y = com.y - comRaw.y;
  comInit.z = com.z - comRaw.z;
  comTempInit = filtTemp;
  selfHMC5883L(false);
  comSelfTest = 0;
  compassNeedComp = true;

  // We don't know values for temperature scaling yet, so
  // assume zero compensation for current temperature
  comTempScale.x = comTempScale.y = comTempScale.z = 0; 
  delay(30);
  debug(L_DEBUG, (char *) F("imu::initHMC5883L: selft-test x/y/z %g %g %g, T: %g\n"), comInit.x, comInit.y, comInit.z, comTempInit);
} // void imu::initHMC5883L() {

// Read data from accelerometer
void imu::readADXL345B() {
  float x, y, z;
  uint8_t numFifo, buf[6];
  uint16_t res;
  int32_t sX, sY, sZ;

  // Get numbers of samples in the FIFO
  if (res = i2cRead(bus, ADXL345B, 0x39, (uint8_t *) buf, 1) != 1) {
    debug(L_NOTICE, (char *) F("imu::readADXL345B: res %d\n"), res);
    addError(2);
    return;
  }
  numFifo = buf[0];

  sX = sY = sZ = 0;
  // Read whole fifo buffer
  for (uint8_t i = 0; i < numFifo; i++) {
    if ((res = i2cRead(bus, ADXL345B, 0x32, (uint8_t *) buf, 6)) != 6) {
     debug(L_NOTICE, (char *) F("imu::readADXL345B: res %d\n"), res);
     addError(2);
     return;
    }
    sX += (int16_t) (((uint16_t) buf[1]) << 8 | buf[0]);
    sY += (int16_t) (((uint16_t) buf[3]) << 8 | buf[2]);
    sZ += (int16_t) (((uint16_t) buf[5]) << 8 | buf[4]);
  }
  // Convert accelerometer reading
  if (accelCalibrated){
    x = sX / numFifo - accOfs.x;
    y = sY / numFifo - accOfs.y;
    z = sZ / numFifo - accOfs.z;
    x /= accScale.x;
    y /= accScale.y;
    z /= accScale.z;
    acc.x = x;
    acc.y = y;
    acc.z = z;
  } else {
    acc.x = sX / numFifo;
    acc.y = sY / numFifo;
    acc.z = sZ / numFifo;
  }
} // void imu::readADXL345B()

// Read data from compass
void imu::readHMC5883L() {
  float x, y, z;
  uint8_t buf[6];
  uint16_t res;

  if ((res = i2cRead(bus, HMC5883L, 0x03, (uint8_t *) buf, 6)) != 6) {
    debug(L_NOTICE, "imu::readHMC5883L: res %d\n", res);
    addError(1);
    return;
  }
  x = (int16_t) (((uint16_t) buf[0]) << 8 | buf[1]);
  y = (int16_t) (((uint16_t) buf[4]) << 8 | buf[5]);
  z = (int16_t) (((uint16_t) buf[2]) << 8 | buf[3]);
  if ((comSelfTest == 0) && (filtTemp != comTempInit)) {
    x *= (1.0 - comTempScale.x * (filtTemp - comTempInit));
    y *= (1.0 - comTempScale.y * (filtTemp - comTempInit));
    z *= (1.0 - comTempScale.z * (filtTemp - comTempInit));
  }
  if (comSelfTest == 0) {
    if (x < comMin.x)
      comMin.x = x;
    if (x > comMax.x)
      comMax.x = x;
    if (y < comMin.y)
      comMin.y = y;
    if (y > comMax.y)
      comMax.y = y;
    if (z < comMin.z)
      comMin.z = z;
    if (z > comMax.z)
      comMax.z = z;
  }
  if (compassCalibrated && (comSelfTest == 0)) {
    x -= comOfs.x;
    y -= comOfs.y;
    z -= comOfs.z;
    x /= comScale.x * 0.5;
    y /= comScale.y * 0.5;
    z /= comScale.z * 0.5;
    com.x = x;
    com.y = y;
    com.z = z;
  } else {
    com.x = x;
    com.y = y;
    com.z = z;
  }
} // void imu::readHMC5883L()

// Read data from gyro
void imu::readL3G4200D() {
  uint8_t fifoReg = 0;
  uint8_t count;

  i2cRead(bus, L3G4200D, 0x2F, &fifoReg, sizeof(fifoReg));
  count = (fifoReg & 0x1F) + 1;

  memset(gyroFifo, 0, sizeof(gyroFifo[0]) * 32);
  i2cRead(bus, L3G4200D, 0xA8, (uint8_t *) gyroFifo, sizeof(gyroFifo[0]) * count);

  gyro.x = gyro.y = gyro.z = 0;
  if (!gyroCalibrated)
    count = 1;
  for (uint8_t i = 0; i < count; i++) {
      gyro.x += (int16_t) (((uint16_t) gyroFifo[i].xh) << 8 | gyroFifo[i].xl);
      gyro.y += (int16_t) (((uint16_t) gyroFifo[i].yh) << 8 | gyroFifo[i].yl);
      gyro.z += (int16_t) (((uint16_t) gyroFifo[i].zh) << 8 | gyroFifo[i].zl);
      if (gyroCalibrated) {
        gyro.x -= gyroOfs.x;
        gyro.y -= gyroOfs.y;
        gyro.z -= gyroOfs.z;
      }
  }
  if (gyroCalibrated) {
    gyro.x *= 0.07 * M_PI / 180.0;  // convert to radiant per second
    gyro.y *= 0.07 * M_PI / 180.0;
    gyro.z *= 0.07 * M_PI / 180.0;
  }
} // void imu::readL3G4200D()

// calculate gyro offsets
_status imu::calibGyro() {
  struct _vec_float_t ofs;

  debug(L_DEBUG, (char *) F("imu::calibGyro\n"));
  gyroCalibrated = false;
  gyroOfs.x = gyroOfs.y = gyroOfs.z = 0;
  while(true) {
    float zmin = 99999;
    float zmax = -99999;

    gyroNoise = 0;
    ofs.x = ofs.y = ofs.z = 0;
    for (uint8_t i = 0; i < 50; i++){
      readL3G4200D();
      zmin = min(zmin, gyro.z);
      zmax = max(zmax, gyro.z);
      ofs.x += gyro.x / 50.0;
      ofs.y += gyro.y / 50.0;
      ofs.z += gyro.z / 50.0;
      // calculate noise with last offset
      gyroNoise += sq(gyro.z - gyroOfs.z) / 50.0;
    }
    debug(L_INFO, (char *) F("imu::calibGyro: min %g, max %g, ofs.z %g, noise %g\n"),
          zmin, zmax, ofs.z, gyroNoise);
    // Found optimum
    if (gyroNoise < 20)
      break;
    // new offset
    gyroOfs = ofs;
  }
  debug(L_INFO, (char *) F("imu::calibGyro: ofs.x %g, ofs.y %g, ofs.z %g\n"), ofs.x, ofs.y, ofs.z);
  gyroCalibrated = true;
} // _status imu::calibGyro()

// L3G4200D gyro sensor init
boolean imu::initL3G4200D() {
  uint8_t buf[6];
  uint8_t retry = 0;

  debug(L_DEBUG, (char *) F("imu::initL3G4200D\n"));
  while (true) {
    i2cRead(bus, L3G4200D, 0x0F, (uint8_t *) buf, 1);
    if (buf[0] != 0xD3) {
      debug(L_ERROR, (char *) F("imu::initL3G4200D: gyro read error\n"));
      retry++;
      if (retry > 2){
        // set soft error state
        addError(3);
        errorStatus = 2;
        return false;
      }
      delay(1000);
    } else break;
  }

  // Normal power mode, all axes enabled, 100 Hz
  i2cWriteOne(bus, L3G4200D, 0x20, 0b00001100);
  // 2000 degree per second
  i2cWriteOne(bus, L3G4200D, 0x23, 0b00100000);
  i2cRead(bus, L3G4200D, 0x23, (uint8_t *) buf, 1);
  if (buf[0] != 0b00100000){
    // Set hard error status
    debug(L_ERROR, (char *) F("imu::initL3G4200D: gyro write error\n"));
    addError(3);
    errorStatus = 1;
  }

  if (!gyroCalibrated) {
    delay(250);
    calibGyro();
  }
  return true;
}

// Returns error status if there were errors or not calibrated
_hwstatus imu::softError() {
  if (!compassCalibrated || !gyroCalibrated || !baroCalibrated || !accelCalibrated)
    return _hwstatus::DISABLED;
  if (errorStatus)
    return _hwstatus::ERROR;
  return _hwstatus::ONLINE;
} // _hwstatus imu::softError()

// Read calibration from eeprom
_status readCalibration() {
  return _status::NOERR;
} // _status readCalibration()

// Hardware initialization (once)
_hwstatus imu::begin() {
  debug(L_DEBUG, (char *) F("imu::begin\n"));
  compassCalibrated = gyroCalibrated = baroCalibrated = accelCalibrated = false;
} // _hwstatus imu::begin()

// (re-)Initialization of IMU
_status imu::init(nvmem *saveMem) {
  debug(L_INFO, (char *) F("imu::init\n"));

  oSave = saveMem;
  if (oSave) {
    nvmemAddr = oSave->curAddr;
    loadCalib();
  }

  // Re-init i2c bus
  i2cInit(bus, 400000);

  // XXX
  // Don't have barometer calibration yet
  baroCalibrated = true;

  // Check if we need calibration
  if (readCalibration() != _status::NOERR) {
    errorStatus = 3;
  }

  // Clear tables
  memset((uint8_t *) &com, 0, sizeof(com));
  memset((uint8_t *) &gyro, 0, sizeof(gyro));
  memset((uint8_t *) &acc, 0, sizeof(acc));

  // Initialize IMU
  if (!initL3G4200D())
    return _status::ERR;
  initBMP085();
  initADXL345B();
  initHMC5883L();

  if (!compassCalibrated) {
#ifdef DEBUG_IMU_CONSTANT
    // permanent debug calibration values
    comOfs.x = -779;
    comOfs.y = -320;
    comOfs.z = 1205.5;
    comScale.x = 669.00;
    comScale.y = 884.00;
    comScale.z = 139.00;
    compassCalibrated = true;
  }
#endif
  startCompassCalib();

  return _status::NOERR;
} // _status imu::init(nvmem *saveMem)

// Complementary filter
float imu::complementary(float newAngle, float newRate, int loopTime, float angle) {
  float k = 5;
  float dtc2 = float(loopTime) / 1000.0;
  float x1 = (newAngle - angle) * k * k;
  float y1 = dtc2 * x1;
  float x2 = y1 + (newAngle - angle) * 2 * k + newRate;

  angle = dtc2 * x2 + angle;
  return angle;
} // float imu::complementary(float newAngle, float newRate, int loopTime, float angle)

// Kalman filter
float imu::kalman(float newAngle, float newRate, int loopTime, float x_angle) {
  float Q_angle =  0.01;
  float Q_gyro = 0.0003;
  float R_angle = 0.01;

  float x_bias = 0;
  float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
  float y, S;
  float K_0, K_1;

  float dt = float(loopTime)/1000;
  x_angle += dt * (newRate - x_bias);
  P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
  P_01 += -dt * P_11;
  P_10 += -dt * P_11;
  P_11 += Q_gyro * dt;

  y = newAngle - x_angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  x_angle += K_0 * y;
  x_bias += K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return x_angle;
} // float imu::Kalman(float newAngle, float newRate, int loopTime, float x_angle)

// scale to -M_PI .. M_PI range
float imu::scalePI(float v) {
  float d = v;

  while (d < 0)
    d += 2 * M_PI;
  while (d >= 2 * M_PI)
    d -= 2 * M_PI;
  if (d >= M_PI)
    return (-2 * M_PI + d);
  else {
    if (d < -M_PI)
      return (2 * M_PI +d);
    else
      return d;
  }
} // static float imu::scalePI(float v)

// scale setAngle to same sign as currAngle
float imu::scalePIangles(float setAngle, float currAngle) {
  if ((setAngle >= (M_PI / 2)) && (currAngle <= (-M_PI / 2)))
    return (setAngle - 2 * M_PI);
  else {
    if ((setAngle <= (-M_PI / 2)) && (currAngle >= (M_PI / 2)))
      return (setAngle + 2 * M_PI);
    else
      return setAngle;
  }
} // static float imu::scalePIangles(float setAngle, float currAngle)

// Convert degrees to -M_PI..M_PI range
float imu::degreePI(int16_t angleDegree) {
  return ((float) angleDegree * M_PI) / 180.0;
} // static float imu::degreePI(float angleDegree)

// Convert -M_PI..M_PI range to degrees
int16_t imu::piDegree(float anglePI) {
  return (anglePI * 180.0) / M_PI;
} // static uint16_t imu::piDegree(float anglePI)

// Set course
void imu::setCourse(int16_t degree, boolean stopWhenReached) {
  courseCur = degreePI(degree);
  stopFinished = stopWhenReached;
} // void imu::setCourse(int16_t degree)

// Error for PID controller
float imu::readCourseError() {
  float d;

  d = scalePI(courseCur - degreePI(readCurDegree(0)));

  // Range check
  if (d < -M_PI)
    d = d + 2 * M_PI;
  else
    if (d > M_PI)
      d = d - 2 * M_PI;

  debug(L_NOTICE, (char *) F("imu::readCourseError: filtYaw %g, inclMag %g, courseCur %g, d %g\n"),
        filtYaw, inclMag, courseCur, d);
  // Check if we've reached needed course
  if (stopFinished && (abs(d) < (M_PI / 20)))
    return -1000;

  return d;
} // float imu::readCourseError()

// Current course
int16_t imu::readCurDegree(numThing n) {
  return piDegree(scalePI(filtYaw + ((inclMag / 180.0) * M_PI)));
} // int16_t imu::readCurDegree(numThing n)

// Current pitch degree
int16_t imu::readCurPitchDegree(numThing n) {
  return piDegree(filtPitch);
} // int16_t imu::readCurPitchDegree(numThing n)

// Current roll degree
int16_t imu::readCurRollDegree(numThing n) {
  return piDegree(filtRoll);
} // int16_t imu::readCurRollDegree(numThing n)

// Current temperature
int16_t imu::readCurTemperature(numThing n) {
  return (int16_t) filtTemp;
} // int16_t imu::readCurTemperature(numThing n)

int16_t imu::scaleDegree(int16_t angleDegree) {
  // -180 .. 180
  if ((angleDegree > -180) && (angleDegree < 180))
    return angleDegree;
  // 180..360
  if (angleDegree > 180)
    return angleDegree - 360;
  // -180..-360
  return angleDegree + 360;
} // int16_t imu::scaleDegree(int16_t angleDegree)

// Start compass calibration
_status imu::startCompassCalib() {
  comMin.x = comMin.y = comMin.z = 999999;
  comMax.x = comMax.y = comMax.z = -999999;
  return _status::NOERR;
} // _status imu::startCompassCalib()

_status imu::stopCompassCalib() {
  float xrange = comMax.x - comMin.x;
  float yrange = comMax.y - comMin.y;
  float zrange = comMax.z - comMin.z;

  comOfs.x = xrange / 2 + comMin.x;
  comOfs.y = yrange / 2 + comMin.y;
  comOfs.z = zrange / 2 + comMin.z;
  comScale.x = xrange;
  comScale.y = yrange;
  comScale.z = zrange;
  debug(L_NOTICE, (char *) F("imu::stopCompassCalib: scale %f %f %f offs %f %f %f\n"),
        comScale.x, comScale.y, comScale.z, comOfs.x, comOfs.y, comOfs.z);
  compassCalibrated = true;
  return _status::NOERR;
} // _status imu::stopCompassCalib()

// Save calibration values in NVMEM
_status imu::saveCalib() {
  uint8_t i;

  if (oSave) {
    oSave->curAddr = nvmemAddr;
    if (compassCalibrated) {
      oSave->writeMem(comOfs.x);
      oSave->writeMem(comOfs.y);
      oSave->writeMem(comOfs.z);
      oSave->writeMem(comScale.x);
      oSave->writeMem(comScale.y);
      oSave->writeMem(comScale.z);
    } else
      // No calibration values, write fake
      for (i = 0; i < 6; i++)
        oSave->writeMem(-9999.0f);
    if (!compassNeedComp) {
      oSave->writeMem(comTempScale.x);
      oSave->writeMem(comTempScale.y);
      oSave->writeMem(comTempScale.z);
    } else
      for (i = 0; i < 3; i ++)
        oSave->writeMem(-9999.0f);
    if (accelCalibrated) {
      oSave->writeMem(accOfs.x);
      oSave->writeMem(accOfs.y);
      oSave->writeMem(accOfs.z);
      oSave->writeMem(accScale.x);
      oSave->writeMem(accScale.y);
      oSave->writeMem(accScale.z);
    } else
      // No calibration values, write fake
      for (i = 0; i < 6; i++)
        oSave->writeMem(-9999.0f);
  }
} // _status imu::saveCalib()

// Load calibration values from NVMEM
_status imu::loadCalib() {
  float tmpX, tmpY, tmpZ;

  if (oSave) {
    oSave->curAddr = nvmemAddr;
    oSave->readMem(tmpX);
    oSave->readMem(tmpY);
    oSave->readMem(tmpZ);
    // Check if we have real calibration data
    if (oSave->haveValid && (tmpX != -9999.0)) {
      comOfs.x = tmpX;
      comOfs.y = tmpY;
      comOfs.z = tmpZ;
    }
    oSave->readMem(tmpX);
    oSave->readMem(tmpY);
    oSave->readMem(tmpZ);
    // Check if we have real calibration data
    if (oSave->haveValid && (tmpX != -9999.0)) {
      comScale.x = tmpX;
      comScale.y = tmpY;
      comScale.z = tmpZ;
      compassCalibrated = true;
    }
    oSave->readMem(tmpX);
    oSave->readMem(tmpY);
    oSave->readMem(tmpZ);
    // Check if we have real calibration data
    if (oSave->haveValid && (tmpX != -9999.0)) {
      comTempScale.x = tmpX;
      comTempScale.y = tmpY;
      comTempScale.z = tmpZ;
      compassNeedComp = false;
    }
    oSave->readMem(tmpX);
    oSave->readMem(tmpY);
    oSave->readMem(tmpZ);
    // Check if we have real calibration data
    if (oSave->haveValid && (tmpX != -9999.0)) {
      accOfs.x = tmpX;
      accOfs.y = tmpY;
      accOfs.z = tmpZ;
    }
    oSave->readMem(tmpX);
    oSave->readMem(tmpY);
    oSave->readMem(tmpZ);
    // Check if we have real calibration data
    if (oSave->haveValid && (tmpX != -9999.0)) {
      accScale.x = tmpX;
      accScale.y = tmpY;
      accScale.z = tmpZ;
      accelCalibrated = true;
    }
  }
} // _status imu::loadCalib()
