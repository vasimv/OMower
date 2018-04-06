// IMU class for OMower, GY-80 and MPU9250 (MPU6500+AK8963)
// $Id$

#include <Arduino.h>
#include <omower-imu.h>
#include <due-i2c-blocking.h>
#include <omower-motors.h>
#include <omower-debug.h>

#ifdef IMU_GY80
// -------------I2C addresses ------------------------
#define ADXL345B (0x53)          // ADXL345B acceleration sensor (GY-80 PCB)
#define HMC5883L (0x1E)          // HMC5883L compass sensor (GY-80 PCB)
#define L3G4200D (0xD2 >> 1)     // L3G4200D gyro sensor (GY-80 PCB)
#define BMP085 (0x77)
#endif

#ifdef IMU_MPU9250
#define MPU_ADDR 0x68
#define AK_ADDR 0x0C
#define MPU9250_ID 0x71
#define AK8963_ID 0x48

#define MPU_ARES 16384.0
#define MPU_GRES 250.0

// MPU9250 (MPU6500+AK8963) registers addresses
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_CONFIG2 0x1D
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_CONFIG 0x1A
#define MPU_FIFO_COUNTH 0x72
#define MPU_FIFO_EN 0x23
#define MPU_FIFO_R_W 0x74
#define MPU_GYRO_CONFIG 0x1B
#define MPU_I2C_MST_CTRL 0x24
#define MPU_INT_ENABLE 0x38
#define MPU_INT_PIN_CFG 0x37
#define MPU_INT_STATUS 0x3A
#define MPU_PWR_MGMT_1 0x6B
#define MPU_PWR_MGMT_2 0x6C
#define MPU_SMPLRT_DIV 0x19
#define MPU_USER_CTRL 0x6A
#define MPU_WHO_AM_I 0x75
#define MPU_XA_OFFSET_H 0x77
#define MPU_XA_OFFSET_L 0x78
#define MPU_XG_OFFSET_H 0x13
#define MPU_XG_OFFSET_L 0x14
#define MPU_YA_OFFSET_H 0x7A
#define MPU_YA_OFFSET_L 0x7B
#define MPU_YG_OFFSET_H 0x15
#define MPU_YG_OFFSET_L 0x16
#define MPU_ZA_OFFSET_H 0x7D
#define MPU_ZA_OFFSET_L 0x7E
#define MPU_ZG_OFFSET_H 0x17
#define MPU_ZG_OFFSET_L 0x18
#define MPU_EXT_SENS_DATA_00 0x49
#define MPU_I2C_SLV0_D0 0x63
#define MPU_I2C_SLV0_ADDR 0x25
#define MPU_I2C_SLV0_REG 0x26
#define MPU_I2C_SLV0_CTRL 0x27
#define AK_WHO_AM_I 0x00
#define AK_AK8963_ASAX 0x10
#define AK_AK8963_CNTL1 0x0A
#define AK_AK8963_CNTL2 0x0B
#define AK_AK8963_ST1 0x02
#define AK_AK8963_XOUT_L 0x03
// Constants
#define MPU_CLOCK_SEL_PLL 0x01
#define MPU_I2C_MST_EN 0x20
#define MPU_I2C_MST_CLK 0x0D
#endif

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
  if (errorStatus != 0 || imuLocked)
    return;

  debug(L_DEBUG, "imu::poll50\n");
  // Read data from sensors
  #ifdef IMU_GY80
  readBMP085();
  readL3G4200D();
  readADXL345B();
  readHMC5883L();
  #endif
  #ifdef IMU_MPU9250
  readAK8963();
  // Skip calculation if no new data from IMU
  if (!readMPU6500())
    return;
  #endif


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

  debug(L_DEBUG, (char *) F("imu: P %g, R %g, Y %g, T %g, MX: %g, MY: %g e %d\n"),
        filtPitch, filtRoll, filtYaw, filtTemp, com.x, com.y, errorStatus);
#ifdef DEBUG_IMU_FULL
  debug(L_DEBUG, (char *) F("IF: %g %g %g  %g %g %g\n"),
	acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z);
#endif

#ifdef IMU_GY80
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
#endif
} // void imu::poll50()

// Calibrate accelerometer
_status imu::calibAccel() {
#ifdef IMU_GY80
  return calibAccelGY80();
#endif
#ifdef IMU_MPU9250
  boolean checked;

  imuLocked = true;
  checked = checkMPU9250();
  calibMPU6500();
  initMPU6500();
  imuLocked = false;
  if (!checked)
    return _status::ERR;
  return _status::NOERR;
#endif
} // _status imu::calibAccel() 

// Calibrate gyroscope
_status imu::calibGyro() {
#ifdef IMU_GY80
  return calibGyroGY80();
#endif
#ifdef IMU_MPU9250
  return _status::NOERR;
#endif
} // _status imu::calibGyro()

#ifdef IMU_MPU9250
// Check MPU9250 is responding
boolean imu::checkMPU9250() {
  uint8_t tmp = 0;
  uint8_t res;

  // Reset FIFO and I2C modules
  i2cWriteOne(bus,MPU_ADDR, MPU_USER_CTRL, 0x06);
  delay(10);
  i2cRead(bus, MPU_ADDR, MPU_WHO_AM_I, &tmp, 1);
  if (tmp != MPU9250_ID) {
    debug(L_WARNING, (char *) F("Bad id from MPU9250/MPU6500 (%hd)\n"), tmp);
    addError(2);
    return false;
  }
  // Set clock and i2c master interface on MPU9250
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_1, 0);
  delay(100);
  i2cWriteOne(bus, MPU_ADDR, MPU_CONFIG, 0x01);
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_1, MPU_CLOCK_SEL_PLL);
  i2cWriteOne(bus, MPU_ADDR, MPU_USER_CTRL, MPU_I2C_MST_EN);
  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_MST_CTRL, MPU_I2C_MST_CLK);
  sendAK8963(AK_AK8963_CNTL1, 0);
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_1, 0x80);
  delay(100);
  sendAK8963(AK_AK8963_CNTL2, 1);
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_1, MPU_CLOCK_SEL_PLL);
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_2, 0);
  i2cWriteOne(bus, MPU_ADDR, MPU_USER_CTRL, MPU_I2C_MST_EN);
  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_MST_CTRL, MPU_I2C_MST_CLK);
  res = recvAK8963(AK_WHO_AM_I, &tmp, 1);
  if (tmp != AK8963_ID) {
    debug(L_WARNING, (char *) F("Bad id from MPU9250/AK8963 (%hd/%hd)\n"), tmp, res);
    addError(1);
    return false;
  }
  return true;
} // boolean imu::checkMPU9250()

// MPU6500 init
void imu::initMPU6500() {
  uint8_t c;

  // wake up device
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(50); // Wait for all registers to reset 

  // Reset
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_1, 0x80);
  delay(50); // Wait for all registers to reset 

  // get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_1, MPU_CLOCK_SEL_PLL);
  delay(50); 
  
  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  i2cWriteOne(bus, MPU_ADDR, MPU_CONFIG, 0x03);  

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV), 42Hz
  i2cWriteOne(bus, MPU_ADDR, MPU_SMPLRT_DIV, 0x17);
 
  // Set gyroscope full scale range (250DPS)
  // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  i2cRead(bus, MPU_ADDR, MPU_GYRO_CONFIG, &c, 1);
  // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x03; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear GFS bits [4:3]
  i2cWriteOne(bus, MPU_ADDR, MPU_GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register
  
  // Set accelerometer full-scale range configuration (2G)
  i2cRead(bus, MPU_ADDR, MPU_ACCEL_CONFIG, &c, 1); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  i2cWriteOne(bus, MPU_ADDR, MPU_ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  i2cRead(bus, MPU_ADDR, MPU_ACCEL_CONFIG2, &c, 1); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  i2cWriteOne(bus, MPU_ADDR, MPU_ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

  // Enable all sensors
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_2, 0x00);

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts enable
  // Not really needed on OMower (as it doesn't have connection for INT pin)
  i2cWriteOne(bus, MPU_ADDR, MPU_INT_PIN_CFG, 0x20);    
  i2cWriteOne(bus, MPU_ADDR, MPU_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

  // Configure I2C master interface to work with AK8963 (400KHz)
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_1, MPU_CLOCK_SEL_PLL);
  i2cWriteOne(bus, MPU_ADDR, MPU_USER_CTRL, MPU_I2C_MST_EN);
  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_MST_CTRL, MPU_I2C_MST_CLK);
  delay(50);
} // void imu::initMPU6500()

// Write to AK8963 register through MPU6500
void imu::sendAK8963(uint8_t reg, uint8_t val) {
  // i2cWriteOne(bus, MPU_ADDR, MPU_I2C_SLV0_CTRL, 0);
  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_SLV0_D0, val);
  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_SLV0_ADDR, AK_ADDR);
  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_SLV0_REG, reg);
  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_SLV0_CTRL, 0x81);
  delayMicroseconds(100);
//  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_SLV0_CTRL, 0x0);
} // void imu::sendAK8963(uint8_t reg, uint8_t val)

// Read from AK8963 registers through MPU6500
uint8_t imu::recvAK8963(uint8_t reg, uint8_t *buf, uint8_t size) {
  uint8_t tmp;

  // i2cWriteOne(bus, MPU_ADDR, MPU_I2C_SLV0_CTRL, 0);
  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_SLV0_ADDR, AK_ADDR | 0x80);
  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_SLV0_REG, reg);
  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_SLV0_CTRL, 0x80 | size);
  // Wait until all data read
  delayMicroseconds(100 + size * 25);
  tmp = i2cRead(bus, MPU_ADDR, MPU_EXT_SENS_DATA_00, buf, size); 
//  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_SLV0_CTRL, 0x0);
  return tmp;
} // uint8_t imu::recvAK8963(uint8_t reg, uint8_t *buf, uint8_t size)

// AK8963 Magnetometer init
void imu::initAK8963() {
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here

  // Reset AK8963
  sendAK8963(AK_AK8963_CNTL1, 0x00); // Power down magnetometer  
  delay(50);
  sendAK8963(AK_AK8963_CNTL2, 0x01);
  delay(50);
  sendAK8963(AK_AK8963_CNTL1, 0x00); // Power down magnetometer  
  delay(10);
  sendAK8963(AK_AK8963_CNTL1, 0x0F); // Enter Fuse ROM access mode
  delay(50);
  recvAK8963(AK_AK8963_ASAX, rawData, 3);  // Read the x-, y-, and z-axis calibration values
  comInit.x = (float)(rawData[0] - 128)/256.0 + 1.0;   // Return x-axis sensitivity adjustment values, etc.
  comInit.y = (float)(rawData[1] - 128)/256.0 + 1.0;  
  comInit.z = (float)(rawData[2] - 128)/256.0 + 1.0; 
  debug(L_NOTICE, (char *) F("imu::initAK8963: FUSE ROM: %g %g %g\n"),
        comInit.x, comInit.y, comInit.z);

  sendAK8963(AK_AK8963_CNTL1, 0x00); // Power down magnetometer  
  delay(10);

  // Configure the magnetometer for continuous read
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  // 16 bit, 100 Hz
  sendAK8963(AK_AK8963_CNTL1, 0x16); // Set magnetometer data resolution and sample ODR

  delay(10);
} // void imu::initAK8963()

// Read from gyro&accel of MPU6500
boolean imu::readMPU6500() {
  uint8_t tmp = 0;
  int16_t data[7];

  acc.x = acc.y = acc.z = gyro.x = gyro.y = gyro.z = 0.0;

  // Read INT status
  if (i2cRead(bus, MPU_ADDR, MPU_INT_STATUS, &tmp, 1) != 1) {
    debug(L_NOTICE, (char *) F("Couldn't read from MPU6500!\n"));
    addError(2);
    return false;
  }
  // Check if there is new data (42Hz data rate)
  if (tmp & 0x01) {
    // Read accel, temp and gyro data
    tmp = i2cRead(bus, MPU_ADDR, MPU_ACCEL_XOUT_H, (uint8_t *) data, 14);
    if (tmp != 14) { 
      debug(L_NOTICE, (char *) F("Couldn't read MPU6500 data (%hd)\n"), tmp);
      addError(2);
      return false;
    }

    // Swap bytes in data array
    for (int i = 0; i < (sizeof(data) / 2); i++)
      swapBytes((uint8_t *) &data[i]);

    // Calculate acceleration (in G), gyro (radiants/s), temperature (celsius)
    acc.x = -(float) data[0] / MPU_ARES;
    acc.y = -(float) data[1] / MPU_ARES;
    acc.z = (float) data[2] / MPU_ARES;
    gyro.x = ((float) data[4] * (MPU_GRES * M_PI)) / (32768.0 * 180.0);
    gyro.y = ((float) data[4] * (MPU_GRES * M_PI)) / (32768.0 * 180.0);
    gyro.z = ((float) data[4] * (MPU_GRES * M_PI)) / (32768.0 * 180.0);
    filtTemp = (float) data[3] / 333.87 + 21.0;
    return true;
  } else
    return false;
} // boolean imu::readMPU6500()

// Read from magnetometer of AK8963
void imu::readAK8963() {
  uint8_t res;
  uint8_t data[8];
  float x, y, z;

  res = recvAK8963(AK_AK8963_ST1, data, 8);
  if (res != 8) {
    debug(L_DEBUG, (char *) F("No data read from AK8963 (%hd)!\n"), res);
    return;
  }
  if ((data[0] & 0x01) && !(data[7] & 0x08)) {
      y= -(int16_t) (((int16_t) data[2] << 8) | data[1]);
      x= -(int16_t) (((int16_t) data[4] << 8) | data[3]);
      z= -(int16_t) (((int16_t) data[6] << 8) | data[5]);
  } else {
    debug(L_DEBUG, (char *) F("No data read from AK8963!\n"));
    return;
  }

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
  if (compassCalibrated) {
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
} // void imu::readAK8963()

// Calibrate gyro&accel of MPU6500
void imu::calibMPU6500() {
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
   
  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
  // else use the internal oscillator, bits 2:0 = 001
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_1, 0x01);  
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_2, 0x00);
  delay(200);                                    

  // Configure device for bias calculation
  i2cWriteOne(bus, MPU_ADDR, MPU_INT_ENABLE, 0x00);   // Disable all interrupts
  i2cWriteOne(bus, MPU_ADDR, MPU_FIFO_EN, 0x00);      // Disable FIFO
  i2cWriteOne(bus, MPU_ADDR, MPU_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  i2cWriteOne(bus, MPU_ADDR, MPU_I2C_MST_CTRL, 0x00); // Disable I2C master
  i2cWriteOne(bus, MPU_ADDR, MPU_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  i2cWriteOne(bus, MPU_ADDR, MPU_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
  // Configure MPU6050 gyro and accelerometer for bias calculation
  i2cWriteOne(bus, MPU_ADDR, MPU_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  i2cWriteOne(bus, MPU_ADDR, MPU_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  i2cWriteOne(bus, MPU_ADDR, MPU_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second
  i2cWriteOne(bus, MPU_ADDR, MPU_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  i2cWriteOne(bus, MPU_ADDR, MPU_USER_CTRL, 0x40);   // Enable FIFO  
  i2cWriteOne(bus, MPU_ADDR, MPU_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO
  i2cWriteOne(bus, MPU_ADDR, MPU_USER_CTRL, 0x44);   // Reset FIFO
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes (512 bytes max FIFO size)

  // At end of sample accumulation, turn off FIFO sensor read
  i2cWriteOne(bus, MPU_ADDR, MPU_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  i2cRead(bus, MPU_ADDR, MPU_FIFO_COUNTH, data, 2); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};

    // Read data from FIFO
    i2cRead(bus, MPU_ADDR, MPU_FIFO_R_W, data, 12);

    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }

  // Normalize sums to get average count biases
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;
    
  // Remove gravity from the z-axis accelerometer bias calculation
  if (accel_bias[2] > 0L)
    accel_bias[2] -= (int32_t) accelsensitivity;
  else
    accel_bias[2] += (int32_t) accelsensitivity;
   
  // Construct the gyro biases for push to the hardware gyro bias registers,
  // which are reset to zero upon device startup
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  // Biases are additive, so change sign on calculated average gyro biases
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF;
  data[1] = (-gyro_bias[0] / 4)       & 0xFF;
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;
  
  // Push gyro biases to hardware registers
  i2cWriteOne(bus, MPU_ADDR, MPU_XG_OFFSET_H, data[0]);
  i2cWriteOne(bus, MPU_ADDR, MPU_XG_OFFSET_L, data[1]);
  i2cWriteOne(bus, MPU_ADDR, MPU_YG_OFFSET_H, data[2]);
  i2cWriteOne(bus, MPU_ADDR, MPU_YG_OFFSET_L, data[3]);
  i2cWriteOne(bus, MPU_ADDR, MPU_ZG_OFFSET_H, data[4]);
  i2cWriteOne(bus, MPU_ADDR, MPU_ZG_OFFSET_L, data[5]);
  
  // Construct the accelerometer biases for push to the hardware accelerometer bias registers.
  // These registers contain factory trim values which must be added to the calculated accelerometer
  // biases; on boot up these registers will hold non-zero values. In addition, bit 0 of the lower
  // byte must be preserved since it is used for temperature compensation calculations. Accelerometer
  // bias registers expect bias input as 2048 LSB per g, so that the accelerometer biases calculated
  // above must be divided by 8.
  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  i2cRead(bus, MPU_ADDR, MPU_XA_OFFSET_H, data, 2); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  i2cRead(bus, MPU_ADDR, MPU_YA_OFFSET_H, data, 2);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  i2cRead(bus, MPU_ADDR, MPU_ZA_OFFSET_H, data, 2);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  // Define array to hold mask bit for each accelerometer bias axis
  uint8_t mask_bit[3];
  
  for(ii = 0; ii < 3; ii++)
    // If temperature compensation bit is set, record that fact in mask_bit
    mask_bit[ii] = (uint8_t) (accel_bias_reg[ii] & 0x1);
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[0] -= (accel_bias[0]/8);
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFE;
  // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFE;
  // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFE;
  // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[5] = data[5] | mask_bit[2];
 
  // Push accelerometer biases to hardware registers
  i2cWriteOne(bus, MPU_ADDR, MPU_XA_OFFSET_H, data[0]);
  i2cWriteOne(bus, MPU_ADDR, MPU_XA_OFFSET_L, data[1]);
  i2cWriteOne(bus, MPU_ADDR, MPU_YA_OFFSET_H, data[2]);
  i2cWriteOne(bus, MPU_ADDR, MPU_YA_OFFSET_L, data[3]);
  i2cWriteOne(bus, MPU_ADDR, MPU_ZA_OFFSET_H, data[4]);
  i2cWriteOne(bus, MPU_ADDR, MPU_ZA_OFFSET_L, data[5]);

  // Not needed actually but why not?
  gyroOfs.x = gyro_bias[0];
  gyroOfs.y = gyro_bias[1];
  gyroOfs.z = gyro_bias[2];
  accScale.x = 16384.0;
  accScale.y = 16384.0;
  accScale.z = 16384.0;
  accOfs.x = accel_bias[0];
  accOfs.y = accel_bias[1];
  accOfs.z = accel_bias[2];
  gyroCalibrated = true;
  accelCalibrated = true;
  baroCalibrated = true;
} // void imu::calibMPU6500()
#endif

#ifdef IMU_GY80
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

// Acceleration sensor calibration
_status imu::calibAccelGY80() {
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
  debug(L_DEBUG, (char *) F("imu::calibAccelGY80: offsets %g %g %g, scale %g %g %g\n"),
        accOfs.x, accOfs.y, accOfs.z, accScale.x, accScale.y, accScale.z);
  accelCalibrated = true;
  return _status::NOERR;
} // void imu::calibAccelGY80()
#endif // IMU_GY80

// Swap two bytes
void imu::swapBytes(uint8_t *p) {
  uint8_t tmp;

  tmp = *p;
  *p = *(p + 1);
  *(p + 1) = tmp;
} // void imu::swapBytes(uint8_t *p)

#ifdef IMU_GY80
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
  
  acc.x = acc.y = acc.z = 0.0;

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
_status imu::calibGyroGY80() {
  struct _vec_float_t ofs;

  debug(L_DEBUG, (char *) F("imu::calibGyroGY80\n"));
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
    debug(L_INFO, (char *) F("imu::calibGyroGY80: min %g, max %g, ofs.z %g, noise %g\n"),
          zmin, zmax, ofs.z, gyroNoise);
    // Found optimum
    if (gyroNoise < 20)
      break;
    // new offset
    gyroOfs = ofs;
  }
  debug(L_INFO, (char *) F("imu::calibGyroGY80: ofs.x %g, ofs.y %g, ofs.z %g\n"), ofs.x, ofs.y, ofs.z);
  gyroCalibrated = true;
} // _status imu::calibGyroGY80()

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
#endif // IMU_GY80

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

  #ifdef IMU_GY80
  // Initialize IMU
  imuLocked = true;
  if (!initL3G4200D())
    return _status::ERR;
  initBMP085();
  initADXL345B();
  initHMC5883L();
  imuLocked = false;
  #endif
  #ifdef IMU_MPU9250
  imuLocked = true;
  if (!checkMPU9250())
    return _status::ERR;
  calibMPU6500();
  initMPU6500();
  initAK8963();
  imuLocked = false;
  #endif

#ifdef DEBUG_IMU_CONSTANT
  if (!compassCalibrated) {
    // permanent debug calibration values
#ifdef IMU_GY80
    comOfs.x = -779;
    comOfs.y = -320;
    comOfs.z = 1205.5;
    comScale.x = 669.00;
    comScale.y = 884.00;
    comScale.z = 139.00;
#else
    comOfs.x = 0;
    comOfs.y = 0;
    comOfs.z = 0;
    comScale.x = 1.0;
    comScale.y = 1.0;
    comScale.z = 1.0;
#endif
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

// Current pressure
// Not implemented yet
int16_t imu::readCurPressure(numThing n) {
  return 0;
} // int16_t imu::readCurPressure(numThing n)

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
