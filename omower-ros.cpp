// ROS support for OMower
// $Id$

#include <omower-ros.h>

#ifdef USE_ROS
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#endif

// ROS object
omowerROS oROS;

// Serial port number for ROS
numThing nPort = 0;

#ifdef USE_ROS
// Which sensors need to be reported on spinOnce() and its data
boolean rosChangedSensors[(int) reportSensor::LAST_VALUE];
uint8_t *rosDataSensors[(int) reportSensor::LAST_VALUE];
uint8_t rosNumDataSensors[(int) reportSensor::LAST_VALUE];

uint8_t rosInputCmdBuf[64];
uint16_t rosInputLength;
boolean rosCmdBufProcessed = false;

void cbCmdReceived(const std_msgs::UInt8MultiArray& msg) {
  rosCmdBufProcessed = false;
  rosInputLength = min(msg.data_length, sizeof(rosInputCmdBuf));
  memcpy(rosInputCmdBuf, msg.data, rosInputLength);
} // void cbCmdReceived(const std_msgs::String& msg)

// Hardware definition for OMower's serial port
omowerROSHardware::omowerROSHardware() {
}

void omowerROSHardware::init() {
}

int omowerROSHardware::read() {
  if (oSerial.available(nPort))
    return oSerial.read(nPort);
  return -1;
}

void omowerROSHardware::write(uint8_t *data, int length) {
  oSerial.write(nPort, data, length, 100);
  return;
}

unsigned long omowerROSHardware::time() {
  return millis();
}

// ROS handler object
ros::omowerNodeHandle nh;

// All publishers and subscribers
// Debugging log channel
std_msgs::String strLog;
ros::Publisher pubLog("debug", &strLog);

std_msgs::MultiArrayDimension rosDim[1];

// pfodApp/Modbus output channel
std_msgs::UInt8MultiArray strCmd;
ros::Publisher pubCmdOut("cmdOut", &strCmd);

// pfodApp/Modbus input channel
ros::Subscriber<std_msgs::UInt8MultiArray> subCmdIn("cmdIn", &cbCmdReceived );

// Motors PWM output (int16_t [2])
std_msgs::Int16MultiArray pwmMot;
ros::Publisher pubMotorPwm("motors/PWM", &pwmMot);

// Motors PID calculation (float [4])
std_msgs::Float32MultiArray pidMot;
ros::Publisher pubMotorPid("motors/PID", &pidMot);

// Sonars (uint16_t [SONARS_NUM])
std_msgs::UInt16MultiArray sonRep;
ros::Publisher pubSonarsRep("sonars/distance", &sonRep);

// Buttons and LEDs (uint8_t [6])
std_msgs::UInt8MultiArray bttRep;
ros::Publisher pubBttLedsRep("buttons_leds/status", &bttRep);

// Odometry (int32_t [4])
std_msgs::Int32MultiArray odoRep;
ros::Publisher pubOdometryRep("odometry/ticks", &odoRep);

// Mowing motor pwm (int16_t)
std_msgs::Int16MultiArray mowPwm;
ros::Publisher pubMowPwm("mowmotors/PWM", &mowPwm);

// IMU orientation (int16_t [3])
std_msgs::Int16MultiArray imuRep;
ros::Publisher pubImuRep("imu/orientation", &imuRep);

// GPS (int32_t [3])
std_msgs::Int32MultiArray gpsRep;
ros::Publisher pubGpsRep("gps/coordinates", &gpsRep);

// Virtual Perimeter (uint8_t)
std_msgs::UInt8MultiArray vpRep;
ros::Publisher pubVirtPerim("virtperim/status", &vpRep);

// Motors currents (float [4])
std_msgs::Float32MultiArray crrMot;
ros::Publisher pubMotorCurrent("motors/current", &crrMot);

// Mowing motor current (float)
std_msgs::Float32MultiArray crrMow;
ros::Publisher pubMowCurrent("mowmotors/current", &crrMow);

// Bumpers (uint8_t [2])
std_msgs::UInt8MultiArray bmpRep;
ros::Publisher pubBumpers("bumpers/status", &bmpRep);

// Voltage sensors
std_msgs::Float32MultiArray powVRep;
ros::Publisher pubVoltages("power/voltage", &powVRep);

// Curent sensors
std_msgs::Float32MultiArray powCRep;
ros::Publisher pubCurrents("power/current", &powCRep);

// Seeker
std_msgs::Int16MultiArray seekRep;
ros::Publisher pubSeekerRep("seeker/status", &seekRep);

// Perimeter (uint8_t [4])
std_msgs::UInt8MultiArray perimRep;
ros::Publisher pubPerimeter("perimeter/status", &perimRep);
#endif

void omowerROS::setSerialPort(numThing n) {
  nPort = n;
} // void omowerROS::setSerialPort(numThing n)

void omowerROS::advertiseSubscribe() {
#ifdef USE_ROS
  // Need the delay for rosserial_python to sync
  delay(2000);
  nh.initNode();
  lastForceReport = millis();

  // Creating dummy layout for 1D array
  strCmd.layout.dim = rosDim;
  pwmMot.layout.dim = rosDim;
  rosDim[0].label = "";
  rosDim[0].stride = 1;
  rosDim[0].size = 0;
  strCmd.layout.data_offset = 0;
  pwmMot.layout.data_offset = 0;

  // Advertise our sensors
  nh.advertise(pubLog);
  nh.advertise(pubCmdOut);
  nh.advertise(pubMotorPwm);
  nh.advertise(pubMotorPid);
  nh.advertise(pubSonarsRep);
  nh.advertise(pubBttLedsRep);
  nh.advertise(pubOdometryRep);
  nh.advertise(pubMowPwm);
  nh.advertise(pubImuRep);
  nh.advertise(pubGpsRep);
  nh.advertise(pubVirtPerim);
  nh.advertise(pubMotorCurrent);
  nh.advertise(pubMowCurrent);
  nh.advertise(pubBumpers);
  nh.advertise(pubVoltages);
  nh.advertise(pubCurrents);
  nh.advertise(pubSeekerRep);
  nh.advertise(pubPerimeter);

  // Subscribe to input commands
  nh.subscribe(subCmdIn);

  nh.spinOnce();
#endif
} // void advertiseSubscribe()

void omowerROS::spinOnce() {
  int cnt = 0;

#ifdef USE_ROS
  nh.spinOnce();

  // Check and report first 5 changed sensors
  for (int i = 0; i < (int) reportSensor::LAST_VALUE; i++) {
    if (rosChangedSensors[i]) {
      rosChangedSensors[i] = false;
      publishSensor((reportSensor) i, rosDataSensors[i], rosNumDataSensors[i]);
      cnt++;
      if (cnt > 5)
        break;
    }
  }
  
  // Check if we have to do force report for a sensor
  if ((millis() - lastForceReport) > DELAY_FORCE_REPORT) {
    lastForceReport = millis();
    rosChangedSensors[currentSensor] = true;
    currentSensor++;
    if (currentSensor >= (uint8_t) reportSensor::LAST_VALUE)
      currentSensor = (uint8_t) reportSensor::FIRST_VALUE;
  }
#endif
} // void omowerROS::spinOnce()

// Send to command channel (pfod menu, modbus answers)
void omowerROS::publishCmd(uint8_t *msg, uint16_t length) {
#ifdef USE_ROS
  strCmd.data = msg;
  strCmd.data_length = length;
  strCmd.layout.dim[0].size = length;

  pubCmdOut.publish(&strCmd);
#endif
} // void omowerROS::publishCmd(uint8_t *msg, uint16_t length)

uint16_t omowerROS::checkCmd(uint8_t **pCmd) {
#ifdef USE_ROS
  if (!rosCmdBufProcessed) {
    rosCmdBufProcessed = true;
    if (rosInputLength > 0) {
      *pCmd = rosInputCmdBuf;
      return rosInputLength;
    }
  }
  *pCmd = NULL;
#endif
  return 0;
} // uint16_t omowerROS::checkCmd(uint8_t **pCmd)

// Constructor
omowerROS::omowerROS() {
#ifdef USE_ROS
  // Clear arrays for sensors's data
  for (int i = 0; i < (int) reportSensor::LAST_VALUE; i++) {
    rosChangedSensors[i] = false;    
    rosDataSensors[i] = NULL;
    rosNumDataSensors[i] = 0;
  }
  currentSensor = (uint8_t) reportSensor::DEBUG;
#endif
} // omowerROS::omowerROS()

void omowerROS::reportToROS(reportSensor sensor, uint8_t *data, uint8_t num) {
#ifdef USE_ROS
  rosDataSensors[(int) sensor] = data;
  rosNumDataSensors[(int) sensor] = num;
  rosChangedSensors[(int) sensor] = true;
#endif
} // void omowerROS::reportToROS(reportSensor sensor, uint8_t *data, uint8_t length)

// Publish sensors information to ROS
void omowerROS::publishSensor(reportSensor sensor, uint8_t *data, uint8_t num) {
#ifdef USE_ROS
  // Don't publish if nothing
  if (num == 0)
    return;

  switch (sensor) {
    case reportSensor::DEBUG:
      strLog.data = (char *) data;
      pubLog.publish(&strLog);
      resetDebugOutput();
      break;
    case reportSensor::MOTORPWM:
      // Current motors PWM output
      pwmMot.data = (int16_t *) data;
      pwmMot.data_length = num;
      pwmMot.layout.dim[0].size = num;
      pubMotorPwm.publish(&pwmMot);
      break;
    case reportSensor::MOTORPID:
      // Current motors PID calculation
      pidMot.data = (float *) data;
      pidMot.data_length = num;
      pidMot.layout.dim[0].size = num;
      pubMotorPid.publish(&pidMot);
      break;
    case reportSensor::SONARS:
      // Sonars
      sonRep.data = (uint16_t *) data;
      sonRep.data_length = num;
      sonRep.layout.dim[0].size = num;
      pubSonarsRep.publish(&sonRep);
      break;
    case reportSensor::BUTTONS:
      // Buttons and LEDs
      bttRep.data = data;
      bttRep.data_length = num;
      bttRep.layout.dim[0].size = num;
      pubBttLedsRep.publish(&bttRep);
      break;
    case reportSensor::ODOMETRY:
      // Odometry
      odoRep.data = (int32_t *) data;
      odoRep.data_length = num;
      odoRep.layout.dim[0].size = num;
      pubOdometryRep.publish(&odoRep);
      break;
    case reportSensor::MOWPWM:
      // Mowing motor PWM
      mowPwm.data = (int16_t *) data;
      mowPwm.data_length = num;
      mowPwm.layout.dim[0].size = num;
      pubMowPwm.publish(&mowPwm);
      break;
    case reportSensor::ORIENTATION:
      // IMU
      imuRep.data = (int16_t *) data;
      imuRep.data_length = num;
      imuRep.layout.dim[0].size = num;
      pubImuRep.publish(&imuRep);
      break;
    case reportSensor::GPS:
      // GPS
      gpsRep.data = (int32_t *) data;
      gpsRep.data_length = num;
      gpsRep.layout.dim[0].size = num;
      pubGpsRep.publish(&gpsRep);
      break;
    case reportSensor::VIRTPERIM:
      // Virtual perimeter
      vpRep.data = data;
      vpRep.data_length = num;
      vpRep.layout.dim[0].size = num;
      pubVirtPerim.publish(&vpRep);
      break;
    case reportSensor::MOTORCURRENT:
      // Currents of wheel motors
      crrMot.data = (float *) data;
      crrMot.data_length = num;
      crrMot.layout.dim[0].size = num;
      pubMotorCurrent.publish(&crrMot);
      break;
    case reportSensor::MOWCURRENT:
      // Currents of mowing motors
      crrMow.data = (float *) data;
      crrMow.data_length = num;
      crrMow.layout.dim[0].size = num;
      pubMowCurrent.publish(&crrMow);
      break;
    case reportSensor::BUMPER:
      // Bumpers
      bmpRep.data = data;
      bmpRep.data_length = num;
      bmpRep.layout.dim[0].size = num;
      pubBumpers.publish(&bmpRep);
    case reportSensor::VOLTAGE:
      // Voltages
      powVRep.data = (float *) data;
      powVRep.data_length = num;
      powVRep.layout.dim[0].size = num;
      pubVoltages.publish(&powVRep);
      break;
    case reportSensor::CURRENT:
      // Currents
      powCRep.data = (float *) data;
      powCRep.data_length = num;
      powCRep.layout.dim[0].size = num;
      pubCurrents.publish(&powCRep);
      break;
    case reportSensor::SEEKER:
      // Seeker
      seekRep.data = (int16_t *) data;
      seekRep.data_length = num;
      seekRep.layout.dim[0].size = num;
      pubSeekerRep.publish(&seekRep);
      break;
    default:
      break;
  }
#endif
} // void omowerROS::publishSensor(reportSensor sensor, uint8_t *data, uint8_t num)
