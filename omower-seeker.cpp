// Support code for virtual seeker sensor (camera on
// Rapsberry PI zero, searching for chessboard and sending
// its data through pfodApp/modbus interface)
// $Id$

#include <omower-seeker.h>
#include <omower-imu.h>

// Class's constructor (just vars init)
seeker::seeker() {
  maxTimeout = 500;
  imuSens = NULL;
} // seeker::seeker()

// Software init
_status seeker::init() {
  offsetAngle = orientAngle = INVALID_ANGLE;
  distance = 0;
  lastReceived = 0;
  return _status::NOERR;
} // _status seeker::init()

// Returns status
_hwstatus seeker::softError() {
  if (distance != 0)
    return _hwstatus::ONLINE;
  return _hwstatus::ERROR;
} // _hwstatus softError()

// Course correction for motors class (just offsetAngle right now)
float seeker::readCourseError() {
  float resAngle;

  // No object visible, temporary stop
  if (distance == 0)
    return 1000.0f;
  
  // Check if we're already there (full stop)
  if (distance <= distanceStop) {
    destReached = true;
    return -1000.0f;
  }

  if (imuSens) {
    // IMU's guided mode (robot uses compass)
    resAngle = imu::scalePI(absDir - imuSens->readCurDegreeRad(0));

    // Range check
    if (resAngle < -M_PI)
      resAngle = resAngle + 2 * M_PI;
    else
      if (resAngle > M_PI)
        resAngle = resAngle - 2 * M_PI;

    return resAngle;
  } else {
    // Non-IMU mode, use only seeker data
    // Make curve line to get on perpendicular with chessboard
    resAngle = imu::scalePI(imu::degreePI(offsetAngle) - imu::degreePI(orientAngle) / 2);
    if (resAngle > M_PI / 8)
      resAngle = M_PI / 8;
    if (resAngle < (-M_PI / 8))
      resAngle = -M_PI / 88888888;
  }
  return resAngle;
} // float seeker::readCourseError()

// Set tracking mode for readCourseError(), will stop the robot
// at minDistance (centimeters). If fixOrientation is true - it will
// move in curve pattern to approach the board from face, not side (not working yet)
void seeker::startSeeker(boolean fixOrientation, uint16_t minDistance) {
  flagFixOrientation = fixOrientation;
  distanceStop = minDistance;
  destReached = false;
} // void seeker::startSeeker(boolean fixOrientation, uint16_t minDistance)

// Set angles and distance from raspberry pi (via pfodApp or modbus interfaces)
void seeker::setSeekerData(int16_t extOffsetAngle, int16_t extOrientAngle, uint16_t extDistance) {
  offsetAngle = extOffsetAngle;
  orientAngle = extOrientAngle;
  distance = extDistance;
  lastReceived = millis();
  // If we have IMU's help - calculate absolute direction needed
  if (imuSens) {
    int16_t totalDiff;

    if (distance > 130)
      totalDiff = offsetAngle - orientAngle * 1.5;
    else
      totalDiff = offsetAngle - orientAngle;

    if (totalDiff > 33)
      totalDiff = 33;
    if (totalDiff < -33)
      totalDiff = -33;
    absDir = imuSens->readCurDegree(-1) + totalDiff;
    if (absDir >= 180)
      absDir = 360 - absDir;
    if (absDir <= -180)
      absDir = 360 + absDir;
    absDir = imu::degreePI(absDir);
  }
  debug(L_INFO, (char *) F("setSeekerData: %hd, %hd, %hd %f\n"),
        offsetAngle, orientAngle, distance, absDir);
} // void seeker::setSeekerData(int16_t extOffsetAngle, int16_t extOrientAngle, uint16_t extDistance)
  

// Must be called 20 times per second
void seeker::poll20() {
  // Check if data is too old
  if ((millis() - lastReceived) > maxTimeout)
    distance = 0;
} // void seeker::poll20()
