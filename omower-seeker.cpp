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
  lastAged = millis();
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
void seeker::setSeekerData(int16_t extOffsetAngle, int16_t extOrientAngle, uint16_t extDistance, uint16_t ageMeasure) {
  offsetAngle = extOffsetAngle;
  orientAngle = extOrientAngle;
  distance = extDistance;
  lastReceived = millis();
  // If we have IMU's help - calculate absolute direction needed
  if (imuSens) {
    int16_t totalDiff;
    int16_t aged;
    int16_t nAged;
    int16_t dirMove;

    // Use orientation angle to try to approach from front
    if (flagFixOrientation) {
      if (distance > 130)
        totalDiff = offsetAngle - orientAngle * 1.5;
      else
        totalDiff = offsetAngle - orientAngle;
    } else
      totalDiff = offsetAngle;

    if (totalDiff > 22)
      totalDiff = 22;
    if (totalDiff < -22)
      totalDiff = -22;
    // Get compass reading for about that age of seeker's measure
    // It'll help to keep robot's movement stable even if seeker calculates very slow
    nAged = ageMeasure / 100  + 1;
    if (nAged > 5)
      nAged = 5;
    aged = agedCompass[nAged];
    // Calculate direction by compass
    dirMove = aged + totalDiff;
    if (dirMove >= 180)
      dirMove = -(360 - dirMove);
    if (dirMove <= -180)
      dirMove = 360 + dirMove;
    absDir = imu::degreePI(dirMove);
  }
  debug(L_INFO, (char *) F("setSeekerData: %hd, %hd, %hd %f\n"),
        offsetAngle, orientAngle, distance, absDir);
} // void seeker::setSeekerData(int16_t extOffsetAngle, int16_t extOrientAngle, uint16_t extDistance, uint16_t ageMeasure)
  

// Must be called 20 times per second
void seeker::poll20() {
  // Check if data is too old
  if ((millis() - lastReceived) > maxTimeout)
    distance = 0;

  // in IMU's guided mode - update agedCompass
  if (imuSens) {
    if ((millis() - lastAged) >= 100) {
      lastAged = millis();

      // Shift agedCompass values
      for (int i = 0; i < 4; i++) {
        agedCompass[5 - i - 1] = agedCompass[5 - i - 2];
      }
      // Remember current value of compass
      agedCompass[0] = imuSens->readCurDegree(-1);
    }
  }
} // void seeker::poll20()
