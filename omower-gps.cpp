// GPS class for OMower
// $Id$

#include <omower-gps.h>
#include <Arduino.h>
#include <omower-debug.h>
#include <omower-imu.h>

// Constructor
gps::gps() {
  imuSens = NULL;
  odoSens = NULL;
} // gps::gps()

_status gps::init() {
  numSats = 0;
  lastReceived = 0;
  lastPrecision = 0;
  return _status::NOERR;
} // _status gps::init()

_hwstatus gps::softError() {
  if ((millis() - lastReceived) > 120000)
    return _hwstatus::ERROR;
  return _hwstatus::ONLINE;
} // _hwstatus gps::softError()

// Convert tinygps's RawDegrees to int32_t (+/-DDMMMMMMM)
int32_t gps::coordToInt(const RawDegrees &degree) {
  int32_t res = degree.deg * 10000000L;

  res = res + (degree.billionths + 50) / 100;
  if (degree.negative)
    res = -res;
  return res;
} // uint32_t gps::coordToInt(RawDegrees &degree)

// Parse GPS string(s)
void gps::parseString(char *buf, uint16_t len) {
  // Send all characters to tinygps
  for (uint16_t i = 0; i < len; i++)
    tinygps.encode(buf[i]);
  
  // Check if new coordinates/date/time received, update timestamp and copy stuff
  lastReceived = millis();
  if (tinygps.location.isValid() && tinygps.location.isUpdated()) {
    numSats = tinygps.satellites.value();
    // Use data only if more than 3 satellites visible
    if (numSats > 3) {
      latitude = coordToInt(tinygps.location.rawLat());
      longitude = coordToInt(tinygps.location.rawLng());
      latitudeRaw = coordToInt(tinygps.location.rawLat());
      longitudeRaw = coordToInt(tinygps.location.rawLng());
      if (numSats > 128) {
        resetOdometryTicks();
        correctCoords();
        lastPrecision = millis();
      }
    } else
      numSats = 0;
    debug(L_DEBUG, (char *) F("gps::parseString: new coordinates received %ld %ld %hu\n"),
          latitude, longitude, numSats);
  }
  if (tinygps.speed.isValid() && tinygps.speed.isUpdated()) {
    speed = (uint16_t) (tinygps.speed.mps() * 100.0);
    if (tinygps.course.deg() > 180.0)
      degree = -360 + tinygps.course.deg();
    else
      degree = tinygps.course.deg();
    debug(L_DEBUG, (char *) F("gps::parseString: course %hd, speed %hu\n"), degree, speed);
  }
  if (tinygps.date.isValid() && tinygps.date.isUpdated()
      && (tinygps.date.year() > 2016) && (tinygps.date.year() < 2050)) {
    year = tinygps.date.year();
    month = tinygps.date.month();
    day = tinygps.date.day();
    debug(L_DEBUG, (char *) F("gps::parseString: new date received %hu-%hu-%hu\n"),
          year, month, day);
  }
  if (tinygps.time.isValid() && tinygps.time.isUpdated() && (tinygps.time.hour() < 24)) {
    hour = tinygps.time.hour();
    minute = tinygps.time.minute();
    second = tinygps.time.second();
    debug(L_DEBUG, (char *) F("gps::parseString: new time received %hu:%hu:%hu\n"),
          hour, minute, second);
  }
} // void gps::parseString(char *buf, uint16_t len)

// 10 times per second poll (check if coordinates are still valid)
void gps::poll10() {
  float deltaX, deltaY, angleCur;
  int32_t deltaLeft, deltaRight;
  float deltaDist;

  if ((millis() - lastReceived) > maxTimeout) {
    debug(L_DEBUG, (char *) F("gps::poll10: millis %lu, lastReceived %lu, maxTimeout %hu\n"), millis(),
          lastReceived, maxTimeout);
    numSats = 0;
  }
  // Correct coordinates by odometer's reading
  if (odoSens && imuSens && ((millis() - lastPrecision) < maxTimeout)) {
    // Calculate distance from odometry ticks
    deltaLeft = odoSens->readTicks(0) - leftTicks;
    deltaRight = odoSens->readTicks(1) - rightTicks;
    deltaDist = 0.5 * (float)(deltaLeft + deltaRight) * odoSens->ticksCm;
    angleCur = imu::degreePI(imuSens->readCurDegree(0));
    deltaX = (deltaDist * sin(angleCur)) / KCORR_CM_TO_COORD;
    deltaY = (deltaDist * cos(angleCur)) / KCORR_CM_TO_COORD;
    debug(L_NOTICE, (char *) F("gps before odo corr: %ld %ld\n"), latitudeRaw, longitudeRaw);
    latitude = latitudeRaw + (int32_t) deltaY;
    longitude = longitudeRaw + (int32_t) deltaX;
    debug(L_NOTICE, (char *) F("gps odo corr %ld %ld, %03.2f %03.2f (%03.2f), final: %ld %ld\n"),
          deltaLeft, deltaRight, deltaX, deltaY, angleCur, latitude, longitude);
  }
} // void gps::poll10()

// Bearing to the GPS point (-M_PI..M_PI)
float gps::bearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  return atan2(lon2 - lon1, (lat2 - lat1) / cos(lat1 * 1.0e-7f * 0.001745329251f));
} // float gps::bearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)

float gps::distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  float dLat = (float)(lat2 - lat1);
  float dLon = (float)(lon2 - lon1) * cos(lat1 * 1.0e-7f * 0.001745329251f);

  return sqrt(dLat*dLat + dLon*dLon) * 0.0111318845f;
} // float gps::distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) 

// Calculate distance to the current position
float gps::calcDist(int32_t latitude, int32_t longitude) {
  return distance(gps::latitude, gps::longitude, latitude, longitude);
} // float gps::calcDist(int32_t latitude, int32_t longitude)

// Sets coordinates for course correction
// if precisionOnly set - stops when lost precision data
void gps::setTarget(int32_t latitudeDest, int32_t longitudeDest, boolean precisionOnly, boolean splitWay) {
  latitudeLast = latitudeDest;
  longitudeLast = longitudeDest;
  precisionTarg = precisionOnly;
  destReached = false;
  // calculating next point of the destination in split-way mode or just go to the coordinates
  if (!splitWay) {
    latitudeTarg = latitudeLast;
    longitudeTarg = longitudeLast;
  } else 
    calcPoint(1.0, latitudeTarg, longitudeTarg, latitudeDest, longitudeDest);
} // void gps::setTarget(int32_t latitudeDest, int32_t longitudeDest, boolean precisionOnly, boolean splitWay)

// Course error for gps navigation
float gps::readCourseError() {
  int16_t degreeCur = degree;
  float degreeCurF, degreeDestF;
  // 1 meter distance reach if not precise
  float maxDist = stopDistance;
  float curDist;

  // 10 cm precision if in precision mode
  if (precisionTarg)
    maxDist = stopDistancePrecision;

  // Stop if we've reached the destination
  curDist = abs(distance(latitude, longitude, latitudeTarg, longitudeTarg));
  if (curDist <= maxDist) {
    // Check if distance is increasing instead decreasing or we're in less than 3 centimeters from the target
    if ((lastDist < curDist) || (curDist < 0.03)) {
      debug(L_INFO, (char *) F("gps::readCourseError: reached destination (cur %ld %ld)\n"), latitude, longitude);
      if ((latitudeTarg == latitudeLast) && (longitudeTarg == longitudeLast)) {
        destReached = true;
        return -1000;
      } else {
        // Split way, we've reached next point and calculating new one
        // Check if we're close to the end already or just make new point
        if (abs(distance(latitudeTarg, longitudeTarg, latitudeLast, longitudeLast)) < 2.0) {
          latitudeTarg = latitudeLast;
          longitudeTarg = longitudeLast;
        } else 
          calcPoint(2.0, latitudeTarg, longitudeTarg, latitudeLast, longitudeLast);
          debug(L_INFO, (char *) F("gps::readCourseError: new waypoint: %ld %ld\n"), latitudeTarg, longitudeTarg);
      }
    }
  }
  lastDist = curDist;
  // Temporary stop if no precision info or no info at all
  if ((precisionTarg && (numSats < 128)) || (numSats == 0))
    return 1000;

  // Get course from IMU if available
  if (imuSens != NULL) 
    degreeCur = imuSens->readCurDegree(0);

  // Calculate course correction
  degreeCurF = imu::degreePI(degreeCur);
  degreeDestF = bearing(latitude, longitude, latitudeTarg, longitudeTarg);  

  float d = imu::scalePI(degreeDestF - degreeCurF);
  debug(L_INFO, (char *) F("gps::readCourseError: %f (%f %f)\n"), d, degreeDestF, degreeCurF);

  // Range check
  if (d < -M_PI)
    d = d + 2 * M_PI;
  else
    if (d > M_PI)
      d = d - 2 * M_PI;
  return d;
} // float gps::readCourseError()

// set GPS coordinates and time from external sources (ignore date/time if year == 0)
void gps::setCoords(int32_t latitudeNew, int32_t longitudeNew, uint8_t numSatsNew,
                    uint16_t yearNew, uint16_t monthNew, uint16_t dayNew,
                    uint16_t hourNew, uint16_t minuteNew, uint16_t secondNew) {
  latitude = latitudeNew;
  longitude = longitudeNew;
  latitudeRaw = latitudeNew;
  longitudeRaw = longitudeNew;
  numSats = numSatsNew;
  if (numSats > 128) {
    resetOdometryTicks();
    correctCoords();
    lastPrecision = millis();
  }
  lastReceived = millis();
  if (yearNew != 0) {
    year = yearNew;
    month = monthNew;
    day = dayNew;
    hour = hourNew;
    minute = minuteNew;
    second = secondNew;
  }
} // void gps::setCoords

// Correct precision coordinates by antenna's position
void gps::correctCoords() {
  float deltaAngle;
  float deltaX, deltaY;

  if (imuSens && (distCenter > 0)) {
    deltaAngle = imu::degreePI(imu::scaleDegree(angleCenter + imuSens->readCurDegree(0)));
    deltaX = ((float) distCenter * sin(deltaAngle)) / KCORR_CM_TO_COORD;
    deltaY = ((float) distCenter * cos(deltaAngle)) / KCORR_CM_TO_COORD;
    debug(L_DEBUG, (char *) F("gps corr: %03.3f %03.3f (%03.3f - %hd/%hd)\n"), deltaX, deltaY, deltaAngle, angleCenter, imuSens->readCurDegree(0));
    latitudeRaw = latitudeRaw - (int32_t) deltaY;
    longitudeRaw = longitudeRaw - (int32_t) deltaX;
    latitude = latitudeRaw;
    longitude = longitudeRaw;
    debug(L_NOTICE, (char *) F("gps corr after: %ld %ld (%03.3f %03.3f)\n"), latitude, longitude, angleCenter, imu::degreePI(imuSens->readCurDegree(0)));
  }
} // void gsp::correctCoords()

// Reset odometer ticks stats on new coordinates
void gps::resetOdometryTicks() {
  if (odoSens) {
    leftTicks = odoSens->readTicks(0);
    rightTicks = odoSens->readTicks(1);
  }
} // void gps::resetOdometryTicks()

// Calculate coordinates of point on line to the target from current
// coordinates and distance from it
void gps::calcPoint(float dist, int32_t &latitudeP, int32_t &longitudeP,
                     int32_t latitudeDest, int32_t longitudeDest) {
  float angle, deltaX, deltaY;

  angle = bearing(latitude, longitude, latitudeDest, longitudeDest);  
  deltaX = (dist * sin(angle) * 100.0) / KCORR_CM_TO_COORD;
  deltaY = (dist * cos(angle) * 100.0) / KCORR_CM_TO_COORD;
  latitudeP = latitude + (int32_t) deltaY;
  longitudeP = longitude + (int32_t) deltaX;
} // void gps::calcPoint
