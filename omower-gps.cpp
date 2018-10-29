// GPS class for OMower
// $Id$

// Problems with Os optimization!
#pragma GCC optimize ("O2")

#include <omower-gps.h>
#include <Arduino.h>
#include <omower-debug.h>
#include <omower-imu.h>

// Constructor
gps::gps() {
  imuSens = NULL;
  odoSens = NULL;
  startAutocorrection = endAutocorrection = flagAutocorrection = false;
} // gps::gps()

_status gps::init() {
  numSats = 0;
  lastReceived = 0;
  lastPrecision = 0;
  for (int i = 0; i < AUTOCORR_SECTORS; i++)
    offsetSectors[i] = 0;
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
    speed = (uint16_t) (tinygps.speed.mps() * 100.0f);
    if (tinygps.course.deg() > 180.0f)
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
    deltaDist = 0.5f * (float)(deltaLeft + deltaRight) * odoSens->ticksCm;
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
    // First split way point should be closer after turn
    calcPoint(SPLIT_DISTANCE / 2, latitudeTarg, longitudeTarg, latitudeDest, longitudeDest);
  startAutocorrection = endAutocorrection = flagAutocorrection = false;
} // void gps::setTarget(int32_t latitudeDest, int32_t longitudeDest, boolean precisionOnly, boolean splitWay)

// Course error for gps navigation
float gps::readCourseError() {
  float degreeCurF = imu::degreePI(degree);
  float degreeDestF;
  // 1 meter distance reach if not precise
  float maxDist = stopDistance;
  float curDist;

  // 10 cm precision if in precision mode
  if (precisionTarg)
    maxDist = stopDistancePrecision;

  // Stop if we've reached the destination
  curDist = abs(distance(latitude, longitude, latitudeTarg, longitudeTarg));
  if (curDist <= maxDist) {
    // Check if distance is increasing instead decreasing or we're in less than 5 centimeters from the target
    if ((lastDist < curDist) || (curDist < 0.05f)) {
      debug(L_INFO, (char *) F("gps::readCourseError: reached destination (cur %ld %ld)\n"), latitude, longitude);
      if ((latitudeTarg == latitudeLast) && (longitudeTarg == longitudeLast)) {
        destReached = true;
        flagAutocorrection = startAutocorrection = endAutocorrection = false;
        return -1000;
      } else {
        // Split way, we've reached next point and calculating new one
        // Check if we're close to the end already or just make new point
        if (abs(distance(latitudeTarg, longitudeTarg, latitudeLast, longitudeLast)) < SPLIT_DISTANCE) {
          latitudeTarg = latitudeLast;
          longitudeTarg = longitudeLast;
        } else {
          calcPoint(SPLIT_DISTANCE, latitudeTarg, longitudeTarg, latitudeLast, longitudeLast);
          startAutocorrection = flagAutocorrection = true;
        }
        debug(L_INFO, (char *) F("gps::readCourseError: new waypoint: %ld %ld\n"), latitudeTarg, longitudeTarg);
      }
    }
  } else {
    // Check if we have to finish autocorrection cycle
    if (flagAutocorrection && (curDist < (SPLIT_DISTANCE / 2)) && (latitudeLast != longitudeTarg) && (longitudeLast != longitudeTarg))
      endAutocorrection = true;
  }
  lastDist = curDist;
  // Temporary stop if no precision info or no info at all
  if ((precisionTarg && (numSats < 128)) || (numSats == 0))
    return 1000;

  // Get course from IMU if available
  if (imuSens != NULL) 
    degreeCurF = imuSens->readCurDegreeRad(0);

  // Calculate course correction
  degreeDestF = bearing(latitude, longitude, latitudeTarg, longitudeTarg);  

#ifndef DISABLE_COMPASS_AUTOCORRECTION
  degreeDestF = imu::scalePI(degreeDestF + offsetSectors[calcSector(degreeDestF)]);
#endif

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
    // Check for compass autocorrection flags
    if (flagAutocorrection) {
      if (startAutocorrection) {
        initAutocorr(latitudeRaw, longitudeRaw, latitudeTarg, longitudeTarg);
        startAutocorrection = false;
      }
      if (endAutocorrection) {
        calcOffset(latitudeRaw, longitudeRaw);
        endAutocorrection = flagAutocorrection = false;
      }
    }
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
  deltaX = (dist * sin(angle) * 100.0f) / KCORR_CM_TO_COORD;
  deltaY = (dist * cos(angle) * 100.0f) / KCORR_CM_TO_COORD;
  latitudeP = latitude + (int32_t) deltaY;
  longitudeP = longitude + (int32_t) deltaX;
} // void gps::calcPoint

// Initialise autocorrection calculation (must be called at start of measured movement)
// latStart, lonStart - coordinates of robots at start of movement
// latEnd, lonEnd - coordinates of destination point
void gps::initAutocorr(int32_t latStart, int32_t lonStart, int32_t latEnd, int32_t lonEnd) {
  startDir = bearing(latStart, lonStart, latEnd, lonEnd);
  latS = latStart;
  lonS = lonStart;
  debug(L_NOTICE, (char *) F("gps::initAutocorr: s: %d %d e: %d %d, bearing %f\n"), latStart, lonStart, latEnd, lonEnd, startDir);
} // void gps::initAutocorr(int32_t latStart, int32_t lonStart, int32_t latEnd, int32_t lonEnd)

// Calculate compass autocorrection offset (must be called at middle of measured movement)
// lat, lon - current robot's coordinates
// Updates offsetSectors array
void gps::calcOffset(int32_t lat, int32_t lon) {
  float trueBearing, diffBearing, currOffset;
  int sect;

#ifndef DISABLE_COMPASS_AUTOCORRECTION
  sect = calcSector(startDir);
  // Real direction from coordinates change
  trueBearing = bearing(latS, lonS, lat, lon);
  // Current offset for the sector
  currOffset = offsetSectors[sect];
  // Calculate offset angle from coordinates change
  diffBearing = imu::scalePI(startDir - trueBearing + currOffset);
  // Update offset angle for the sector (average)
  offsetSectors[sect] = (currOffset + diffBearing) / 2;
  // Prevent to make too big offset
  if (abs(offsetSectors[sect]) > (M_PI / 8))
    offsetSectors[sect] /= 4;
  debug(L_NOTICE, (char *) F("gps::calcOffset: s: %d %d e: %d %d, old: %f trueBearing: %f new %f sect %d\n"), latS, lonS, lat, lon, currOffset, trueBearing, offsetSectors[sect], sect);
#endif
} // void gps::calcOffset(int32_t lat, int32_t lon)

// Calculate sector from angle
uint8_t gps::calcSector(float angle) {
  return int((imu::scalePI(angle + AUTOCORR_SECTOR_ANGLE / 2) + M_PI) / AUTOCORR_SECTOR_ANGLE + 0.5);
} // uint8_t gps::calcSector(float angle)

