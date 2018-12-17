// Virtual perimeter sensor (uses GPS and set of corner points)
// based on code from https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
// $Id$

#include <omower-virtperim.h>
#include <omower-ros.h>

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))

// Define Infinite (Using INT_MAX caused overflow problems) 
#define INF 1800000000L

// Constructor
virtPerim::virtPerim() {
  oGps = NULL;
  imuSens = NULL;
  statsInside = 0;
  currentState = false;
  filterK = 0.2;
  destReached = false;
  errorTimeout = 120;
  lastInside = 0;
  precisionOnly = false;
} // virtPerim::virtPerim()

// "Hardware" initialization (once, resets variables)
_hwstatus virtPerim::begin() {
  for (int i = 0; i < _VIRTPERIM_MAX; i++) {
    perimeter[i].x = INVALID_COORD;
    perimeter[i].y = INVALID_COORD;
  }
  trackingMode = false;
  enabledPerimeter = true;
  return _hwstatus::ONLINE;
} // _hwstatus virtPerim::begin()

// Enable perimeter
_status virtPerim::enableThings() {
  enabledPerimeter = true;
  return _status::NOERR;
} // _status virtPerim::enableThings()

// Disable perimeter
_status virtPerim::disableThings() {
  enabledPerimeter = false;
  return _status::NOERR;
} // _status virtPerim::disableThings()

// Check for perimeter timeout
void virtPerim::poll10() {
  boolean insideCur = checkInside();

  // Filtering inside/outside status
  statsInside = statsInside + filterK * ((insideCur ? 1 : 0) - statsInside);
  // Update current state with hystersis
  if (currentState && (statsInside < 0.5)) {
    currentState = false;
    reportToROS();
  }
  if (!currentState && (statsInside > 0.7)) {
    reportToROS();
    currentState = true;
  }
  if (currentState)
    lastInside = millis();
} // void virtPerim::poll10()

// count number of corners in the perimeter array
int virtPerim::countCorners() {
  int i;

  // Count number of corners in perimeter array
  for (i = 0; i < _VIRTPERIM_MAX; i++)
    if ((perimeter[i].x == INVALID_COORD) || (perimeter[i].y == INVALID_COORD))
      break;
  return i;
} // int virtPerim::countCorners()

// Delete specific corner
void virtPerim::delCorner(int n) {
  // Check for right corner number
  if ((n < 0) || (n >= _VIRTPERIM_MAX))
    return; 

  // Shift array
  for (int i = n; i < (_VIRTPERIM_MAX - 1); i++) {
    perimeter[i].x = perimeter[i + 1].x;
    perimeter[i].y = perimeter[i + 1].y;
  }
  // Clear last corner in array
  perimeter[_VIRTPERIM_MAX - 1].x = perimeter[_VIRTPERIM_MAX - 1].y = INVALID_COORD;
} // void virtPerim::delCorner(int n)

// Append new corner
void virtPerim::appendCorner(int32_t latitude, int32_t longitude) {
  int n = countCorners();

  // Check for overflow
  if (n >= _VIRTPERIM_MAX)
    return;
  perimeter[n].x = longitude;
  perimeter[n].y = latitude;
} // void virtPerim::appendCorner(int32_t latitude, int32_t longitude)

// Return specific corner of the array
void virtPerim::getCorner(int n, int32_t &latitude, int32_t &longitude) {
  // Check for right corner number
  if ((n < 0) || (n >= _VIRTPERIM_MAX)) {
    latitude = longitude = INVALID_COORD;
    return;
  }
  longitude = perimeter[n].x;
  latitude = perimeter[n].y;
} // void virtPerim::getCorner(int n, int32_t &latitude, int32_t &longitude)

// Update specific corner of the array
void virtPerim::setCorner(int n, int32_t latitude, int32_t longitude) {
  // Check for right corner number
  if ((n < 0) || (n >= _VIRTPERIM_MAX))
    return;
  perimeter[n].x = longitude;
  perimeter[n].y = latitude;
} // void virtPerim::setCorner(int n, int32_t latitude, int32_t longitude)

// Returns last state
boolean virtPerim::inside() {
  return currentState;
} // boolean virtPerim::inside()

// Returns unknown status
boolean virtPerim::unknown() {
  if (!enabledPerimeter || !oGps)
    return true;
  if (!precisionOnly && (oGps->numSats == 0))
    return true;
  if (precisionOnly && (oGps->numSats < 128))
    return true;
  return false;
} // boolean virtPerim::unknown()

// If we're inside of perimeter
boolean virtPerim::checkInside() {
  boolean res;
  point_t curLocation;

  // Can't calculate at all
  if (!enabledPerimeter || !oGps)
    return false;

  if (precisionOnly && (oGps->numSats < 128))
    return false;
  
  curLocation.x = oGps->longitude;
  curLocation.y = oGps->latitude;
  res = isInside(perimeter, countCorners(), curLocation); 
  debug(L_DEBUG, (char *) F("virtPerim::checkInside: %d\n"), (int) res);
  return res;
} // boolean virtPerim::inside()

// Set tracking mode for readCourseError() if parameter is true
// (when outside of perimeter - turn right, when inside - left)
// When false - just turn for maximum magnitude signal
void virtPerim::setTracking(boolean tracking) {
  trackingMode = tracking;
  destReached = false;
} // void virtPerim::setTracking(boolean tracking)

// Course correction by perimeter
float virtPerim::readCourseError() {
  point_t pClosest;
  float curDir, moveDir, d;

  if (trackingMode) {
    // Tracking perimeter 
    if (softError() != _hwstatus::ONLINE)
      return 1000;
    // Outside - turn right, inside - turn left
    if (checkInside())
      return -M_PI / 8;
    else
      return M_PI / 8;
  } else {
    // Searching for closest point of the perimeter
    // Stop if we're inside perimeter
    if (!imuSens || inside())
      return -1000;

    // Find closest point at the perimeter
    getPointPerim(pClosest);
    // Calculate direction by IMU
    curDir = imuSens->readCurDegreeRad(0);
    moveDir = gps::bearing(oGps->latitude, oGps->longitude, pClosest.y, pClosest.x);
    d = imu::scalePI(moveDir - curDir);
    debug(L_INFO, (char *) F("virtPerim::readCourseError: %f (%f %f to [%d,%d]\n"),
          d, moveDir, curDir, pClosest.x, pClosest.y);
    // Range check
    if (d < -M_PI)
      d = d + 2 * M_PI;
    else
      if (d > M_PI)
        d = d - 2 * M_PI;

    // Check if we've reached needed direction
    if (!trackingMode && (abs(d) < (M_PI / 20.0f))) {
      destReached = true;
      return -1000;
    }

    return d;
  }
} // float virtPerim::readCourseError()

// Error status if we're out of perimeter too long
_hwstatus virtPerim::softError() {
  if (!enabledPerimeter)
    return _hwstatus::DISABLED;
  if (((millis() - lastInside) / 1000) > errorTimeout)
    return _hwstatus::ERROR;
  return _hwstatus::ONLINE;
} // _hwstatus virtPerim::softError()

// Save perimeter array to nvmem
void virtPerim::savePerimeter() {
  if (!oSave)
    return;

  // Set NVMEM pointer to the array
  oSave->curAddr = nvmemAddr;

  debug(L_NOTICE, (char *) F("Saving virtual perimeter to %04x\n"), oSave->curAddr);
  // Perform save operation
  for (int i = 0; i < _VIRTPERIM_MAX; i++) {
    oSave->writeMem(perimeter[i].x);
    oSave->writeMem(perimeter[i].y);
  }
} // void virtPerim::savePerimeter()

// Load perimeter array from nvmem (if there is any)
void virtPerim::loadPerimeter() {
  int32_t tmp;
  int i;

  if (!oSave)
    return;

  // Set NVMEM pointer to the array
  oSave->curAddr = nvmemAddr;

  debug(L_NOTICE, (char *) F("Loading virtual perimeter from %04x\n"), oSave->curAddr);
  // If we don't have valid perimeter array in nvmem - just don't load it
  if (!oSave->haveValid) {
    for (i = 0; i < _VIRTPERIM_MAX; i++) {
      oSave->readMem(tmp);
      oSave->readMem(tmp);  
    }
    return;
  }

  // Do actual load
  for (i = 0; i < _VIRTPERIM_MAX; i++) {
    oSave->readMem(tmp);
    perimeter[i].x = tmp;
    oSave->readMem(tmp);
    perimeter[i].y = tmp;
    debug(L_NOTICE, (char *) F("virtPerim::loadPerimeter(): point #%d - %d, %d\n"),
          i, perimeter[i].x, perimeter[i].y);
  }
} // void virtPerim::loadPerimeter()

// (re-)Initialization of virtual perimeter (load perimeter array from nvmem)
_status virtPerim::init(nvmem *saveMem) {
  debug(L_INFO, (char *) F("virtPerim::init\n"));

  // Check if we have set oSave already (re-init)
  if (!oSave) {
    oSave = saveMem;
    if (oSave) {
      nvmemAddr = oSave->curAddr;
      loadPerimeter();
    }
  } else {
    oSave->curAddr = nvmemAddr;
    loadPerimeter();
  }
  reportToROS();
  return _status::NOERR;
} // _status virtPerim::init(nvmem *saveMem)


// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
boolean virtPerim::onSegment(point_t p, point_t q, point_t r) {
  if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
      q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
    return true;
  return false;
} // boolean virtPerim::onSegment(point_t p, point_t q, point_t r)

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int virtPerim::orientation(point_t p, point_t q, point_t r) {
  int64_t val = ((int64_t) q.y - (int64_t) p.y) * ((int64_t) r.x - (int64_t) q.x)
                - ((int64_t) q.x - (int64_t) p.x) * ((int64_t) r.y - (int64_t) q.y);

  if (val == 0)
    return 0;  // colinear 
  return (val > 0) ? 1: 2; // clock or counterclock wise 
} // int virtPerim::orientation(point_t p, point_t q, point_t r) 

// The function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
boolean virtPerim::doIntersect(point_t p1, point_t q1, point_t p2, point_t q2) {
  // Find the four orientations needed for general and 
  // special cases 
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);

  // General case 
  if (o1 != o2 && o3 != o4)
    return true;

  // Special Cases 
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
  if (o1 == 0 && onSegment(p1, p2, q1))
    return true;

  // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
  if (o2 == 0 && onSegment(p1, q2, q1))
    return true;

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
  if (o3 == 0 && onSegment(p2, p1, q2))
    return true;

  // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
  if (o4 == 0 && onSegment(p2, q1, q2))
    return true;

  return false; // Doesn't fall in any of the above cases 
} // boolean virtPerim::doIntersect(point_t p1, point_t q1, point_t p2, point_t q2)

// Returns true if the point p lies inside the polygon[] with n vertices 
boolean virtPerim::isInside(point_t polygon[], int n, point_t p) {
  point_t extreme = {INF, p.y};

  // There must be at least 3 vertices in polygon[] 
  if (n < 3)
    return false;

  // Create a point for line segment from p to infinite 

  // Count intersections of the above line with sides of polygon 
  int count = 0, i = 0;
  do {
    int next = (i+1)%n;

    // Check if the line segment from 'p' to 'extreme' intersects 
    // with the line segment from 'polygon[i]' to 'polygon[next]' 
    if (doIntersect(polygon[i], polygon[next], p, extreme))
    {
      // If the point 'p' is colinear with line segment 'i-next', 
      // then check if it lies on segment. If it lies, return true, 
      // otherwise false 
      if (orientation(polygon[i], p, polygon[next]) == 0)
        return onSegment(polygon[i], p, polygon[next]);

      count++;
    }
    i = next;
  } while (i != 0);

  // Return true if count is odd, false otherwise 
  return count&1;  // Same as (count%2 == 1) 
} // boolean virtPerim::isInside(point_t polygon[], int n, point_t p)

// Find closest point to the line
void virtPerim::getPointLine(point_t l1, point_t l2, point_t pCur, point_t &pClosest) {
  float m1, m2, t1, t2;

  // Check for straight lines
  if (l1.x == l2.x) {
    pClosest.x = l1.x;
    pClosest.y = pCur.y;
    return;
  }
  if (l1.y == l2.y) {
    pClosest.x = pCur.x;
    pClosest.y = l1.y;
    return;
  }
  m1 = (float) (l2.y - l1.y) / (float) (l2.x - l1.x);
  m2 = -1 / m1;
  t1 = (float) l1.y - m1 * (float) l1.x;
  t2 = (float) pCur.y - m2 * (float) pCur.x;
  pClosest.x = (t2 - t1) / (m1 - m2);
  pClosest.y = m1 * (float) pClosest.x + t1;
  
  // Check if we've got point on the segment
  if (onSegment(l1, pClosest,l2))
    return;
  // Not on segment, just choose closest corner
  if (oGps->calcDist(l1.y, l1.x) > oGps->calcDist(l2.y, l2.x)) {
    pClosest.x = l2.x;
    pClosest.y = l2.y;
  } else {
    pClosest.x = l1.x;
    pClosest.y = l1.y;
  }
} // void virtPerim::getPointLine(point_t l1, point_t l2, point_t pCur, point_t &pClosestX)

// Find closest point to the perimeter (check all perimeter's lines)
void virtPerim::getPointPerim(point_t &pClosest) {
  int nCorners = countCorners();
  point_t pCur, pTmp;
  float dist, minDist;

  pClosest.x = perimeter[0].x;
  pClosest.y = perimeter[0].y;

  if (!oGps)
    return;
  
  pCur.x = oGps->longitude;
  pCur.y = oGps->latitude;
  minDist = 99999999;

  // Check all lines in the perimeter to find closest point
  for (int i = 0; i < nCorners; i++) {
    getPointLine(perimeter[i], perimeter[(i + 1) % nCorners], pCur, pTmp);
    dist = oGps->calcDist(pTmp.y, pTmp.x);
    if (dist < minDist) {
      minDist = dist;
      pClosest.x = pTmp.x;
      pClosest.y = pTmp.y;
    }
  }
} // void virtPerim::getPointPerim(point_t &pClosest)

// Force report status to ROS
void virtPerim::reportToROS() {
#ifdef USE_ROS
  oROS.reportToROS(reportSensor::VIRTPERIM, (uint8_t *) &currentState, 1);
#endif
} // void virtPerim::reportToROS()
