// Virtual perimeter sensor (uses GPS and set of corner points)
// based on code from https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
// $Id$

#ifndef _VIRTPERIM_OMOWER_H
#define _VIRTPERIM_OMOWER_H

#include <omower-root.h>
#include <omower-defs.h>
#include <stdint.h>
#include <Arduino.h>
#include <omower-gps.h>
#include <omower-imu.h>

// Maximum size of polygon array (how many corners)
#define _VIRTPERIM_MAX 20

typedef struct {
  int32_t x, y;
} point_t;

class virtPerim : public navThing {
public:
  // Perimeter filtering koefficient
  float filterK;

  // Time to generate soft error when we're out of perimeter too long (in seconds)
  uint16_t errorTimeout;

  // Work only if high precision coordinates are available
  boolean precisionOnly;

  // Pointer to the GPS object
  gps *oGps;

  // Pointer to the IMU object
  imu *imuSens;

  // "Hardware" initialization (once, resets variables)
  _hwstatus begin();

  // Software (re)init (load perimeter array from nvmem if there is any
  _status init(nvmem *saveMem);

  // Load perimeter array from nvmem
  void loadPerimeter();

  // Save perimeter array in nvmem
  void savePerimeter();

  // Disable/enable perimeter
  _status enableThings();
  _status disableThings();

  // If we're inside of perimeter
  boolean inside();

  // Returns true if status is unknown (must temporarily stop!)
  boolean unknown();

  // Set tracking mode for readCourseError() if parameter is true
  // (when outside of perimeter - turn right, when inside - left)
  // When false - just turn for closest point
  void setTracking(boolean tracking);

  // Course correction by maximum signal strength
  float readCourseError();

  // Error status if outside of perimeter too long or virtual perimeter is disabled
  _hwstatus softError();

  // Perimeter array operations (append/delete/count)
  int countCorners();
  void delCorner(int n);
  void appendCorner(int32_t latitude, int32_t longitude);
  void getCorner(int n, int32_t &latitude, int32_t &longitude);
  void setCorner(int n, int32_t latitude, int32_t longitude);

  // Constructor
  virtPerim();

  // Must be called 10 times per second (perimeter timeout)
  void poll10();

private:

  // pointer to NVMEM and array address
  nvmem *oSave;
  uint32_t nvmemAddr;

  // Virtual perimeter array (first point with coordinates (INVALID_COORD,INVALID_COORD) is end of the array
  point_t perimeter[_VIRTPERIM_MAX];

  // Enabled or disabled status
  boolean enabledPerimeter;

  // When we were inside perimeter last time (for softEror())
  uint32_t lastInside;

  // Tracking mode
  boolean trackingMode;

  // current state (inside/outside)
  boolean currentState;

  // filtered number of inside/outside
  float statsInside;

  // Given three colinear points p, q, r, the function checks if 
  // point q lies on line segment 'pr' 
  boolean onSegment(point_t p, point_t q, point_t r);

  // To find orientation of ordered triplet (p, q, r). 
  // The function returns following values 
  // 0 --> p, q and r are colinear 
  // 1 --> Clockwise 
  // 2 --> Counterclockwise 
  int orientation(point_t p, point_t q, point_t r);

  // The function that returns true if line segment 'p1q1' 
  // and 'p2q2' intersect. 
  boolean doIntersect(point_t p1, point_t q1, point_t p2, point_t q2);

  // Returns true if the point p lies inside the polygon[] with n vertices 
  boolean isInside(point_t polygon[], int n, point_t p);

  // Internal check (called from poll10())
  boolean checkInside();

  // Find closest point to the perimeter (check all perimeter's lines)
  void getPointPerim(point_t &pClosest);

  // Find closest point to the line
  void getPointLine(point_t l1, point_t l2, point_t pCur, point_t &pClosest);
};

#endif
