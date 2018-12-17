// GPS sensor for OMower
// $Id$

#ifndef _GPS_OMOWER_H
#define _GPS_OMOWER_H

#include <omower-root.h>
#include <omower-imu.h>
#include <omower-odometry.h>
#include <stdint.h>
#include <TinyGPS++.h>

// koefficient for centimeters to coordinates change
#define KCORR_CM_TO_COORD 1.11318845f

// Distance to next target point in split way mode (meters)
#define SPLIT_DISTANCE 3.0f

// Compass autocorrection - it'll calculate compass offset after driving half of distance to target point and compensate
// All direction are grouped in 4 sectors, 90 degrees each

// Disable the autocorrection
// #define DISABLE_COMPASS_AUTOCORRECTION

// All direction are grouped in 4 sectors, 90 degrees each
#define AUTOCORR_SECTORS 4
// Sector's angle, in radians
#define AUTOCORR_SECTOR_ANGLE ((M_PI * 2) / (AUTOCORR_SECTORS - 1))

class gps : public navThing {
public:
  // Settings variables:
  // Timeout of data receive, in ms
  uint16_t maxTimeout;
  // For precision GPS (RTK GPS) only!
  // Polar coordinates of the GPS antenna of robot's center
  // Angle (in degrees)
  int16_t angleCenter;
  // Distance (in cm)
  int16_t distCenter;
  // Stop when within meters from given coordinates (non-precision mode)
  float stopDistance;
  // Stop when within meters from given coordinates (precision mode)
  float stopDistancePrecision;

  // Number of satellites, if > 128 - precision data received from RTKLIB
  // 0 - no coordinates received (coordinates, speed, degree and date/time must be ignored)
  uint8_t numSats;
  // Current coordinates (all coordinates in integer form of +/-DDDMMMMMMM)
  int32_t latitude;
  int32_t longitude;
  // Raw coordinates from GPS (without odometry correction but with correction of antenna position)
  int32_t latitudeRaw;
  int32_t longitudeRaw;

  // Offsets for all sectors
  float offsetSectors[AUTOCORR_SECTORS];

  // Approximate speed (in cm per second)
  uint16_t speed;

  // Approximate course in degrees
  int16_t degree;

  // Date/time received from GPS
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t hour;
  uint16_t minute;
  uint16_t second;

  // IMU object for GPS navigation (course correction)
  // if set to NULL (default) - will use GPS course (very rough)
  imu *imuSens;

  // Odometer object for precision correction
  odometryMotors *odoSens;

  // Current target coordinates
  int32_t latitudeTarg, longitudeTarg;

  // Constructor (var init)
  gps();

  // Initialization
  _status init();
 
  // returns soft error status if too long time without GPS data
  _hwstatus softError();

  // Sets coordinates for course correction
  // if precisionOnly set - stops when lost precision data
  // if splitWay set - the robot will try to move precisely over line
  //  between start and end point (splitting the line to set of small lines),
  //  otherwise - it will try to reach the destination just (ignoring slipping
  //  on turns and such)
  void setTarget(int32_t latitudeDest, int32_t longitudeDest, boolean precisionOnly, boolean splitWay);

  // Course error for PID controller
  float readCourseError();

  // set GPS coordinates and time from external sources (ignore date/time if year == 0)
  void setCoords(int32_t latitudeNew, int32_t longitudeNew, uint8_t numSatsNew,
                 uint16_t yearNew, uint16_t monthNew, uint16_t dayNew,
                 uint16_t hoursNew, uint16_t minuteNew, uint16_t secondNew);
  
  // Parse received GPS string(s)
  void parseString(char *buf, uint16_t len);

  // Calculate distance to coordinates (in meters)
  float calcDist(int32_t latitude, int32_t longitude);

  // Course between points
  static float bearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);

  // Distance between points
  static float distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);

  // Calculate coordinates of point on line to the target with current
  // coordinates and distance from it
  void calcPoint(float dist, int32_t &latitudeP, int32_t &longitudeP,
                  int32_t latitudeDest, int32_t longitudeDest);

  void poll10();

  // Force report coordinates to ROS
  void reportToROS();

private:
  // TinyGPS++ object
  TinyGPSPlus tinygps;

  // Last timestamp of data received
  unsigned long lastReceived;

  // last timestamp of precision data received
  unsigned long lastPrecision;

  // Distance to the target at previous cycle
  float lastDist;

  // Convert tinygps's RawDegrees to int32_t
  int32_t coordToInt(const RawDegrees &degree);

  // Last target coordinates and precision flag
  int32_t latitudeLast, longitudeLast;
  boolean precisionTarg;

  // Odometry ticks at last coordinates receive
  int32_t leftTicks, rightTicks;

  // Autocorrection variables:
  // Proposed direction of the movement
  float startDir;

  // Coordinates of the start point
  int32_t latS, lonS;

  // Flag variables for compass autocorrection
  boolean startAutocorrection, endAutocorrection, flagAutocorrection;

  // Correct precision coordinates by antenna's position on the robot
  void correctCoords();

  // Reset odometry stats for precision correction
  void resetOdometryTicks();

  // Autocorrection functions
  // Initialise autocorrection calculation (must be called at start of measured movement)
  // latStart, lonStart - coordinates of robots at start of movement
  // latEnd, lonEnd - coordinates of destination point
  void initAutocorr(int32_t latStart, int32_t lonStart, int32_t latEnd, int32_t lonEnd);

  // Calculate compass autocorrection offset (must be called at middle of measured movement)
  // lat, lon - current robot's coordinates
  // Updates offsetSectors array
  void calcOffset(int32_t lat, int32_t lon);

  // Calculate autocorrection sector from angle
  uint8_t calcSector(float angle);
};
#endif
