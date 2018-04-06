// DW1000 positioning sensor support for OMower
// $Id$

#ifndef _RADIOTAG_OMOWER_H
#define _RADIOTAG_OMOWER_H

#include <omower-root.h>
#include <omower-imu.h>
#include <omower-serial.h>
#include <stdint.h>

class radiotag: public navThing {
public:
  // Settings variables:
  // Timeout of data receive, in ms
  uint16_t maxTimeout;

  // Valid coordinates flag (true - coordinates are valid)
  boolean validCoords;
  // Current coordinates (all coordinates in centimeters, axis Y - north-south)
  int32_t coordX;
  int32_t coordY;
  uint16_t coordRoom;

  // Approximate speed (in cm per second)
  uint16_t speed;

  // Approximate course in degrees
  int16_t degree;

  // IMU object for navigation (course correction)
  // if set to NULL (default) - will use radiotag's calculated course (very rough)
  imu *imuSens;

  // Serial object for reading messages from radiotag
  serial *serialPorts;

  // Number of serial port radiotag connected to
  numThing tagPort;

  // Initialization
  _status init();
 
  // returns soft error status if too long time without GPS data
  _hwstatus softError();

  // Sets coordinates for course correction
  // if precisionOnly set - stops when lost precision data
  void setTarget(int32_t destX, int32_t destY);

  // Course error for PID controller
  float readCourseError();

  // set coordinates from external sources
  void setCoords(int32_t coordX, int32_t coordY);

  // Calculate distance to current coordinates (in meters)
  float calcDist(int32_t destX, int32_t destY);

  void poll10();

private:
  // Read buffer
  uint8_t bufRead[128];
  uint8_t cntBuf;

  // Last timestamp of data received
  unsigned long lastReceived;

  // target coordinates and precision flag
  int32_t targX, targY;

  // Course between points
  float bearing(int32_t X1, int32_t Y1, int32_t X2, int32_t Y2);

  // Distance between points
  float distance(int32_t X1, int32_t Y1, int32_t X2, int32_t Y2);
};
#endif
