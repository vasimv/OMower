// Support code for virtual seeker sensor (camera on
// Rapsberry PI zero, searching for chessboard and sending
// its data through pfodApp/modbus interface)
// $Id$

#ifndef _SEEKER_OMOWER_H
#define _SEEKER_OMOWER_H

#include <omower-defs.h>
#include <omower-root.h>
#include <omower-imu.h>
#include <stdint.h>

class seeker : public navThing {
public:
  // Constructor
  seeker();

  _hwstatus begin();

  // Software init
  _status init();

  // Settings variables
  // Timeout of data receive, in ms
  uint16_t maxTimeout;

  // Offset angle (how far chessboard from center of camera)
  int16_t offsetAngle;
  // Chessboard's orientation angle (not working correctly yet)
  int16_t orientAngle;
  // Distance to the chessboard in cm (if 0 - no chessboard visible)
  uint16_t distance;

  // For IMU's guided mode (when robot drives by compass with offset from seeker) it must set to imu object
  // When NULL - robot will use only seeker data to move to the chessboard
  imu *imuSens;
  
  // Set tracking mode for readCourseError(), will stop the robot
  // at minDistance (centimeters). If fixOrientation is true - it will
  // move in curve pattern to approach the board from face, not side (not working yet)
  void startSeeker(boolean fixOrientation, uint16_t minDistance);

  // Course correction by offsetAngle (todo: use orientation angle too)
  float readCourseError();

  // Error status if no chessboard visible
  _hwstatus softError();

  // Set angles and distance from raspberry pi (via pfodApp or modbus interfaces)
  void setSeekerData(int16_t extOffsetAngle, int16_t extOrientAngle, uint16_t extDistance, uint16_t ageMeasure);

  // Must be called 20 times per second
  void poll20();

  // Force report to ROS
  void reportToROS();

private:
  // timestamp of last data
  uint32_t lastReceived;

  // How close we should be to the chessboard (stops when reached this)
  uint16_t distanceStop;

  // flag for orientation fix curve movement
  boolean flagFixOrientation;

  // For IMU's guided mode - absolute direction calculated when seeker sends data
  volatile float absDir;

  // Array of old compass readings for IMU's guided mode
  volatile int16_t agedCompass[5];

  // time of last update agedCompass
  volatile uint32_t lastAged;
};

#endif
