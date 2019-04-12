// ROS support for OMower
// $Id$

#ifndef _OMOWER_ROS_H
#define _OMOWER_ROS_H

#include <omower-defs.h>
// oROS will be empty stub if no USE_ROS defined!

#include <omower-root.h>
#include <omower-serial.h>
#include <stdint.h>

// How often we do force report (one sensor at time!) in milliseconds
#define DELAY_FORCE_REPORT 20

#ifdef USE_ROS
#include <ros.h>

// Hardware object for rosserial
class omowerROSHardware {
public:
  omowerROSHardware();

  void init();

  // Read a byte from serial port, -1 if none
  int read();

  // Write data to serial port
  void write(uint8_t *data, int length);

  // Returns milliseconds from start of the program
  unsigned long time();
};

namespace ros {
  typedef ros::NodeHandle_<omowerROSHardware, 50, 50, 2048, 2048> omowerNodeHandle;
}
#endif

// Sensor types for ROS publishers (other modules will use these to mark ones need to report change)
enum class reportSensor : uint8_t { DEBUG = 0, BUMPER, BUTTONS, MOTORPWM, MOTORCURRENT, MOTORPID,
                                  MOWPWM, MOWCURRENT, DROP, GPS, ORIENTATION, LAWN,
                                  ODOMETRY, PERIMETER, VOLTAGE, CURRENT, RADIOTAG,
                                  RAIN, RTCCLOCK, SEEKER, SONARS, VIRTPERIM, PWMSERVO,
                                  // LAST_VALUE must be last in the enum!
                                  LAST_VALUE, FIRST_VALUE = DEBUG };

extern serial oSerial;

class omowerROS {
public:
  // Constructor
  omowerROS();

  // Set serial port for ROS
  void setSerialPort(numThing n);

  // Advertise and subscribe to/on topics
  void advertiseSubscribe();

  // Must be called in loop(), internal ROS's stuff and reporting changed sensors
  void spinOnce();

  // Send message to CMD channel
  void publishCmd(uint8_t *msg, uint16_t length);

  // Check if there is any command received through ROS (returns NULL if no any)
  uint16_t checkCmd(uint8_t **pCmd);

  // Mark a sensor to publish its changed status through ROS
  void reportToROS(reportSensor sensor, uint8_t *data, uint8_t num);

  // Set callback for /move_base_simple/goal (robot will move to the point by itself, without /cmd_vel commands)
  // Parameters of the callback - x,y,z,qX,qY,qZ,qW
  // If not set or NULL - will ignore this topic
  void setCallBackMove(void (*pfMove)(float, float, float, float, float, float, float));

  // Set callback for /current_position (coordinates in UTM for omower-gps module)
  // Parameters of the callback - x,y,z,qX,qY,qZ,qW
  // If not set or NULL - will ignore this topic
  void setCallBackCurPose(void (*pfCurPose)(float, float, float, float, float, float, float));

  // Set callback for /cmd_vel (lX,lY,lZ,aX,aY,aZ)
  // If not set or NULL - will ignore the topic
  void setCallBackCmdVel(void (*pfCmdVel)(float, float, float, float, float, float));

  // Temporarily stop
  void pause();

  // Continue after pause()
  void unpause();

private:
  // Current sensor to force report
  uint8_t currentSensor;

  // Time of last force report
  uint32_t lastForceReport;

  // Temporarily disabled
  boolean paused;

  // Publish information about sensor
  void publishSensor(reportSensor sensor, uint8_t *data, uint8_t num);
};

extern omowerROS oROS;

#endif
