// DW1000 tag support or OMower
// $Id$

#include <omower-radiotag.h>
#include <Arduino.h>
#include <omower-debug.h>
#include <omower-modbus.h>
#include <string.h>

// Hardware initialization
_hwstatus radiotag::begin() {
  passGps = NULL;
  imuSens = NULL;
  serialPorts = NULL;
  tagPort = -1;
  maxTimeout = 400;
  return _hwstatus::ONLINE;
} // _hwstatus radiotag::begin() {

// Software (re)initialization
_status radiotag::init() {
  validCoords = false;
  lastReceived = 0;
  cntBuf = 0;
  return _status::NOERR;
} // _status radiotag::init()

_hwstatus radiotag::softError() {
  if ((millis() - lastReceived) > 120000)
    return _hwstatus::ERROR;
  return _hwstatus::ONLINE;
} // _hwstatus radiotag::softError()

// 10 times per second poll (check if coordinates are still valid)
void radiotag::poll10() {
  boolean found = false;
  uint8_t i;

  // Try to read modbus packets from serial port
  if (serialPorts && (tagPort > 0)) {
    while (serialPorts->available(tagPort)) {
      bufRead[cntBuf] = serialPorts->read(tagPort);
      cntBuf++;
      // Reset buffer if we're reading too much nonsense
      if (cntBuf >= sizeof(bufRead))
        cntBuf = 0;
    }
    // Try to find radiotag modbus packet in the buffer
    if (cntBuf > 0) {
      for (i = 0; i < cntBuf; i++) {
        if ((bufRead[i] == 0x20) && (bufRead[i + 1] == 0x03) && (bufRead[i + 2] == 0x14)) {
          found = true;
          break;
        }
      }
      if (found) {
        // Move packet to start of buffer
        if ((i > 0) && (i < cntBuf)) {
          memmove((void *) bufRead, (void *) (bufRead + i), cntBuf - i);
          cntBuf = cntBuf - i;
        }
        uint8_t *pkt = bufRead;

        // Radiotag's modbus output packet is always 25 bytes length
        if (cntBuf >= 25) {
          if (modbusCrc(pkt, 23) == getModbus16(pkt + 23)) {
            // We've found valid CRC in the packet, read coordinates
            coordRoom = (uint16_t) getModbus16(pkt + 5);
            coordX = (int32_t) getModbus32(pkt + 9);
            coordY = (int32_t) getModbus32(pkt + 13);
            validCoords = true;
            lastReceived = millis();
            debug(L_DEBUG, (char *) F("radiotag::poll10: received coords %ld %ld\n"), coordX, coordY);
            // Send coordinates to the GPS module in pass-through mode
            if (passGps != NULL)
              passGps->setCoords(coordY / KCORR_CM_TO_COORD, coordX / KCORR_CM_TO_COORD, 129, 0, 0, 0, 0, 0, 0);
          }
          // Remove packet from the buffer
          if (cntBuf > 25) {
            memmove((void *) bufRead, (void *) (bufRead + 25), cntBuf - 25);
            cntBuf = cntBuf - 25;
          } else
            cntBuf = 0;
        }
      }
    }
  }

  // Check if our coordinates are still valid
  if ((millis() - lastReceived) > maxTimeout)
    validCoords = false;
} // void radiotag::poll10()

// Bearing to the point (-M_PI..M_PI)
float radiotag::bearing(int32_t X1, int32_t Y1, int32_t X2, int32_t Y2) {
  return atan2(X2 - X1, Y2 - Y1);
} // float radiotag::bearing(int32_t X1, int32_t Y1, int32_t X2, int32_t Y2)

float radiotag::distance(int32_t X1, int32_t Y1, int32_t X2, int32_t Y2) {
  float dLat = (float)(Y2 - Y1);
  float dLon = (float)(X2 - X1);

  return sqrt(dLat*dLat + dLon*dLon) * 0.01f;
} // float radiotag::distance(int32_t X1, int32_t Y1, int32_t X2, int32_t Y2)

// Calculate distance to the current position
float radiotag::calcDist(int32_t destX, int32_t destY) {
  return distance(coordX, coordY, destX, destY);
} // float radiotag::calcDist(int32_t destX, int32_t destY)

// Sets coordinates for course correction
void radiotag::setTarget(int32_t destX, int32_t destY) {
  targX = destX;
  targY = destY;
} // void radiotag::setTarget(int32_t destX, int32_t destY)

// Course error for navigation
float radiotag::readCourseError() {
  int16_t degreeCur = degree;
  float degreeCurF, degreeDestF;

  // Stop if we've reached the destination
  if (abs(distance(coordX, coordY, targX, targY)) <= 0.1)
    return -1000;
  // Temporary stop if no valid coordinates
  if (!validCoords);
    return 1000;

  // Get course from IMU if available
  if (imuSens != NULL)
    degreeCur = imuSens->readCurDegree(0);

  // Calculate course correction
  degreeCurF = imu::degreePI(degreeCur);
  degreeDestF = bearing(coordX, coordY, targX, targY);

  float d = imu::scalePI(degreeDestF - degreeCurF);

  // Range check
  if (d < -M_PI)
    d = d + 2 * M_PI;
  else
    if (d > M_PI)
      d = d - 2 * M_PI;
  return d;
} // float radiotag::readCourseError()

// Set coordinates from external source
void radiotag::setCoords(int32_t coordX, int32_t coordY) {
  radiotag::coordX = coordX;
  radiotag::coordY = coordY;
  validCoords = true;
  lastReceived = millis();
} // void radiotag::setCoords(int32_t coordX, int32_t coordY)
