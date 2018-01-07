// OMower enums definitions
// $Id$

#ifndef _OMOWER_ENUMS_H
#define _OMOWER_ENUMS_H

#include <stdint.h>

// Function result codes
enum class _status : int8_t {ERR = 0, NOERR = 1};   

// Device status, ERROR - unrecoverable hardware error (repair needed)
enum class _hwstatus : int8_t {DISABLED = 0, ERROR = -1, ONLINE = 1};

// Direction of movement/roll
enum class _dir : int8_t { BACKWARD = -2, LEFT = -1, STOP = 0, RIGHT = 1, FORWARD = 2};                           

// Number of device, -1 - for all devices of same class
typedef int8_t numThing;

// Devices placement variants (numeration from zero)
enum class _locationThings : uint8_t {JUSTONE = 0, FORW_BACK = 1, LEFT_RIGHT = 2,
                              LEFT_CENTER_RIGHT = 3,
                              LEFTFORW_RIGHTFORW_LEFTBACK_RIGHTBACK = 4,
                              SPECIAL = 5};

// Invalid angle
#define INVALID_ANGLE ((float) -999)
// Invalid coordinates
#define INVALID_COORD ((int32_t) 2000000000L)
// Maximum speed for motor drivers (2^8 - 1)
#define SPEED_MAX 255
#endif
