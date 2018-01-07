// Serial UART/USARTs support class, no-interrupt sending
// $Id$

#ifndef _SERIAL_OMOWER_H
#define _SERIAL_OMOWER_H

#include <omower-root.h>
#include <Arduino.h>
#include <stdarg.h>
#include <stdint.h>

#define CONSOLE_PORT 0
#define SERIAL1_PORT 1
#define SERIAL2_PORT 2
#define SERIAL3_PORT 3

class serial : public thing {
public:
  // hardware initialization
  _hwstatus begin();

  // set baud for serial interface
  void setBaud(numThing n, uint32_t baud);

  // Receive character from serial port n (n = 0 -> debug console), blocking!
  unsigned char read(numThing n);

  // Check if there are byte(s) in the receive buffer
  boolean available(numThing n);

  // Write bytes to the serial port, returns number of bytes
  // sent before timeout (in milliseconds, 0 - infinity)
  uint16_t write(numThing n, unsigned char *buf, uint16_t len, uint16_t timeout = 0);

  // Write formatted string to the console
  // (it uses its own formatting library, you can use floats too)
  // it will return after all chars successful sent only
  void printf(numThing n, char *fmt, ...);

  // Returns number of devices of the class
  numThing numThings();

  // Returns placement of devices
  _locationThings locThings() { return _locationThings::SPECIAL; }
};
#endif
