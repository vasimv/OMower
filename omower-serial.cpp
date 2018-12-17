// Serial UART/USARTs support class for ATSAM3X8E
// $Id$

#include <omower-defs.h>
#include <omower-serial.h>
#include <omower-debug.h>
#include <stdarg.h>
#include <stdio.h>
#include <xsystem.h>

// Returns true if there are chars in receive buffer
boolean serial::available(numThing n) {
  switch (n) {
    case 0:
      return consAvailable();
    case 1:
      return Serial1.available();
    case 2:
      return Serial2.available();
    case 3:
      return Serial3.available();
    default:
      return false;
  }
} // boolean serial::available(numThing n)

// read a char from the serial port (blocking!)
unsigned char serial::read(numThing n) {
  switch (n) {
    case 0:
      unsigned char tmp;
      while (consAvailable() == 0);
      consRead(&tmp, 1);
      return tmp;
    case 1:
      return (unsigned char) Serial1.read();
    case 2:
      return (unsigned char) Serial2.read();
    case 3:
      return (unsigned char) Serial3.read();
    default:
      return 0;
  }
} // unsigned char serial::read(numThing n)

// set baud for the serial port
void serial::setBaud(numThing n, uint32_t baud) {
  switch (n) {
    case 0:
      consInit(baud);
      return;
    case 1:
      Serial1.begin(baud);
      USART0->US_IDR = US_IDR_TXRDY;
      return;
    case 2:
      Serial2.begin(baud);
      USART1->US_IDR = US_IDR_TXRDY;
      return;
    case 3:
      Serial3.begin(baud);
      USART2->US_IDR = US_IDR_TXRDY;
      return;
    default:
      return;
  }
} // void serial::setBaud(numThing n, uint32_t baud)

// Write bytes to the serial port, returns number of bytes
// sent before timeout (in milliseconds, 0 - infinity)
uint16_t serial::write(numThing n, unsigned char *buf, uint16_t len, uint16_t timeout) {
  uint32_t tStart = millis();
  uint16_t sent = 0;
  int32_t tTimeout = timeout;
  Usart *port;

  // Zero timeout specified
  if (tTimeout == 0)
    tTimeout = 99999;

  // Serial console is UART, working through separate interface
  if (n == 0) {
    uint16_t wrote;

#ifndef USE_ROS
    debugDisable = true;
#endif
    wrote = consWrite(buf, len);
    sent = wrote;
    if (wrote != len) {
      while (sent != len) {
        while (((millis() - tStart) < tTimeout) && !consSendBusy());
        if ((millis() - tStart) >= tTimeout)
          break;
        wrote = consWrite(buf + sent, len - sent);
        sent += wrote;
      }
    }
#ifndef USE_ROS
    debugDisable = false;
#endif
    return sent;
  }

  // USARTs send
  switch (n) {
    case 1:
      port = USART0;
      break;
    case 2:
      port = USART1;
      break;
    case 3:
      port = USART2;
      break;
    default:
      return 0;
  }
  port->US_IDR = US_IDR_TXRDY;
  for (sent = 0; sent < len; sent++) {
    // Wait for ready to transmit
    while ((port->US_CSR & US_CSR_TXRDY) != US_CSR_TXRDY) {
      if ((millis() - tStart) >= tTimeout)
        return sent;
    }
    // Send next byte
    port->US_THR = buf[sent];
  }
  return sent;
} // uint16_t serial::write(unsigned char *buf, uint16_t len, uint16_t timeout)

// Write formatted string to the console
// (it uses its own formatting library, you can use floats too)
// it will return after all chars successful sent only
void serial::printf(numThing n, char *fmt, ...) {
  va_list args;
  int len;
  unsigned char buf[256];

  va_start(args, fmt);
  len = rpl_vsnprintf((char *) buf, sizeof(buf), fmt, args);
  if (len < 0)
    len = strlen((char *) buf);
  va_end(args);
  write(n, buf, len);
} // void serial::printf(numThing n, char *fmt, ...)

// Returns number of devices of the class
numThing serial::numThings() {
  return 4;
} // numThing serial::numThings()

// Hardware init
_hwstatus serial::begin() {
  debug(L_DEBUG, (char *) F("serial::begin\n"));
  NVIC_DisableIRQ(UART_IRQn);
  NVIC_SetPriority(USART0_IRQn, 3);
  NVIC_SetPriority(USART1_IRQn, 3);
  NVIC_SetPriority(USART2_IRQn, 3);
  return _hwstatus::ONLINE;
} // _hwstatus serial::begin()
