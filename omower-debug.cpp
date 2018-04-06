// Debugging functions for Omower
// $Id$

#include <Arduino.h>
#include <omower-defs.h>
#include <omower-debug.h>
#include <stdarg.h>
#include <stdio.h>
#include <xsystem.h>

va_list args;
int nlen;
char debugOutBuf[256];

volatile boolean debugBusy = false;

uint8_t debugLevel = L_DEBUG;

void debug(uint8_t level, const char *fmt, ...) {
  // Don't send debug output if the port is busy in main/interrupt thread
  boolean dontSend = false;
  uint32_t ticks = 0;
  
  // Check if something is sending on the port already (see omower-serial)
  if (debugBusy || (level < debugLevel)) 
    dontSend = true;
  else
    debugBusy = true;
  va_start(args, fmt);
  if (!dontSend)
    nlen = rpl_vsnprintf(debugOutBuf, sizeof(debugOutBuf), fmt, args);
  va_end(args);

  if (dontSend)
    return;
  // Non-interrupt output to serial port
  UART->UART_IDR = UART_IDR_TXRDY;
  UART->UART_CR |= UART_CR_TXEN;
  for (uint16_t i = 0; i < nlen; i++) {
    // Wait for ready to transmit
    while (!(UART->UART_SR & UART_SR_TXRDY)) {
      ticks++;
      if (ticks > 0x7fff)
        break;
    }
    // Send next byte
    UART->UART_THR = debugOutBuf[i];
  }
  debugBusy = false;
} // void debug(uint8_t log_level, const char *fmt, ...)

