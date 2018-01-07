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
  if (debugBusy || (level < debugLevel)) {
    va_start(args, fmt);
    va_end(args);
    return;
  }
  debugBusy = true;
  va_start(args, fmt);
  nlen = rpl_vsnprintf(debugOutBuf, sizeof(debugOutBuf), fmt, args);
  va_end(args);

  // Non-interrupt output to serial port
  UART->UART_IDR = UART_IDR_TXRDY;
  UART->UART_CR |= UART_CR_TXEN;
  for (uint16_t i = 0; i < nlen; i++) {
    // Wait for ready to transmit
    while (!(UART->UART_SR & UART_SR_TXRDY));
    // Send next byte
    UART->UART_THR = debugOutBuf[i];
  }
  debugBusy = false;
} // void debug(uint8_t log_level, const char *fmt, ...)

