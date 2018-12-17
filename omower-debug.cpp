// Debugging functions for Omower
// $Id$

#include <Arduino.h>
#include <omower-defs.h>
#include <omower-debug.h>
#include <stdarg.h>
#include <stdio.h>
#include <xsystem.h>
#include <omower-ros.h>


volatile boolean debugDisable = false;
volatile boolean debugInProgress = false;

uint8_t debugLevel = L_DEBUG;

uint8_t consSBuf[_WRITE_BUFFER_SIZE];
uint8_t consRBuf[_READ_BUFFER_SIZE];
#ifdef USE_ROS
// We need bigger buffer for debugging logs in ROS mode
char debugOutBuf[1560];
uint16_t cntDebugOut = 0;
#else
char debugOutBuf[256];
#endif

void debug(uint8_t level, const char *fmt, ...) {
  va_list args;
  int nlen, freeBytes;

  // Don't send debug output if the port is busy in main/interrupt thread
  if (debugDisable || debugInProgress || (level < debugLevel))
    return;
  debugInProgress = true;
  
  // Check if something is sending on the port already (see omower-serial)
  va_start(args, fmt);
  freeBytes = sizeof(debugOutBuf) - 1 - cntDebugOut;
  if (freeBytes > 1) {
    nlen = rpl_vsnprintf(debugOutBuf + cntDebugOut, freeBytes, fmt, args);
    if (nlen > 0) {
      cntDebugOut += nlen;
      if (cntDebugOut > (sizeof(debugOutBuf) - 1))
        cntDebugOut = sizeof(debugOutBuf) - 1;
    }
  }
  va_end(args);

#if !defined(USE_ROS) || defined(DEBUG_ON_CONSOLE)
  // Check if sending is blocked or we don't have enough space in buffer
  if ((UART->UART_TCR + UART->UART_TNCR + cntDebugOut) < sizeof(consSBuf))
    consWrite((uint8_t *) debugOutBuf, cntDebugOut);
  cntDebugOut = 0;
#else
  debugOutBuf[cntDebugOut] = '\0';
  oROS.reportToROS(reportSensor::DEBUG, (uint8_t *) debugOutBuf, cntDebugOut);
#endif
  debugInProgress = false;
} // void debug(uint8_t log_level, const char *fmt, ...)

// Resets debug output buffer
void resetDebugOutput() {
  debugOutBuf[0] = '\0';
  cntDebugOut = 0;
} // void resetDebugOutput()

// Returns number of available characters in receive buffer
uint16_t consAvailable() {
  uint32_t status = UART->UART_SR;

  // Reset error flags if any
  if ((status & UART_SR_OVRE) == UART_SR_OVRE || (status & UART_SR_FRAME) == UART_SR_FRAME)
    UART->UART_CR |= UART_CR_RSTSTA;

  return sizeof(consRBuf) - UART->UART_RCR;
} //uint16_t consAvailable()

uint8_t *_pRecv;
volatile boolean _consDisable;

// Re-init receive PDC transfer registers
void _recvReInit() {
  UART->UART_RPR = (uint32_t) consRBuf;
  UART->UART_RCR = sizeof(consRBuf);
  _pRecv = consRBuf;
} // void _recvReInit()

// Non-blocking read of receive buffer (returns number of characters read)
uint16_t consRead(uint8_t *buf, uint16_t length) {
  uint32_t count;
  uint16_t res;

  // Disable PDC
  UART->UART_PTCR = UART_PTCR_RXTDIS;
  // Wait for PDC disable
  while (UART->UART_PTSR & UART_PTSR_RXTEN);

  count = (uint8_t *) UART->UART_RPR - _pRecv;
  if (count <= length) {
    if (count != 0) {
      memcpy(buf, _pRecv, count);
      _recvReInit();
    }
    res = count;
  } else {
    memcpy(buf, _pRecv, length);
    _pRecv += length;
    res = length;
  }

  // Enable PDC
  UART->UART_PTCR = UART_PTCR_RXTEN;
  return res;
} // uint16_t consRead(uint8_t *buf, uint16_t length)

// Initialize and set baud speed for the console
void consInit(uint32_t baud) {
  // Configure PMC
  PMC->PMC_WPMR = 0x504D4300;
  PMC->PMC_PCER0 = (1 << ID_UART);

  // Disable UART
  UART->UART_CR = UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RSTSTA;

  // Set 8N1 mode and baud rate
  UART->UART_MR = 0xC0 | UART_MR_PAR_NO;
  UART->UART_BRGR = (SystemCoreClock / baud) / 16;

  // Disable all UART interrupts
  UART->UART_IDR = 0xFFFFFFFF;
  UART->UART_IER = 0;

  _recvReInit();

  // Reset PDC for sending
  UART->UART_TPR = 0;
  UART->UART_TCR = 0;
  UART->UART_TNPR = 0;
  UART->UART_TNCR = 0;

  _consDisable = false;

  // Enable UART
  UART->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
  // Enable receive PDC
  UART->UART_PTCR = UART_PTCR_RXTEN;
} // void consInit(uint32_t baud)

// Put stuff in the output buffer
uint32_t _putToSBuf(uint8_t *buf, int16_t left, uint8_t *data, int16_t length) {
  if (left <= 0)
    return length;
  if (length <= left) {
    memcpy(buf, data, length);
    return 0;
  } else {
    memcpy(buf, data, left);
    return length - left;
  }
} // uint32_t _putToSBuf(uint8_t *buf, uint32_t left, uint8_t *data, uint32_t length)

// Non-blocking write to sending buffer (returns number of characters wrote)
uint16_t consWrite(uint8_t *buf, uint16_t length) {
  uint8_t *pCur, *pNext;
  uint32_t nCur, nNext;
  uint32_t res, left;

  // Check if transmitter is disabled (means something writes to the buffer now)
  if (_consDisable)
    return 0;
  _consDisable = true;

  // Disable PDC
  UART->UART_PTCR = UART_PTCR_TXTDIS;
  // Wait for PDC disable
  while (UART->UART_PTSR & UART_PTSR_TXTEN);

  // Check if we have anything sending at the moment
  nCur = UART->UART_TCR;
  if (nCur == 0) {
    // Just copy data to the output buffer
    left = _putToSBuf(consSBuf, sizeof(consSBuf), buf, length);
    UART->UART_TPR = (uint32_t) consSBuf;
    UART->UART_TCR = length - left;
    res = length - left;
  } else {
    // Something is already sending, add stuff to the buffer
    nNext = UART->UART_TNCR;
    pCur = (uint8_t *) UART->UART_TPR;
    // Check if there is no next PDC transfer
    if (nNext == 0) {
      left = _putToSBuf(pCur + nCur, sizeof(consSBuf) - (pCur - consSBuf) - nCur - 1, buf, length);
      UART->UART_TCR = (uint32_t) nCur + length - left;
      if (left != 0) {
        // We have to copy something to start of buffer and set up next PDC transfer
        // Check if we have buffer already full
        if (pCur == consSBuf)
          res = left;
        else {
          // Set up next PDC transfer
          res = _putToSBuf(consSBuf, pCur - consSBuf - 1, buf + length - left, left);
          UART->UART_TNPR = (uint32_t) consSBuf;
          UART->UART_TNCR = left - res;
          res = length - res;
        }
      } else {
        // Everything is copied into buffer
        res = length;
      }
    } else {
      // Next transfer in the progress
      pNext = (uint8_t *) UART->UART_TNPR;
      left = _putToSBuf(pNext + nNext, pCur - pNext - nNext - 1, buf, length);
      UART->UART_TNCR = nNext + length - left;
      res = length - left;
    }
  }
  
  // Enable PDC
  UART->UART_PTCR = UART_PTCR_TXTEN;
  _consDisable = false;
  return (uint16_t) res;
} // uint16_t consWrite(uint8_t *buf, uint16_t length)

// Returns status of transfer (true - busy)
boolean consSendBusy() {
  uint32_t status = UART->UART_SR;
  if (status & UART_SR_TXEMPTY)
    return false;
  return true;
} // boolean consSendBusy()
