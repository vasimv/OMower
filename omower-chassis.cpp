// Chassis data and common hardware initialization
// $Id$

#include <omower-defs.h>
#include <omower-chassis.h>
#include <Arduino.h>
#include <DueTimer.h>
#include <omower-debug.h>
#include <due-adc-scan.h>
#include <due-i2c-blocking.h>
#include <max11617-adc-scan.h>
#include <omower-debug.h>

volatile uint32_t cyclesPollNum;

// These functions will be called 50/20/10 times per second                                                   
// From them all things poll*() functions should be called                                                    
void (*_hookPoll50)();                                                                                  
void (*_hookPoll20)();                                                                                  
void (*_hookPoll10)();   

#ifdef USE_SOFTWARE_INTERRUPTS
// these interrupts handlers are redictors to the poll stuff
void EMAC_Handler() {
  _hookPoll50();
} // void EMAC_Handler()

void HSMCI_Handler() {
  _hookPoll20();
} // void HSMCI_Handler()

void SSC_Handler() {
  _hookPoll10();
} // void SSC_Handler()
#endif


// 100 Hz timer interrupt for poll*
static void TIM7_intcallback() {
  interrupts();
  cyclesPollNum++;
 
  // Check if we're still initializing stuff
  if (_hookPoll50 == NULL)
    return;

  // Read and filter data from MAX11617
  adc11617Read();

#ifdef USE_SOFTWARE_INTERRUPTS
  // 50 Hz
  if (cyclesPollNum % 2 == 0)
    NVIC_SetPendingIRQ(EMAC_IRQn);

  // 20 Hz
  if (cyclesPollNum % 5 == 0)
    NVIC_SetPendingIRQ(HSMCI_IRQn);
  
  // 10 Hz
  if (cyclesPollNum % 10 == 0)
    NVIC_SetPendingIRQ(SSC_IRQn);
#else
  // 50 Hz
  if (cyclesPollNum % 2 == 0)
    _hookPoll50();

  // 20 Hz
  if (cyclesPollNum % 5 == 0)
    _hookPoll20();
  
  // 10 Hz
  if (cyclesPollNum % 10 == 0)
    _hookPoll10();
#endif
} // void TIM7_intcallback()

// Empty hook function (for init)
static void _emptyHook() {
  return;
} // void _emptyHook()

// Hardware init stuff (must be called before all other begin()'s)
_hwstatus chassis::begin() {
  debug(L_DEBUG, (char *) F("chassis::begin\n"));
  // Initialize ADC scan
  adcScanInit();
  adc11617ScanInit();

  // Initialize hooks with empty stuff
  setPollHooks(NULL, NULL, NULL);
  
  pauseTime = 2000;
  init();
  return _hwstatus::ONLINE;
} // _hwstatus chassis::begin()

_status chassis::init() {
  // Timer7 (poll*) init at 100 Hz
  cyclesPollNum = 0;
  Timer7.attachInterrupt(TIM7_intcallback).start(10000);
  NVIC_SetPriority(TC7_IRQn, 10);
#ifdef USE_SOFTWARE_INTERRUPTS
  NVIC_SetPriority(EMAC_IRQn, 11);
  NVIC_EnableIRQ(EMAC_IRQn);
  NVIC_SetPriority(HSMCI_IRQn, 12);
  NVIC_EnableIRQ(HSMCI_IRQn);
  NVIC_SetPriority(SSC_IRQn, 13);
  NVIC_EnableIRQ(SSC_IRQn);
#endif
  // Increase systick interrupt priority so millis() will work correctly 
  // When large amount of slow interrupts from ADC/poll
  NVIC_SetPriority(SysTick_IRQn, 1);

  NVIC_DisableIRQ(PIOA_IRQn);
  NVIC_ClearPendingIRQ(PIOA_IRQn);
  NVIC_SetPriority(PIOA_IRQn, 14);
  NVIC_EnableIRQ(PIOA_IRQn);
  NVIC_DisableIRQ(PIOB_IRQn);
  NVIC_ClearPendingIRQ(PIOB_IRQn);
  NVIC_SetPriority(PIOB_IRQn, 14);
  NVIC_EnableIRQ(PIOB_IRQn);
  NVIC_DisableIRQ(PIOC_IRQn);
  NVIC_ClearPendingIRQ(PIOC_IRQn);
  NVIC_SetPriority(PIOC_IRQn, 14);
  NVIC_EnableIRQ(PIOC_IRQn);
  NVIC_DisableIRQ(PIOD_IRQn);
  NVIC_ClearPendingIRQ(PIOD_IRQn);
  NVIC_SetPriority(PIOD_IRQn, 14);
  NVIC_EnableIRQ(PIOD_IRQn);


  return _status::NOERR;
} // _status chassis::init()

// Set hooks functions for poll*(), can be NULL to set to empty hook
void chassis::setPollHooks(void (*hookPoll50)(), void (*hookPoll20)(), void (*hookPoll10)()) {
  debug(L_INFO, (char *) F("chassis::setPollHooks %X %X %X\n"), hookPoll50, hookPoll20, hookPoll10);
  if (hookPoll50 != NULL)
    _hookPoll50 = hookPoll50;
  else
    _hookPoll50 = &_emptyHook;

  if (hookPoll20 != NULL)
    _hookPoll20 = hookPoll20;
  else
    _hookPoll20 = &_emptyHook;

  if (hookPoll10 != NULL)
    _hookPoll10 = hookPoll10;
  else
    _hookPoll10 = &_emptyHook;
} // void chassis::setPollHooks(void (*hookPoll50)(), void (*hookPoll20)(), void (*hookPoll10)()) 
