// ATSAM3X8E adc multichannel scan routine (without PDC/DMA)
// $Id$

#include <due-adc-scan.h>
#include <DueTimer.h>
#include <omower-defs.h>

volatile uint32_t adcCounter;
volatile adcArr_t adcArr[_MAX_ADC_CHANNELS_NUM];
volatile uint32_t adcAllChannelsMask;

// Dummy function for steppers motors driver
__attribute__((weak)) void pulseSteppers() {}

// Timer interrupt handler
void adcTimerCallback() {
  interrupts();
  // Increase adc conversion counter
  adcCounter++;

  // Read data
  for (uint8_t i = 0; i < _MAX_ADC_CHANNELS_NUM; i++) {
    if (adcArr[i].active) {
      adcArr[i].lastRead = *(uint16_t *)(ADC->ADC_CDR + i);
      // Copy to buffer if needed
      if (adcArr[i].buf && (adcArr[i].bufPos < adcArr[i].bufSize)) {
        adcArr[i].buf[adcArr[i].bufPos] = adcArr[i].lastRead;
        adcArr[i].bufPos++;
      }
    }
  }

  // Start new conversion
  ADC->ADC_CHER = adcAllChannelsMask;
  ADC->ADC_CR = ADC_CR_START;

  // Call steppers motor driver
  pulseSteppers();
} // void adcTimerCallback

// Add adc channel to the scan table
void adcAddChannel(uint16_t adcChannel, uint16_t *buf, uint16_t bufSize) {
  boolean startScanning = false;

  if (!adcAllChannelsMask)
    startScanning = true;

  // Generate channels mask (for ADC_CHER)
  adcAllChannelsMask |= (1 << adcChannel);
  ADC->ADC_CHER = adcAllChannelsMask;

  adcArr[adcChannel].buf = buf;
  adcArr[adcChannel].bufSize = bufSize;
  adcArr[adcChannel].active = true;
  adcArr[adcChannel].bufPos = 0;

  if (startScanning) {
    // Start first conversion
    ADC->ADC_CR = ADC_CR_START;

    // Configure and start interrupt timer (wait enough for first conversion before)
    delayMicroseconds(100);
    Timer5.attachInterrupt(adcTimerCallback).setFrequency(_ADC_SAMPLE_RATE).start();
    NVIC_SetPriority(TC5_IRQn, 2);
  }
} // void adcAddChannel(uint16_t adcChannel, uint16_t *buf, uint16_t bufSize)

// Setup scanning (with timer interrupts)
void adcScanInit() {
  adcCounter = 0;

  memset((void *) adcArr, 0, sizeof(adcArr));
  adcAllChannelsMask = 0;
  
  // Configure ADC, maximum speed, no interrupts, software trigger
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_set_resolution(ADC, (adc_resolution_t) ADC_MR_LOWRES_BITS_12);
  adc_disable_interrupt(ADC, ADC_IER_DRDY);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0);

  // Start scanning on all channels
  adcAddChannel(0, NULL, 0);
  adcAddChannel(1, NULL, 0);
  adcAddChannel(3, NULL, 0);
  adcAddChannel(4, NULL, 0);
  adcAddChannel(5, NULL, 0);
  adcAddChannel(6, NULL, 0);
#ifndef NO_ADC_CHANNEL7
  adcAddChannel(7, NULL, 0);
#endif
  adcAddChannel(10, NULL, 0);
  adcAddChannel(11, NULL, 0);
  adcAddChannel(12, NULL, 0);
} // void adcScanInit()
