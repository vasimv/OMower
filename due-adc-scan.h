// multichannel ADC scanning for ATSAM3X8E (by timer interrupt)
// $Id$

#ifndef _DUE_ADC_SCAN_H
#define _DUE_ADC_SCAN_H

#include <stdint.h>
#include <Arduino.h>

// Number of channels to scan
#define _MAX_ADC_CHANNELS_NUM 15

// ADC sample rate (should be low enough to convert all ADC channels)
#define _ADC_SAMPLE_RATE 38461

typedef struct {
  uint16_t lastRead;
  boolean active;
  uint16_t *buf;
  uint16_t bufSize;
  uint16_t bufPos; } adcArr_t;

// Results array
extern volatile adcArr_t adcArr[_MAX_ADC_CHANNELS_NUM];
extern volatile uint32_t adcCounter;

// Initialize ADC scanner
void adcScanInit();

// Add channel to scanner
void adcAddChannel(uint16_t adcChannel, uint16_t *buf, uint16_t bufSize);

#endif
