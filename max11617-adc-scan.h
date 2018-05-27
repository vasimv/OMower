// multichannel ADC scanning for MAX11617
// $Id$

#ifndef _MAX11617_ADC_SCAN_H
#define _MAX11617_ADC_SCAN_H

#include <stdint.h>
#include <Arduino.h>

// Number of channels to scan
#define _ADC_11617_CHANNELS_NUM 12

// Exponential filter koefficient (1/5)
#define _ADC_11617_FILTER_K 5

// Device address
#define MAX11617_ADDRESS 0x35

// Setup and configuration bytes
#define MAX11617_SETUP 0b10000010
#define MAX11617_CONFIG 0b00010111

// Disable flag (to prevent I2C bus conflicts)
extern volatile boolean adc11617Disable;

// Results array
extern volatile uint16_t adc11617Arr[_ADC_11617_CHANNELS_NUM];

// Initialize ADC scanner
void adc11617ScanInit();

// Must be called 100 times per second
void adc11617Read();

#endif

