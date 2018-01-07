// MAX11617 scan routines
// $Id$

#include <max11617-adc-scan.h>
#include <due-i2c-blocking.h>
#include <omower-defs.h>

// Number of channels to scan
#define _ADC_11617_CHANNELS_NUM 12

// Exponential filter koefficient (1/10)
#define _ADC_11617_FILTER_K 10

// Results array
volatile uint16_t adc11617Arr[_ADC_11617_CHANNELS_NUM];

boolean adc11617Initialized = false;

// Initialize ADC scanner
void adc11617ScanInit() {
#ifdef BUS_MAX11617
  uint8_t buf[2];

  i2cInit(BUS_MAX11617, 400000);
  memset((uint8_t *) adc11617Arr, 0, sizeof(adc11617Arr));

  // Configuration registers - Vdd as ref, scan all channels, single ended, internal clock
  buf[0] = MAX11617_SETUP;
  buf[1] = MAX11617_CONFIG;
  i2cWrite(BUS_MAX11617, MAX11617_ADDRESS, 0, buf, 2);
  adc11617Initialized = true;
#endif
} // void adc11617ScanInit()

// Must be called 100 times per second
void adc11617Read() {
#ifdef BUS_MAX11617
  uint8_t buf[_ADC_11617_CHANNELS_NUM * 2];
  uint16_t reordered;
  int res;

  if (!adc11617Initialized)
    return;
  memset(buf, 0, sizeof(buf));
  // Read MAX11617 captured values
  res = i2cRead(BUS_MAX11617, 0x35, 0b00010111, buf, sizeof(buf));
  if (res != sizeof(buf))
    debug(L_WARNING, (char *) F("adc11617Read: Did read only %d bytes from MAX11617!\n"), res);

  // Reorder bytes from MAX11617 and apply exponential filter
  for (int i = 0; i < _ADC_11617_CHANNELS_NUM; i++) {
    reordered = (uint16_t) buf[i * 2 + 1] | (((uint16_t) buf[i * 2] & 0xf) << 8);
    adc11617Arr[i] = (uint16_t) (((uint32_t) reordered + (uint32_t) adc11617Arr[i] * (_ADC_11617_FILTER_K - 1))
                  / _ADC_11617_FILTER_K);
  }
#endif
} // void adc11617Read()

