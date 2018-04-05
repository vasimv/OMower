// Wire perimeter sensor
// $Id$

#ifndef _PERIMETER_OMOWER_H
#define _PERIMETER_OMOWER_H

#include <omower-root.h>
#include <omower-defs.h>
#include <stdint.h>
#include <Arduino.h>

// Positions of perimeters sensors
#ifndef _PERIMETERS_POS
#define _PERIMETERS_POS _locationThings::SPECIAL
#endif

// Size of ADC buffer for the perimeter sensor (140..255)
// bigger number of samples will give better results but slows down the processing
// You will have to reduce its number to minimum  for more than two perimeter sensors
#define _PERIMETERS_BUF_SIZE 255

// Size of average/min-max arrays
#define _PERIMETERS_AVG_SIZE 10

// Sub-sample size for 38462Hz
#define SUBSAMPLE_SIZE 4

class perimeter : public navThing {
public:
  // Hardware initialization (once)
  _hwstatus begin();

  // Software init
  _status init();

  // Disable/enable perimeter (to save MCU time)
  _status enableThings();
  _status disableThings();

  // Settings variables
  // Minimum quality (ignored if lower)
  float minQuality;
  // Timeout (in seconds) of perimeter outage (generates softError())
  uint8_t errorTimeout;
  // Inverted perimeter signal
  boolean invertMag;

  // If we're inside of perimeter (too low signal will be counted as outside)
  boolean inside(numThing n);

  // Signal strength (negative - inside, positive - outside)
  int16_t magnitude(numThing n);

  // Filtered signal strength
  int16_t magnitudeSmooth(numThing n);

  // Maximum signal strength (approximate)
  int16_t maximumAvg(numThing n);

  // Minimum signal strength (approximate)
  int16_t minimumAvg(numThing n);

  // "Quality" of signal
  float quality(numThing n);

  // Set tracking mode for readCourseError() if parameter is true
  // (when outside of perimeter - turn right, when inside - left)
  // When false - just turn for maximum magnitude signal
  void setTracking(boolean tracking);

  // Course correction by maximum signal strength
  float readCourseError();

  // Error status if outside of perimeter too long
  _hwstatus softError();

  // Calibrate ADC offset (perimeter must be turned off!)
  _status calibADC();

  // Must be called 20 times per second
  void poll20();

private:
  // Enabled or disabled status
  boolean enabledPerimeter;

  // Samples buffer
  uint16_t samples[_PERIMETERS_NUM][_PERIMETERS_BUF_SIZE];

  // perimeter sensor number -> channel number
#if _PERIMETERS_NUM == 2
  const uint8_t periChNum[_PERIMETERS_NUM] = { ADC_PERIMETER_CH1, ADC_PERIMETER_CH2 };
#else
  const uint8_t periChNum[_PERIMETERS_NUM] = { ADC_PERIMETER_CH1};
#endif

  // Signal's magnitude
  int16_t mag[_PERIMETERS_NUM];

  // Smoothed signal's magnitude
  float smoothMag[_PERIMETERS_NUM];

  // filtered quality
  float filterQuality[_PERIMETERS_NUM];

  // Signal counter array (for low strength detection
  int32_t signalCounter[_PERIMETERS_NUM];

  // Approximate minimum magnitude
  int16_t minMag[_PERIMETERS_NUM];

  // Approximate maximum magnitude
  int16_t maxMag[_PERIMETERS_NUM];

  // Approximate average magnitude
  int16_t avgMag[_PERIMETERS_NUM];

  // Averaging array (0.5 seconds buffer)
  int16_t avgArr[_PERIMETERS_NUM][_PERIMETERS_AVG_SIZE];

  // Array containing averages to calculate min/max (for few seconds)
  int16_t minMaxArr[_PERIMETERS_NUM][_PERIMETERS_AVG_SIZE];
  uint8_t minMaxPos;

  // Calibration offset values
  uint16_t offsetADC[_PERIMETERS_NUM];

  // Cycle counter for perimeter checks
  unsigned long periCycle;

  // Tracking mode
  boolean trackingMode;

  // SIGCODE array
  const int8_t sigcode[24] = { 1,0,-1, 0,1,-1,1,-1, 0,1,-1,1,0,-1, 0,1,-1, 0,1,-1, 0,1,0,-1 };

  // Converted sigcode array (just repeats sigcode by SUBSAMPLE_SIZE)
  int16_t sigProcessed[24 * SUBSAMPLE_SIZE];

  // last time when we were inside
  unsigned long lastInside;

  // Internal stuff for readCourseError()
  uint16_t maximumMag;
  uint16_t minimumMag;
  uint16_t searchingMax;

  // perimeter V2 uses a digital matched filter
  // from ardumower code (had to optimize it to process 255 samples in <5ms)
  void matchedFilter(numThing n);
  int16_t corrFilter(uint16_t *ip, uint8_t nPts, float &quality, uint16_t &offset);
};

#endif
