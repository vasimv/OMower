// Perimeter sensors for test smallcar
// Based on ardumower code (https://github.com/Ardumower/ardumower)
// $Id$

// We need maximum speed on this one
#pragma GCC optimize ("O3") 

#include <omower-defs.h>
#include <omower-perimeter.h>
#include <due-adc-scan.h>
#include <omower-debug.h>

// Initialize perimeter sensors pins and add channels to adc scanner
// Should be called after adcScanInit()
_hwstatus perimeter::begin() {
  debug(L_DEBUG, (char *) F("perimeter::begin\n"));

  // add all perimeters sensors to adc scanner
  for (uint8_t i = 0; i < _PERIMETERS_NUM; i++) {
    adcAddChannel(periChNum[i],  samples[i], _PERIMETERS_BUF_SIZE);
    offsetADC[i] = 2150;
  }
  return _hwstatus::ONLINE;
} // _hwstatus perimeter::begin() {

// Initialize perimeter sensors variables
_status perimeter::init() {
  // Initialize variables
  memset((void *) mag, 0, sizeof(mag));
  memset((void *) smoothMag, 0, sizeof(smoothMag));
  memset((void *) avgArr, 0, sizeof(avgArr));
  memset((void *) minMaxArr, 0, sizeof(minMaxArr));
  memset((void *) avgMag, 0, sizeof(avgMag));
  memset((void *) minMag, 0, sizeof(minMag));
  memset((void *) maxMag, 0, sizeof(maxMag));
  memset((void *) filterQuality, 0, sizeof(filterQuality));
  memset((void *) signalCounter, 0, sizeof(signalCounter));
  minMaxPos = 0;
  periCycle = 0;
  enabledPerimeter = true;
  lastInside = millis();
  // Fill in sigsProcessed array
  for (uint8_t i = 0; i < sizeof(sigcode); i++)
    for (uint8_t j = 0; j < SUBSAMPLE_SIZE; j++)
      sigProcessed[i * SUBSAMPLE_SIZE + j] = (int16_t) sigcode[i];
  reportToROS();
  return _status::NOERR;
} // _status perimeter::init() {

// Returns signal's magnitude
int16_t perimeter::magnitude(numThing n) {
  return mag[n];
} // int16_t perimter::magnitude(numThing n)

// Returns smoothed signal's magnitude
int16_t perimeter::magnitudeSmooth(numThing n) {
  return smoothMag[n];
} // int16_t perimter::smoothMag(numThing n)

// Maximum signal strength (approximate)
int16_t perimeter::maximumAvg(numThing n) {
  return maxMag[n];
} // int16_t perimeter::maximumAvg(numThing n)

// Minimum signal strength (approximate)
int16_t perimeter::minimumAvg(numThing n) {
  return minMag[n];
} // int16_t perimeter::minimumAvg(numThing n)

// "Quality" of signal
float perimeter::quality(numThing n) {
  return filterQuality[n];
} // float perimeter::quality(numThing n)

// Called 20 times per second
void perimeter::poll20() {
  if (!enabledPerimeter)
    return;

  // Check all perimeters sensors
  for (uint8_t i = 0; i < _PERIMETERS_NUM; i++) {
    matchedFilter(i);
    if (inside(i))
      lastInside = millis();
  }
} // void perimeter::poll20()

// Soft error (outside timer reached)
_hwstatus perimeter::softError() { 
  if (((millis() - lastInside) / 1000) > errorTimeout)
    return _hwstatus::ERROR;
  return _hwstatus::ONLINE;
} // _hwstatus perimeter::softError() 

// Set tracking mode for readCourseError()
void perimeter::setTracking(boolean tracking) {
  trackingMode = tracking;
  searchingMax = 0;
  maximumMag = 0;
  minimumMag = 9999;
} // void perimeter::setTracking(boolean tracking)

// Course error for PID regulator
float perimeter::readCourseError() {
  boolean insideCur = inside(0);
  uint16_t magnitude = abs(mag[0]);
  float res;

  if (trackingMode) {
    // Tracking perimeter wire
    if (filterQuality[0] < minQuality)
      return 1000;
    if ((locThings() == _locationThings::JUSTONE) || (locThings() == _locationThings::FORW_BACK)) {
      // Only one forward sensor in use
      // Outside - turn right, inside - turn left
      if (insideCur)
        res = -M_PI / 2;
      else
        res = M_PI / 2;
    } else {
      // Left&right sensor use (trying to stay one out and one inside)
      if (!inside(0) && inside(1)) {
        // left - outside, right - inside, let's try to make them same magnitude
        // by small correction
        if (abs(mag[0]) > abs(mag[1]))
          res = - mag[1] / mag[0] * (M_PI / 20);
        else
          res = mag[0] / mag[1] * (M_PI / 20);
      } else {
        // Both sensor in wrong position, make zig-zag turn
        if (inside(0))
          res = -M_PI / 2;
        else
          res = M_PI / 2;
      }
    }
  } else {
    // Searching for maximum magnitude signal
    // Temporary stop if we're inside perimeter
    if (insideCur)
      return 1000;
    debug(L_NOTICE, (char *) F("perimeter::readCourseError: mag %hu, maximumMag %hu, status %lu\n"),
          magnitude, maximumMag, (unsigned short) searchingMax);
    // Finding maximum - just rotating in one direction
    searchingMax++;
    if (searchingMax < 200) {
      if (magnitude > maximumMag)
        maximumMag = magnitude;
      if (magnitude < minimumMag)
        minimumMag = magnitude;
      // Check if we went through maximum and minimum already
      return M_PI / 2;
      if (searchingMax < 10)
        return M_PI / 2;
      if ((magnitude < (((uint32_t) minimumMag * 12) / 10)) || 
          (magnitude >= (((uint32_t) maximumMag * 9) / 10))) 
        return M_PI / 2;
      searchingMax = 200;
    }
    // Rotating left until we see around maximum
    if (magnitude >= (((uint32_t) maximumMag * 9) / 10)) {
      res = -1000.0;
    } else {
      // If we're rotating for more than 40 seconds - reset
      if (searchingMax > 400) {
        maximumMag = 0;
        minimumMag = 9999;
        searchingMax = 0;
      }
      res = -M_PI / 2;
    }
  }
  return res;
} // float perimeter::readCourseError()

// perimeter V2 uses a digital matched filter
// from ardumower code
void perimeter::matchedFilter(numThing n) {
  int32_t sum;
  int16_t sampleCount = adcArr[periChNum[n]].bufPos;

  if (sampleCount < _PERIMETERS_BUF_SIZE ) {
    debug(L_NOTICE, (char *) F("perimeter::matchedFilter: buffer underrun: %d\n"), sampleCount);
    return;
  }

  // magnitude for tracking (fast but inaccurate)    
  mag[n] = corrFilter(samples[n], sampleCount, filterQuality[n], offsetADC[n]);
  if (invertMag)
    mag[n] = -mag[n];        

  // smoothed magnitude used for signal-off detection
  smoothMag[n] = 0.99 * smoothMag[n] + 0.01 * (float) mag[n];

  // perimeter inside/outside detection
  if (abs(mag[n]) > 200) {
    // Reset signal counter at very high strength signals
    if (mag[n] >= 0)
      signalCounter[n] = 1;
    else
      signalCounter[n] = -1;
  } else {
    // in case of too low signal  - count as outside
    if ((mag[n] > -11) || (filterQuality[n] < minQuality)) {
      signalCounter[n] = min(signalCounter[n] + 1, 3);   
    } else {
      signalCounter[n] = max(signalCounter[n] - 1, -3);
    }
  }
  debug(L_DEBUG, (char *) F("perim: %hd = %hd/%hd/%f (%hu)\n"), n, mag[n],
        signalCounter[n], filterQuality[n], offsetADC[n]);

  // Restart buffer filling in ADC scanner
  adcArr[periChNum[n]].bufPos = 0;

  // Recalculate average/min/max
  avgArr[n][periCycle % _PERIMETERS_AVG_SIZE] = mag[n];
  sum = 0;
  for (uint8_t i = 0; i < _PERIMETERS_AVG_SIZE; i++)
    sum += avgArr[n][i];
  avgMag[n] = sum / _PERIMETERS_AVG_SIZE;

  periCycle++;
  if (periCycle % _PERIMETERS_AVG_SIZE == 0) {
    minMaxArr[n][minMaxPos] = avgMag[n];
    minMaxPos = (minMaxPos + 1) % _PERIMETERS_AVG_SIZE;
    uint16_t min = 9999;
    uint16_t max = -9999;
    for (uint8_t i = 0; i < _PERIMETERS_AVG_SIZE; i++) {
      if (minMaxArr[n][i] > max)
        max = minMaxArr[n][i];
      if (minMaxArr[n][i] < min)
        min = minMaxArr[n][i];
    }
    minMag[n] = min;
    maxMag[n] = max;
  }
} // void perimeter::matchedFilter(numThing n)

// Check if we're inside or outside
// from ardumower code
boolean perimeter::inside(numThing n) {
  if (n == -1) {
    // Recursive check all perimeter sensors
    for (numThing i = 0; i < numThings(); i++)
      if (!inside(i))
        return false;
    return true;
  } 
  if (abs(mag[n]) > 1000) {
    // Large signal, the in/out detection is reliable.
    // Using mag yields very fast in/out transition reporting.
    return (mag[n] < 0);
  } else {
    // Low signal, use filtered value for increased reliability
    return (signalCounter[n] < 0);
  }
} // boolean perimeter::inside(numThing n)


// digital matched filter (cross correlation) from ardumower code, optimized for speed
int16_t perimeter::corrFilter(uint16_t *ip, uint8_t nPts, float &quality, uint16_t &offset) {
  int32_t sumMax = -9999; // max correlation sum
  int32_t sumMin = 9999; // min correlation sum
  uint8_t Ms = sizeof(sigcode) * SUBSAMPLE_SIZE; // number of filter coeffs including subsampling
  int32_t sum;
  int16_t *Hi;
  uint8_t ss;
  int16_t *ipi;
  uint8_t i, j;
  int16_t newSamples[_PERIMETERS_BUF_SIZE];
  uint8_t nnPts;
  int16_t tOffset = (int16_t) offset;
  int16_t tSample;
  
  nnPts = nPts - Ms; 
  // convert samples array to signed int to speed-up and calculate new ADC offset
  sum = 0;
  for (i = 0; i < nPts; i++) {
    tSample = (int16_t) ip[i];
    newSamples[i] = tSample - tOffset;
    sum += tSample;
  }
  offset = sum / nPts;

  // compute correlation
  // for each input value
  for (j = 0; j < nnPts; j++)
  {
      sum = 0;      
      Hi = sigProcessed;
      ss = 0;
      ipi = &newSamples[j];
      // for each filter coeffs
      for (i = 0; i < Ms; i++)
      {        
        sum += *Hi * *ipi;
        ipi++;
        Hi++;
      }      
      if (sum > sumMax) sumMax = sum;
      if (sum < sumMin) sumMin = sum;
  }      
  
  // compute ratio min/max 
  if (sumMax > -sumMin) {
    quality = ((float)sumMax) / ((float)-sumMin);
    return sumMax / 64;
  } else {
    quality = ((float)-sumMin) / ((float)sumMax);
    return sumMin / 64;
  }  
} // int16_t perimeter::corrFilter(uint16_t *ip, uint8_t nPts, float &quality, uint16_t &offset)

// Disable perimeter
_status perimeter::disableThings() {
  enabledPerimeter = false;
  return _status::NOERR;
} // _status perimeter::disableThings()

// Enable perimeter
_status perimeter::enableThings() {
  enabledPerimeter = true;
  return _status::NOERR;
} // _status perimeter::enableThings()
