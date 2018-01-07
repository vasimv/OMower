// RTC module for OMower
// $Id$

#ifndef _RTC_OMOWER_H
#define _RTC_OMOWER_H

#include <omower-root.h>
#include <stdint.h>
#include <Arduino.h>
#include <RTCDue.h>

// RTC data                                                                                                   
class rtc : public thing {                                                                                    
public:                                                                                                       
  // Date/time in RTC                                                                                         
  uint16_t year;                                                                                              
  uint16_t month;                                                                                             
  uint16_t day;                                                                                               
  uint16_t hour;                                                                                             
  uint16_t minute;                                                                                            
  uint16_t second;                                                                                            
  uint8_t dayweek;                                                                                            
  // Hardware init
  _hwstatus begin();
                                                                                                              
  // Read RTC data                                                                                            
  _status readRTC();                                                                                          

  // Get current time in unixtime format
  uint32_t getUnixTime();
                                                                                                              
  // Save RTC data (dayweek will be calculated)                                                               
  _status saveRTC();                                                                                          

private:
  RTCDue *myRTC;
};               

#endif
