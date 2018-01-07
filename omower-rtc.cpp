// RTC module for OMower (internal RTC in ATSAM3X8E)
// $Id$

#include <omower-defs.h>
#include <omower-rtc.h>
#include <omower-debug.h>

// Hardware init
_hwstatus rtc::begin() {
  debug(L_DEBUG, (char *) F("rtc::begin\n"));
#ifdef NO_RTC_CRYSTAL
  myRTC = new RTCDue(RC);
#else
  myRTC = new RTCDue(XTAL);
#endif
  myRTC->begin();
  readRTC();
  return _hwstatus::ONLINE;
} // _hwstatus rtc::begin()

_status rtc::readRTC() {
  year = myRTC->getYear();
  month = myRTC->getMonth();
  day = myRTC->getDay();
  dayweek = myRTC->getDayofWeek();
  hour = myRTC->getHours();
  minute = myRTC->getMinutes();
  second = myRTC->getSeconds();
  return _status::NOERR;
} // _status rtc::readRTC()

uint32_t rtc::getUnixTime() {
  return myRTC->unixtime();
} // uint32_t rtc::getUnixTime()

_status rtc::saveRTC() {
  myRTC->setDate(day, month, year);
  myRTC->setTime(hour, minute, second);
  dayweek = myRTC->getDayofWeek();
  return _status::NOERR;
} // _status rtc::saveRTC()
