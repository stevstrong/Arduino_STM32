//
// RTClock.h
//

#ifndef _RTCLOCK_H_
#define _RTCLOCK_H_

#ifdef __cplusplus
extern "C" {
#endif

#undef __time_t_defined
#undef _TIME_T_DECLARED
#include <sys/types.h> // for __time_t_defined
#include <libmaple/rtc.h>
#include <libmaple/rcc.h>
#include <libmaple/bkp.h>
#include <libmaple/exti.h>
#include <libmaple/nvic.h>
#include <stdio.h>
#include <libmaple/pwr.h>
#include <ext_interrupts.h>

//#define RTC_DEBUG 1
typedef	_TIME_T_ time_t;

class RTClock {
  public:
 	RTClock() {}
	void begin(rtc_clk_src _src, uint16 _sync_prescaler, uint16 _async_prescaler);
	void begin(rtc_clk_src _src) { begin(_src, 0, 0); }
	void begin() { begin(RTCSEL_LSE, 0, 0); }
	//~RTClock(); //to implement
	
	void breakTime(time_t timeInput, tm_t & tm);
	time_t makeTime(tm_t & tm);

	void setTime (time_t time_stamp);
	void setTime (tm_t & tm); 
	
	time_t getTime();
	void getTime(tm_t & tm);

	time_t now() { return getTime(); }
	void now(tm_t & tmm ) { getTime(tmm); }  // non-standard use of now() function, added for compatibility with previous versions of the library

	void reset(void);

	uint8_t year(void)    { getTime(_tm); return _tm.year; }
	uint8_t month(void)   { getTime(_tm); return _tm.month; }
	uint8_t day(void)     { getTime(_tm); return _tm.day; }
	uint8_t weekday(void) { getTime(_tm); return _tm.weekday; }
	uint8_t hour(void)    { getTime(_tm); return _tm.hour; }
	uint8_t minute(void)  { getTime(_tm); return _tm.minute; }
	uint8_t second(void)  { getTime(_tm); return _tm.second; }
	//uint8_t pm(void)      { return _pm(RTC->TR); }
	uint8_t isPM(void)    { return ( hour()>=12 ); }
	
	uint8_t year(time_t t)    { breakTime(t, _tm); return _tm.year; }
	uint8_t month(time_t t)   { breakTime(t, _tm); return _tm.month; }
	uint8_t day(time_t t)     { breakTime(t, _tm); return _tm.day; }
	uint8_t weekday(time_t t) { breakTime(t, _tm); return _tm.weekday; }
	uint8_t hour(time_t t)    { breakTime(t, _tm); return _tm.hour; }
	uint8_t minute(time_t t)  { breakTime(t, _tm); return _tm.minute; }
	uint8_t second(time_t t)  { breakTime(t, _tm); return _tm.second; }
	uint8_t isPM(time_t t)    { return (hour(t)>=12); }
	
	void setAlarmATime (tm_t * tm_ptr, bool hours_match = true, bool mins_match = true, bool secs_match = true, bool date_match = false);
	void setAlarmATime (time_t alarm_time, bool hours_match = true, bool mins_match = true, bool secs_match = true, bool date_match = false); 
	void turnOffAlarmA();
	void setAlarmBTime (tm_t * tm_ptr, bool hours_match = true, bool mins_match = true, bool secs_match = true, bool date_match = false);
	void setAlarmBTime (time_t alarm_time, bool hours_match = true, bool mins_match = true, bool secs_match = true, bool date_match = false); 
	void turnOffAlarmB();
	
	void setPeriodicWakeup(uint16 period);

	void attachPeriodicWakeupInterrupt(voidFuncPtr function); 
	void detachPeriodicWakeupInterrupt();
	inline void attachSecondsInterrupt(voidFuncPtr function) { attachPeriodicWakeupInterrupt(function); }
	inline void detachSecondsInterrupt() { detachPeriodicWakeupInterrupt(); }

	void attachAlarmAInterrupt(voidFuncPtr function); 
	void detachAlarmAInterrupt();
	void attachAlarmBInterrupt(voidFuncPtr function); 
	void detachAlarmBInterrupt();

  private:
	inline uint8_t _year(uint32_t dr)    { return bcd2bin( (dr>>RTC_DR_YEAR_BIT) & RTC_DR_YEAR_MASK ); }
	inline uint8_t _month(uint32_t dr)   { return bcd2bin( (dr>>RTC_DR_MONTH_BIT) & RTC_DR_MONTH_MASK ); }
	inline uint8_t _day(uint32_t dr)     { return bcd2bin( (dr>>RTC_DR_DAY_BIT) & RTC_DR_DAY_MASK ); }
	inline uint8_t _weekday(uint32_t dr) { return bcd2bin( (dr>>RTC_DR_WEEKDAY_BIT) & RTC_DR_WEEKDAY_MASK ); }
	inline uint8_t _pm(uint32_t tr)      { return ( (tr>>RTC_TR_PM_BIT) & RTC_TR_PM_MASK ); }
	inline uint8_t _hour(uint32_t tr)    { return bcd2bin( (tr>>RTC_TR_HOUR_BIT) & RTC_TR_HOUR_MASK ); }
	inline uint8_t _minute(uint32_t tr)  { return bcd2bin( (tr>>RTC_TR_MINUTE_BIT) & RTC_TR_MINUTE_MASK ); }
	inline uint8_t _second(uint32_t tr)  { return bcd2bin( (tr>>RTC_TR_SECOND_BIT) & RTC_TR_SECOND_MASK ); }

	tm_t _tm;
	uint16_t sync_prescaler, async_prescaler;
	rtc_clk_src clk_src;
};

//extern volatile uint32 rtc_tr, rtc_dr;
volatile void getTimeStamp(void);// { rtc_tr = RTC->TR; (void)RTC->DR; rtc_dr = RTC->DR; }



#ifdef __cplusplus
}
#endif

#endif // _RTCLOCK_H_
