
#ifndef _RTCLOCK_H_
#define _RTCLOCK_H_

#include <usb_serial.h>

#ifdef __cplusplus
extern "C" {
#endif


#include <sys/types.h> // for __time_t_defined
#include <libmaple/rcc.h>
#include <libmaple/bkp.h>
#include <libmaple/exti.h>
#include <libmaple/nvic.h>
#include <stdio.h>

//#define RTC_DEBUG 1


#if !defined(__time_t_defined) // avoid conflict with newlib or other posix libc
  #warning "Using private time_t definintion"
  typedef uint32_t time_t;
#endif


#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24UL)
#define DAYS_PER_WEEK (7UL)
#define SECS_PER_WEEK (SECS_PER_DAY * DAYS_PER_WEEK)
#define SECS_PER_YEAR (SECS_PER_WEEK * 52UL)
#define SECS_YR_2000  (946684800UL) // the time at the start of y2k
#define LEAP_YEAR(Y)  ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )

static  const unsigned char monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31}; // API starts months from 1, this array starts from 0


typedef struct rtc_reg_map {
	__IO uint32 TR;			/**< Time register */
	__IO uint32 DR;			/**< Date register */
	__IO uint32 CR;			/**< Control register */
	__IO uint32 ISR;		/**< Init Status register */
	__IO uint32 PRER;		/**< Prescaler register */
	__IO uint32 WUTR;		/**< Wakeup Timer register */
	__IO uint32 CALIBR;		/**< Calibration register */
	__IO uint32 ALRMAR;		/**< Alarm A register */
	__IO uint32 ALRMBR;		/**< Alarm B register */
	__IO uint32 WPR;		/**< Write Protect register */
	__IO uint32 SSR;		/**< SubSecond register */
	__IO uint32 SHIFTR;		/**< Shift Control register */
	__IO uint32 TSTR;		/**< TimeStamp Time register */
	__IO uint32 TSDR;		/**< TimeStamp Date register */
	__IO uint32 TSSSR;		/**< TimeStamp SubSecond register */
	__IO uint32 CALR;		/**< Calibration register */
	__IO uint32 TAFCR;		/**< Tamper and Alternate Function Config register */
	__IO uint32 ALRMASSR;		/**< Alarm A subSecond register */
	__IO uint32 ALRMBSSR;		/**< Alarm B subSecond register */
	__IO uint32 BKPxR;		/**< Backup registers */
} rtc_reg_map;

/** RTC register map base pointer */
#define RTC        ((struct rtc_reg_map*)0x40002800)

//#define NR_RTC_HANDLERS         4
//voidFuncPtr rtc_handlers[NR_RTC_HANDLERS];     /**< User IRQ handlers */


/**
 * @brief RTC clock source.
 *
 */
typedef enum rtc_clk_src {
	RTCSEL_DEFAULT	= 0,
	RTCSEL_NONE		= 0x10,
	RTCSEL_LSE		= 0x11,
	RTCSEL_LSI		= 0x12,
	RTCSEL_HSE		= 0x13,
} rtc_clk_src;

// Time register
#define RTC_TR_PM_BIT     22
#define RTC_TR_HOUR_BIT   16
#define RTC_TR_MINUTE_BIT  8
#define RTC_TR_SECOND_BIT  0

#define RTC_TR_PM_MASK     (0x01)//<<RTC_TR_PM_BIT)
#define RTC_TR_HOUR_MASK   (0x3F)//<<RTC_TR_HOUR_BIT)
#define RTC_TR_MINUTE_MASK (0x7F)//<<RTC_TR_MINUTE_BIT)
#define RTC_TR_SECOND_MASK (0x7F)//<<RTC_TR_SECOND_BIT)

// Date register
#define RTC_DR_YEAR_BIT    16
#define RTC_DR_WEEKDAY_BIT 13
#define RTC_DR_MONTH_BIT    8
#define RTC_DR_DAY_BIT      0

#define RTC_DR_YEAR_MASK    (0xFF)//<<RTC_TR_YEAR_BIT)
#define RTC_DR_WEEKDAY_MASK (0x07)//<<RTC_TR_WEEKDAY_BIT)
#define RTC_DR_MONTH_MASK   (0x1F)//<<RTC_TR_MONTH_BIT)
#define RTC_DR_DAY_MASK     (0x3F)//<<RTC_TR_DAY_BIT)


/* Control Register */
#define RTC_CR_TSIE 	    (1<<15)
#define RTC_CR_WUTIE 	    (1<<14)
#define RTC_CR_ALRBIE	    (1<<13)
#define RTC_CR_ALRAIE 	    (1<<12)
#define RTC_CR_TSE 		    (1<<11)
#define RTC_CR_WUTE 	    (1<<10)
#define RTC_CR_ALRBE	    (1<<9)
#define RTC_CR_ALRAE 	    (1<<8)
#define RTC_CR_FMT          (1<<6)
#define RTC_CR_BYPSHAD      (1<<5)

/* Initialization and Status Register */
#define RTC_ISR_TSOVF       (1<<12)
#define RTC_ISR_TSF         (1<<11)
#define RTC_ISR_WUTF        (1<<10)
#define RTC_ISR_ALRBF       (1<<9)
#define RTC_ISR_ALRAF       (1<<8)
#define RTC_ISR_INIT        (1<<7)
#define RTC_ISR_INITF       (1<<6)
#define RTC_ISR_RSF         (1<<5)
#define RTC_ISR_INITS       (1<<4)
#define RTC_ISR_SHPF        (1<<3)
#define RTC_ISR_WUTWF       (1<<2)
#define RTC_ISR_ALRBWF      (1<<1)
#define RTC_ISR_ALRAWF      (1<<0)


#define BUILD_TIME_REGISTER(h, m, s) ( ( bin2bcd((h&RTC_TR_HOUR_MASK)) << RTC_TR_HOUR_BIT ) | \
                                       ( bin2bcd((m&RTC_TR_MINUTE_MASK)) << RTC_TR_MINUTE_BIT ) | \
								       ( bin2bcd((s&RTC_TR_SECOND_MASK)) << RTC_TR_SECOND_BIT) )

#define BUILD_DATE_REGISTER(y, m, d, wd) ( ( bin2bcd((y&RTC_DR_YEAR_MASK)) << RTC_DR_YEAR_BIT ) | \
                                           ( bin2bcd((m&RTC_DR_MONTH_MASK)) << RTC_DR_MONTH_BIT) | \
									       ( bin2bcd((d&RTC_DR_DAY_MASK)) << RTC_DR_DAY_BIT ) | \
									       ( (wd&RTC_DR_WEEKDAY_MASK) << RTC_DR_WEEKDAY_BIT ) )

typedef struct tm_t {
	uint8_t  year;    // years since 1970
	uint8_t  month;   // month of a year - [ 1 to 12 ]
	uint8_t  day;     // day of a month - [ 1 to 31 ]
	uint8_t  weekday; // day of a week (first day is Monday) - [ 1 to 7 ]
	uint8_t  pm;      // AM: 0, PM: 1
	uint8_t  hour;    // hour of a day - [ 0 to 23 ]
	uint8_t  minute;  // minute of an hour - [ 0 to 59 ]
	uint8_t  second;  // second of a minute - [ 0 to 59 ]
} tm_t;

static inline uint8_t bcd2bin(uint8_t b) { return ( (10*(b>>4)) + (b&0x0F) ); }
static inline uint8_t bin2bcd(uint8_t b) { return ( ((b/10)<<4) + (b%10) ); }

class RTClock {
  public:
 	RTClock() { RTClock(RTCSEL_LSE, 0, 0); }
    RTClock(rtc_clk_src _src ) { RTClock(_src, 0, 0);	} 
	RTClock(rtc_clk_src _src, uint16 _sync_prescaler, uint16 _async_prescaler);
	void begin();
	//~RTClock(); //to implement
	
	void breakTime(time_t epoch_time, tm_t & tm);
	time_t makeTime(tm_t & tm);

	void setTime (time_t time_stamp);
	void setTime (tm_t & tm); 
	
	time_t getTime();
	void getTime(tm_t & tm);

	time_t now() { return getTime(); }
	void now(tm_t & tmm ) { getTime(tmm); }  // non-standard use of now() function, added for compatibility with previous versions of the library

	uint8_t year(void)    { getTime(tm); return tm.year; }
	uint8_t month(void)   { getTime(tm); return tm.month; }
	uint8_t day(void)     { getTime(tm); return tm.day; }
	uint8_t weekday(void) { getTime(tm); return tm.weekday; }
	uint8_t hour(void)    { getTime(tm); return tm.hour; }
	uint8_t minute(void)  { getTime(tm); return tm.minute; }
	uint8_t second(void)  { getTime(tm); return tm.second; }
	//uint8_t pm(void)      { return _pm(RTC->TR); }
	uint8_t isPM(void)    { return ( hour()>=12 ); }
	
	uint8_t year(time_t t)    { breakTime(t, tm); return tm.year; }
	uint8_t month(time_t t)   { breakTime(t, tm); return tm.month; }
	uint8_t day(time_t t)     { breakTime(t, tm); return tm.day; }
	uint8_t weekday(time_t t) { breakTime(t, tm); return tm.weekday; }
	uint8_t hour(time_t t)    { breakTime(t, tm); return tm.hour; }
	uint8_t minute(time_t t)  { breakTime(t, tm); return tm.minute; }
	uint8_t second(time_t t)  { breakTime(t, tm); return tm.second; }
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
	rtc_clk_src src;
	uint16_t sync_prescaler, async_prescaler;
	inline uint8_t _year(uint32_t dr)    { return bcd2bin( (dr>>RTC_DR_YEAR_BIT) & RTC_DR_YEAR_MASK ); }
	inline uint8_t _month(uint32_t dr)   { return bcd2bin( (dr>>RTC_DR_MONTH_BIT) & RTC_DR_MONTH_MASK ); }
	inline uint8_t _day(uint32_t dr)     { return bcd2bin( (dr>>RTC_DR_DAY_BIT) & RTC_DR_DAY_MASK ); }
	inline uint8_t _weekday(uint32_t dr) { return bcd2bin( (dr>>RTC_DR_WEEKDAY_BIT) & RTC_DR_WEEKDAY_MASK ); }
	inline uint8_t _pm(uint32_t tr)      { return ( (tr>>RTC_TR_PM_BIT) & RTC_TR_PM_MASK ); }
	inline uint8_t _hour(uint32_t tr)    { return bcd2bin( (tr>>RTC_TR_HOUR_BIT) & RTC_TR_HOUR_MASK ); }
	inline uint8_t _minute(uint32_t tr)  { return bcd2bin( (tr>>RTC_TR_MINUTE_BIT) & RTC_TR_MINUTE_MASK ); }
	inline uint8_t _second(uint32_t tr)  { return bcd2bin( (tr>>RTC_TR_SECOND_BIT) & RTC_TR_SECOND_MASK ); }

	tm_t tm;
};

inline uint32_t getTReg(void) { return (uint32_t)(RTC->TR); }
inline uint32_t getDReg(void) { return (uint32_t)(RTC->DR); }



#ifdef __cplusplus
}
#endif

#endif // _RTCLOCK_H_
