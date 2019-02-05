
#ifndef _LIBMAPLE_RTC_H_
#define _LIBMAPLE_RTC_H_


#ifdef __cplusplus
extern "C" {
#endif


//#include <sys/types.h> // for __time_t_defined
//#include <libmaple/rcc.h>
//#include <libmaple/bkp.h>
//#include <libmaple/exti.h>
//#include <libmaple/nvic.h>
//#include <stdio.h>
//#include <libmaple/pwr.h>
//#include <ext_interrupts.h>

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
	RTCSEL_NONE		= 0,
	RTCSEL_LSE		= 1,
	RTCSEL_LSI		= 2,
	RTCSEL_HSE		= 3,
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
#define RTC_CR_BYPSHAD_BIT      5

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
#define RTC_CR_REFCKON      (1<<4)
#define RTC_CR_TSEDGE       (1<<3)
#define RTC_CR_WUCKSEL_MASK (0x07)

/* Initialization and Status Register */
#define RTC_ISR_TAMP2F_BIT     14
#define RTC_ISR_TAMP1F_BIT     13
#define RTC_ISR_TSOVF_BIT      12
#define RTC_ISR_TSF_BIT        11
#define RTC_ISR_WUTF_BIT       10
#define RTC_ISR_ALRBF_BIT       9
#define RTC_ISR_ALRAF_BIT       8
#define RTC_ISR_INIT_BIT        7
#define RTC_ISR_INITF       BIT(6)
#define RTC_ISR_RSF_BIT         5
#define RTC_ISR_INITS       BIT(4)
#define RTC_ISR_SHPF        BIT(3)
#define RTC_ISR_WUTWF       BIT(2)
#define RTC_ISR_ALRBWF      BIT(1)
#define RTC_ISR_ALRAWF      BIT(0)


uint8_t bcd2bin(uint8_t b) { return ( (10*(b>>4)) + (b&0x0F) ); }
uint8_t bin2bcd(uint8_t b) { return ( ((b/10)<<4) + (b%10) ); }

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

static tm_t _tm;

static void breakTime(time_t epoch_time, tm_t *tm);
static time_t makeTime(tm_t *tm);

void setTime (time_t time_stamp);
void setTimeTM(tm_t *tm); 

static time_t getTimeNow();
static void getTimeTM(tm_t *tm);
tm_t * getTimePtr(time_t t) { breakTime(t, &_tm); return &_tm; }

time_t now() { return getTimeNow(); }
//void nowTM(tm_t *tmm ) { getTime(tmm); }  // non-standard use of now() function, added for compatibility with previous versions of the library

uint8_t yearNow(void)    { getTimeTM(&_tm); return _tm.year; }
uint8_t monthNow(void)   { getTimeTM(&_tm); return _tm.month; }
uint8_t dayNow(void)     { getTimeTM(&_tm); return _tm.day; }
uint8_t weekdayNow(void) { getTimeTM(&_tm); return _tm.weekday; }
uint8_t hourNow(void)    { getTimeTM(&_tm); return _tm.hour; }
uint8_t minuteNow(void)  { getTimeTM(&_tm); return _tm.minute; }
uint8_t secondNow(void)  { getTimeTM(&_tm); return _tm.second; }
//uint8_t pm(void)      { return _pm(RTC->TR); }
uint8_t isPMNow(void)    { return ( hourNow()>=12 ); }

uint8_t year(time_t t)    { breakTime(t, &_tm); return _tm.year; }
uint8_t month(time_t t)   { breakTime(t, &_tm); return _tm.month; }
uint8_t day(time_t t)     { breakTime(t, &_tm); return _tm.day; }
uint8_t weekday(time_t t) { breakTime(t, &_tm); return _tm.weekday; }
uint8_t hour(time_t t)    { breakTime(t, &_tm); return _tm.hour; }
uint8_t minute(time_t t)  { breakTime(t, &_tm); return _tm.minute; }
uint8_t second(time_t t)  { breakTime(t, &_tm); return _tm.second; }
uint8_t isPM(time_t t)    { return (hour(t)>=12); }

static volatile uint32_t rtc_tr, rtc_dr;
//-----------------------------------------------------------------------------
uint8_t _year(uint32_t dr)    { return bcd2bin( (dr>>RTC_DR_YEAR_BIT) & RTC_DR_YEAR_MASK ); }
uint8_t _month(uint32_t dr)   { return bcd2bin( (dr>>RTC_DR_MONTH_BIT) & RTC_DR_MONTH_MASK ); }
uint8_t _day(uint32_t dr)     { return bcd2bin( (dr>>RTC_DR_DAY_BIT) & RTC_DR_DAY_MASK ); }
uint8_t _weekday(uint32_t dr) { return bcd2bin( (dr>>RTC_DR_WEEKDAY_BIT) & RTC_DR_WEEKDAY_MASK ); }
uint8_t _pm(uint32_t tr)      { return ( (tr>>RTC_TR_PM_BIT) & RTC_TR_PM_MASK ); }
uint8_t _hour(uint32_t tr)    { return bcd2bin( (tr>>RTC_TR_HOUR_BIT) & RTC_TR_HOUR_MASK ); }
uint8_t _minute(uint32_t tr)  { return bcd2bin( (tr>>RTC_TR_MINUTE_BIT) & RTC_TR_MINUTE_MASK ); }
uint8_t _second(uint32_t tr)  { return bcd2bin( (tr>>RTC_TR_SECOND_BIT) & RTC_TR_SECOND_MASK ); }

//-----------------------------------------------------------------------------
static volatile void getTimeStamp(void)
{
	rtc_tr = RTC->TR;
	(void)RTC->DR;
	rtc_dr = RTC->DR;
}
//-----------------------------------------------------------------------------
static void getTimeTM(tm_t *tm)
{
	uint32 tr;
	do { // read multiple time till both readings are equal
		getTimeStamp();
		tr = rtc_tr;
		getTimeStamp();
	} while ( tr!=rtc_tr );
//    PRINTF("> INFO: getTimeTM DR: %08X, TR: %08X\n", rtc_dr, rtc_tr);
	tm->year    = _year(rtc_dr);
    tm->month   = _month(rtc_dr);
    tm->day     = _day(rtc_dr);
    tm->weekday = _weekday(rtc_dr);
    tm->pm      = _pm(rtc_tr);
    tm->hour    = _hour(rtc_tr);
    tm->minute  = _minute(rtc_tr);
    tm->second  = _second(rtc_tr);
}
//-----------------------------------------------------------------------------
static time_t getTimeNow()
{
	getTimeTM(&_tm);
	return makeTime(&_tm);
}
//-----------------------------------------------------------------------------
static void breakTime(time_t timeInput, tm_t *tm)
{
// break the given time_t into time components
// this is a more compact version of the C library localtime function
// note that year is offset from 1970 !!!

	uint32_t time = (uint32_t)timeInput;
	tm->second = time % 60;
	time /= 60; // now it is minutes
	tm->minute = time % 60;
	time /= 60; // now it is hours
	tm->hour = time % 24;
	time /= 24; // now it is days
	tm->weekday = ((time + 4) % 7); // Monday is day 1 // (time + 4): Sunday is day 1 

	uint8_t year = 0;
	uint32_t days = 0;
	while((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) {
		year++;
	}
	tm->year = year; // year is offset from 1970 

	days -= LEAP_YEAR(year) ? 366 : 365;
	time -= days; // now it is days in this year, starting at 0

	uint8_t month = 0;
	uint8_t monthLength = 0;
	for (month=0; month<12; month++)
	{
		if (month==1) { // february
			if (LEAP_YEAR(year)) {
				monthLength=29;
			} else {
				monthLength=28;
			}
		} else {
			monthLength = monthDays[month];
		}

		if (time >= monthLength) {
			time -= monthLength;
		} else {
			break;
		}
	}
	tm->month = month + 1;  // jan is month 1  
	tm->day = time + 1;     // day of month
}
//-----------------------------------------------------------------------------
static time_t makeTime(tm_t *tm)
{
// assemble time elements into time_t 
// note year argument is offset from 1970 (see macros in time.h to convert to other formats)
// previous version used full four digit year (or digits since 2000),i.e. 2009 was 2009 or 9
  
	// seconds from 1970 till 1 jan 00:00:00 of the given year
	uint32_t seconds = tm->year*(SECS_PER_DAY * 365);
	for (uint16_t i = 0; i < tm->year; i++) {
		if (LEAP_YEAR(i)) {
			seconds +=  SECS_PER_DAY;   // add extra days for leap years
		}
	}

	// add days for this year, months start from 1
	for (uint16_t i = 1; i < tm->month; i++) {
		if ( (i == 2) && LEAP_YEAR(tm->year)) { 
			seconds += SECS_PER_DAY * 29;
		} else {
			seconds += SECS_PER_DAY * monthDays[i-1];  //monthDay array starts from 0
		}
	}
	seconds+= (tm->day-1) * SECS_PER_DAY;
	seconds+= tm->hour * SECS_PER_HOUR;
	seconds+= tm->minute * SECS_PER_MIN;
	seconds+= tm->second;
	return (time_t)seconds; 
}



#ifdef __cplusplus
}
#endif

#endif // _LIBMAPLE_RTC_H_
