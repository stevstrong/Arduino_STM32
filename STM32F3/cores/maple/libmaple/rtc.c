#include <libmaple/rtc.h>

uint8_t bcd2bin(uint8_t b) { return ( (10*(b>>4)) + (b&0x0F) ); }
uint8_t bin2bcd(uint8_t b) { return ( ((b/10)<<4) + (b%10) ); }

tm_t * getTimePtr(time_t t) { breakTimeTM(t, &_tm); return &_tm; }

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

uint8_t year(time_t t)    { breakTimeTM(t, &_tm); return _tm.year; }
uint8_t month(time_t t)   { breakTimeTM(t, &_tm); return _tm.month; }
uint8_t day(time_t t)     { breakTimeTM(t, &_tm); return _tm.day; }
uint8_t weekday(time_t t) { breakTimeTM(t, &_tm); return _tm.weekday; }
uint8_t hour(time_t t)    { breakTimeTM(t, &_tm); return _tm.hour; }
uint8_t minute(time_t t)  { breakTimeTM(t, &_tm); return _tm.minute; }
uint8_t second(time_t t)  { breakTimeTM(t, &_tm); return _tm.second; }
uint8_t isPM(time_t t)    { return (hour(t)>=12); }


uint8_t _year(uint32_t dr)    { return bcd2bin( (dr>>RTC_DR_YEAR_BIT) & RTC_DR_YEAR_MASK ); }
uint8_t _month(uint32_t dr)   { return bcd2bin( (dr>>RTC_DR_MONTH_BIT) & RTC_DR_MONTH_MASK ); }
uint8_t _day(uint32_t dr)     { return bcd2bin( (dr>>RTC_DR_DAY_BIT) & RTC_DR_DAY_MASK ); }
uint8_t _weekday(uint32_t dr) { return bcd2bin( (dr>>RTC_DR_WEEKDAY_BIT) & RTC_DR_WEEKDAY_MASK ); }
uint8_t _pm(uint32_t tr)      { return ( (tr>>RTC_TR_PM_BIT) & RTC_TR_PM_MASK ); }
uint8_t _hour(uint32_t tr)    { return bcd2bin( (tr>>RTC_TR_HOUR_BIT) & RTC_TR_HOUR_MASK ); }
uint8_t _minute(uint32_t tr)  { return bcd2bin( (tr>>RTC_TR_MINUTE_BIT) & RTC_TR_MINUTE_MASK ); }
uint8_t _second(uint32_t tr)  { return bcd2bin( (tr>>RTC_TR_SECOND_BIT) & RTC_TR_SECOND_MASK ); }

uint32_t rtc_tr, rtc_dr;

volatile void getTimeStamp(void)
{
	rtc_tr = RTC->TR;
	(void)RTC->DR;
	rtc_dr = RTC->DR;
}

void getTimeTM(tm_t *tm)
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

time_t getTimeNow()
{
	getTimeTM(&_tm);
	return makeTimeTM(&_tm);
}

static void breakTimeTMTM(time_t timeInput, tm_t *tm)
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

time_t makeTimeTM(tm_t *tm)
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


