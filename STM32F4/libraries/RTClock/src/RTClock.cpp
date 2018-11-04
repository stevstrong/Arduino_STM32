/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 Inspired of the F1xx version adapted for the F4xx, not much F1xx left.
 author : Martin Ayotte, 2015.
 */

#include "RTClock.h"
#include <wirish_time.h>

voidFuncPtr handlerAlarmA = NULL;
voidFuncPtr handlerAlarmB = NULL;
voidFuncPtr handlerPeriodicWakeup = NULL;


  char dbg_s[200];
#ifdef RTC_DEBUG
  #define PRINTF(...) { sprintf(dbg_s, __VA_ARGS__); Serial.print(dbg_s); }
#else
  #define PRINTF(...) 
#endif
  #define PRINTF1(...) { sprintf(dbg_s, __VA_ARGS__); Serial.print(dbg_s); }
//-----------------------------------------------------------------------------
// Clear the register synchronized flag. The flag is then set by hardware after a write to PRL/DIV or CNT.
//-----------------------------------------------------------------------------
static inline void rtc_clear_sync() {
	*bb_perip(&RTC->ISR, RTC_ISR_RSF_BIT) = 0;
}

#if 0
//-----------------------------------------------------------------------------
// Check (wait if necessary) to see RTC registers are synchronized.
//-----------------------------------------------------------------------------
void rtc_wait_sync()
{
	PRINTF("> rtc_wait_sync\n");
	uint32 t = millis();
	while ( !(RTC->ISR & BIT(RTC_ISR_RSF_BIT)) ) 
	{
	    if ( (millis()-t)>1500) {
			PRINTF("Sync Timeout ! ISR = %08X\n", RTC->ISR);
			break;
		}
	}
	PRINTF("< rtc_wait_sync\n");
}
#endif
//-----------------------------------------------------------------------------
// Enter configuration mode.
//-----------------------------------------------------------------------------
static void rtc_enter_config_mode()
{
	PRINTF("> rtc_enter_config_mode\n");
	noInterrupts();
	// Unlock Write Protect
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	PRINTF("RTC->ISR(1) = %08X\n", RTC->ISR);
	//*bb_perip(&RTC->ISR, RTC_ISR_INIT_BIT) = 1;
	RTC->ISR = 0x1FFFF;
	PRINTF("RTC->ISR(2) = %08X\n", RTC->ISR);
	uint32 t = 0;
	while (!(RTC->ISR & RTC_ISR_INITF))
	{
	    if (++t > 10000000) {
			PRINTF("RTC->ISR.INITF Timeout ! ISR = %08X\n", RTC->ISR);
			break;;
		}
	}
	PRINTF("< rtc_enter_config_mode\n");
}

//-----------------------------------------------------------------------------
// Exit configuration mode.
//-----------------------------------------------------------------------------
static inline void rtc_exit_config_mode()
{
	*bb_perip(&RTC->ISR, RTC_ISR_INIT_BIT) = 0;
	interrupts();
	PRINTF("< rtc_exit_config_mode\n");
	//delayMicroseconds(100);
}

//-----------------------------------------------------------------------------
// Enable an RTC alarm event. Enabling this event allows waking up from deep sleep via WFE.
//-----------------------------------------------------------------------------
static void rtc_enable_alarm_event()
{
    EXTI_BASE->IMR  |= EXTI_RTC_ALARM;
	EXTI_BASE->EMR  |= EXTI_RTC_ALARM;
	EXTI_BASE->RTSR |= EXTI_RTC_ALARM;
}

//-----------------------------------------------------------------------------
// Disable the RTC alarm event.
//-----------------------------------------------------------------------------
//static void rtc_disable_alarm_event()
//{
//	EXTI_BASE->EMR  &= ~(EXTI_RTC_ALARM);
//	EXTI_BASE->RTSR &= ~(EXTI_RTC_ALARM);
//}

//-----------------------------------------------------------------------------
// @brief Enable an RTC Wakeup event. 
//-----------------------------------------------------------------------------
static void rtc_enable_wakeup_event()
{
    EXTI_BASE->IMR  |= EXTI_RTC_WAKEUP;
	EXTI_BASE->EMR  |= EXTI_RTC_WAKEUP;
	EXTI_BASE->RTSR |= EXTI_RTC_WAKEUP;
}
volatile void getTimeStamp(void)
{
	rtc_tr = RTC->TR;
	(void)RTC->DR;
	rtc_dr = RTC->DR;
}

//-----------------------------------------------------------------------------
// @brief Disable the RTC alarm event.
//-----------------------------------------------------------------------------
//static void rtc_disable_wakeup_event()
//{
//	EXTI_BASE->EMR  &= ~(EXTI_RTC_WAKEUP);
//	EXTI_BASE->RTSR &= ~(EXTI_RTC_WAKEUP);
//}
typedef struct {
uint16_t s_presc;
uint16_t as_presc;
} prescaler_t;
const prescaler_t prescalers[4] = {
	{   0,   0}, // RTCSEL_NONE	
	{ 255, 127}, // RTCSEL_LSE
	{ 249, 127}, // RTCSEL_LSI
	{7999, 124}, // RTCSEL_HSE
};
//-----------------------------------------------------------------------------
void RTClock::begin(rtc_clk_src src, uint16 sync_presc, uint16 async_presc)
{
	clk_src = src;
	sync_prescaler = sync_presc;
	async_prescaler = async_presc;

    PRINTF("> RTClock::begin\n");
    //PRINTF("PWR->CR(1) = %08X\n", PWR->CR);
    bkp_init();		// turn on peripheral clocks to PWR and BKP and reset the backup domain via RCC registers.

    PRINTF("bkp_enable_writes\n");
    bkp_enable_writes();	// enable writes to the backup registers and the RTC registers via the DBP bit in the PWR control register
    PRINTF("PWR->CR(2) = %08X\n", PWR->CR);

	rcc_set_prescaler(RCC_PRESCALER_RTC, RCC_RTCCLK_DIV(CRYSTAL_FREQ)); // Set the RTCPRE to 8.
	PRINTF("RCC->CFGR = %08X\n", RCC->CFGR);

	PRINTF("RTC clock source: %s\n", (clk_src==RTCSEL_LSE)?"LSE":((clk_src==RTCSEL_LSI)?"LSI":((clk_src==RTCSEL_HSE)?"HSE":"NONE")));
    switch (clk_src)
	{
	case RTCSEL_LSE:
	{
		PRINTF("Preparing RTC for LSE mode, RCC->BDCR = %08X\n", RCC->BDCR);
	    if ((RCC->BDCR & RCC_BDCR_RTCSEL_MASK) == RCC_BDCR_RTCSEL_LSE)
			break;
		RCC->BDCR = RCC_BDCR_BDRST; // Reset the entire Backup domain
		PRINTF("BCKP domain reset\n");

		RCC->BDCR = (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSE | RCC_BDCR_LSEON);
		PRINTF("RCC->BDCR = %08X\n", RCC->BDCR);
		uint32 t = 0;
		while (!(RCC->BDCR & RCC_BDCR_LSERDY)) {
			if (++t > 10000000) {
				PRINTF("RCC LSERDY Timeout ! BDCR = %08X\n", RCC->BDCR);
				break;
			}
		}
	}	break;

	case RTCSEL_LSI:
	{
	    PRINTF("Preparing RTC for LSI mode\n");
	    if ((RCC->BDCR & RCC_BDCR_RTCSEL_MASK) == RCC_BDCR_RTCSEL_LSI)
			break;
		RCC->BDCR = RCC_BDCR_BDRST; // Reset the entire Backup domain
		PRINTF("BCKP domain reset\n");
		RCC->BDCR = (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSI | RCC_BDCR_LSEBYP);
		PRINTF("RCC->BDCR = %08X\r\n", RCC->BDCR);

		RCC->CSR = RCC_CSR_LSION;
		uint32 t = 0;
		while (!(RCC->CSR & RCC_CSR_LSIRDY)) {
			if (++t > 10000000) {
				PRINTF("RCC LSIRDY Timeout ! CSR = %08X\n", RCC->CSR);
				goto end0;
			}
		}
		PRINTF("RCC->CSR = %08X\n", RCC->CSR);
	}   break;
	
	case RTCSEL_HSE :
	    PRINTF("Preparing RTC for HSE mode, RCC->BDCR = %08X\n", RCC->BDCR);
	    if ((RCC->BDCR & RCC_BDCR_RTCSEL_MASK) == RCC_BDCR_RTCSEL_HSE)
			break;
		RCC->BDCR = RCC_BDCR_BDRST; // Reset the entire Backup domain
		PRINTF("BCKP domain reset\n");
		RCC->BDCR = (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_HSE | RCC_BDCR_LSEBYP);
		PRINTF("RCC->BDCR = %08X\n", RCC->BDCR);
	    break;

	case RTCSEL_NONE:
	    PRINTF("Preparing RTC for NONE mode\n");
	    if ((RCC->BDCR & RCC_BDCR_RTCSEL_MASK) != RCC_BDCR_RTCSEL_NONE)
            RCC->BDCR = RCC_BDCR_BDRST; // Reset the entire Backup domain
	    //do nothing. Have a look at the clocks to see the diff between NONE and DEFAULT
	    goto end0;
		break;
    }
	if ( (sync_prescaler + async_prescaler) == 0) {
		sync_prescaler = prescalers[clk_src].s_presc;
		async_prescaler = prescalers[clk_src].as_presc;
	}
	PRINTF("sync_prescaler = %d, async_prescaler = %d\n", sync_prescaler, async_prescaler);
	rtc_enter_config_mode();
	RTC->PRER = (uint32)(async_prescaler << 16) + sync_prescaler;
	RTC->DR = 0x00002101; // reset value
	RTC->TR = 0x00000000; // reset value
    //RCC->CR |= RTC_CR_BYPSHAD;
	*bb_perip(&RTC->CR, RTC_CR_BYPSHAD_BIT) = 1; // bypass shadow regs
	PRINTF("RTC PRER: %08X, CR: %08X\n", RTC->PRER, RTC->CR);
    rtc_exit_config_mode();

end0:
    PRINTF("< RTClock::begin\n");
}

/*
RTClock::~RTClock() {
    //to implement
}
*/	
	
//-----------------------------------------------------------------------------
void RTClock::setTime (tm_t & tm)
{
    if (tm.year > 99)
        tm.year = tm.year % 100;
    rtc_dr = BUILD_DATE_REGISTER(tm.year, tm.month, tm.day, tm.weekday);
    rtc_tr = BUILD_TIME_REGISTER(tm.hour, tm.minute, tm.second);
    rtc_enter_config_mode();
    RTC->TR = rtc_tr;
    RTC->DR = rtc_dr;
    rtc_exit_config_mode();
	//getTimeStamp(); // fix wrong first read
    PRINTF("RTClock::setTime DR: %08X, TR: %08X\n", rtc_dr, rtc_tr);
}

//-----------------------------------------------------------------------------
void RTClock::setTime (time_t time_stamp)
{
	breakTime(time_stamp, _tm); // time will be broken to tm
    setTime(_tm);
}

/*============================================================================*/	
/* functions to convert to and from system time */
/* These are for interfacing with time serivces and are not normally needed in a sketch */

// leap year calulator expects year argument as years offset from 1970
#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )

//static const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31}; // API starts months from 1, this array starts from 0

//-----------------------------------------------------------------------------
void RTClock::breakTime(time_t timeInput, tm_t & tm)
{
// break the given time_t into time components
// this is a more compact version of the C library localtime function
// note that year is offset from 1970 !!!

	uint32_t time = (uint32_t)timeInput;
	tm.second = time % 60;
	time /= 60; // now it is minutes
	tm.minute = time % 60;
	time /= 60; // now it is hours
	tm.hour = time % 24;
	time /= 24; // now it is days
	tm.weekday = ((time + 4) % 7); // Monday is day 1 // (time + 4): Sunday is day 1 

	uint8_t year = 0;
	uint32_t days = 0;
	while((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) {
		year++;
	}
	tm.year = year; // year is offset from 1970 

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
	tm.month = month + 1;  // jan is month 1  
	tm.day = time + 1;     // day of month
}

//-----------------------------------------------------------------------------
time_t RTClock::makeTime(tm_t & tm)
{
// assemble time elements into time_t 
// note year argument is offset from 1970 (see macros in time.h to convert to other formats)
// previous version used full four digit year (or digits since 2000),i.e. 2009 was 2009 or 9
  
	// seconds from 1970 till 1 jan 00:00:00 of the given year
	uint32_t seconds = tm.year*(SECS_PER_DAY * 365);
	for (uint16_t i = 0; i < tm.year; i++) {
		if (LEAP_YEAR(i)) {
			seconds +=  SECS_PER_DAY;   // add extra days for leap years
		}
	}

	// add days for this year, months start from 1
	for (uint16_t i = 1; i < tm.month; i++) {
		if ( (i == 2) && LEAP_YEAR(tm.year)) { 
			seconds += SECS_PER_DAY * 29;
		} else {
			seconds += SECS_PER_DAY * monthDays[i-1];  //monthDay array starts from 0
		}
	}
	seconds+= (tm.day-1) * SECS_PER_DAY;
	seconds+= tm.hour * SECS_PER_HOUR;
	seconds+= tm.minute * SECS_PER_MIN;
	seconds+= tm.second;
	return (time_t)seconds; 
}

//-----------------------------------------------------------------------------
void RTClock::getTime(tm_t & tm)
{
	uint32 tr;
	do { // read multiple time till both readings are equal
		getTimeStamp();
		tr = rtc_tr;
		getTimeStamp();
	} while ( tr!=rtc_tr );
    PRINTF1("RTClock::getTime DR: %08X, TR: %08X\n", rtc_dr, rtc_tr);
	tm.year    = _year(rtc_dr);
    tm.month   = _month(rtc_dr);
    tm.day     = _day(rtc_dr);
    tm.weekday = _weekday(rtc_dr);
    tm.pm      = _pm(rtc_tr);
    tm.hour    = _hour(rtc_tr);
    tm.minute  = _minute(rtc_tr);
    tm.second  = _second(rtc_tr);
}

//-----------------------------------------------------------------------------
time_t RTClock::getTime()
{
	getTime(_tm);
	return makeTime(_tm);
}

//-----------------------------------------------------------------------------
void RTClock::setAlarmATime (tm_t * tm_ptr, bool hours_match, bool mins_match, bool secs_match, bool date_match)
{
    rtc_enter_config_mode();
    unsigned int bits = (bin2bcd(tm_ptr->day)<<24) + (bin2bcd(tm_ptr->hour)<<16) + 
			(bin2bcd(tm_ptr->minute)<<8) + bin2bcd(tm_ptr->second);
    if (!date_match) bits |= (1 << 31);
    if (!hours_match) bits |= (1 << 23);
    if (!mins_match) bits |= (1 << 15);
    if (!secs_match) bits |= (1 << 7);
    RTC->CR &= ~(RTC_CR_ALRAE);
    uint32 t = 0;
    while (!(RTC->ISR & RTC_ISR_ALRAWF)) {
       if (++t > 1000000) {
           PRINTF("RTC ALRAWF Timeout ! ISR = %08X\n", RTC->ISR);
           return;
       }
    }
    RTC->ALRMAR = bits;
    RTC->CR |= (RTC_CR_ALRAE  |RTC_CR_ALRAIE); // turn on ALRAIE
    rtc_exit_config_mode();
    nvic_irq_enable(NVIC_RTCALARM);
    nvic_irq_enable(NVIC_RTC);
    rtc_enable_alarm_event();
}

//-----------------------------------------------------------------------------
void RTClock::setAlarmATime (time_t alarm_time, bool hours_match, bool mins_match, bool secs_match, bool date_match)
{	
    breakTime(alarm_time, _tm);
    setAlarmATime(&_tm, hours_match, mins_match, secs_match, date_match);
}

//-----------------------------------------------------------------------------
void RTClock::turnOffAlarmA(void)
{
    rtc_enter_config_mode();
    RTC->CR &= ~(RTC_CR_ALRAIE); // turn off ALRAIE
    rtc_exit_config_mode();
}

//-----------------------------------------------------------------------------
void RTClock::setAlarmBTime (tm_t * tm_ptr, bool hours_match, bool mins_match, bool secs_match, bool date_match)
{
    rtc_enter_config_mode();
    unsigned int bits = (bin2bcd(tm_ptr->day) << 24) + (bin2bcd(tm_ptr->hour) << 16) + 
			(bin2bcd(tm_ptr->minute) << 8) + bin2bcd(tm_ptr->second);
    if (!date_match) bits |= (1 << 31);
    if (!hours_match) bits |= (1 << 23);
    if (!mins_match) bits |= (1 << 15);
    if (!secs_match) bits |= (1 << 7);
    RTC->CR &= ~(RTC_CR_ALRBE);
    uint32 t = 0;
    while (!(RTC->ISR & RTC_ISR_ALRBWF)) {
       if (++t > 1000000) {
           PRINTF("RTC ALRBWF Timeout ! ISR = %08X\n", RTC->ISR);
           return;
       }
    }
    RTC->ALRMBR = bits;
    RTC->CR |= (RTC_CR_ALRBE | RTC_CR_ALRBIE); // turn on ALRBIE
    rtc_exit_config_mode();
    nvic_irq_enable(NVIC_RTCALARM);
    nvic_irq_enable(NVIC_RTC);
    rtc_enable_alarm_event();
}

//-----------------------------------------------------------------------------
void RTClock::setAlarmBTime (time_t alarm_time, bool hours_match, bool mins_match, bool secs_match, bool date_match)
{	
    breakTime(alarm_time, _tm);
    setAlarmBTime(&_tm, hours_match, mins_match, secs_match, date_match);
}

//-----------------------------------------------------------------------------
void RTClock::turnOffAlarmB() {
    rtc_enter_config_mode();
    RTC->CR &= ~(RTC_CR_ALRBIE); // turn off ALRBIE
    rtc_exit_config_mode();
}

//-----------------------------------------------------------------------------
void RTClock::setPeriodicWakeup(uint16 period)
{
    PRINTF("< setPeriodicWakeup\n");
    rtc_enter_config_mode();
    RTC->CR &= ~(RTC_CR_WUTE);
    uint32 t = 0;
    while (!(RTC->ISR & RTC_ISR_WUTWF)) {
       if (++t > 1000000) {
           PRINTF("RTC WUTWF Timeout ! ISR = %08X\n", RTC->ISR);
           return;
       }
    }
    PRINTF("before setting RTC->WUTR\r\n");    
    RTC->WUTR = period; // set the period
    PRINTF("RTC->WUTR = %08X\r\n", RTC->WUTR);
    PRINTF("before setting RTC->CR.WUCKSEL\r\n");    
    RTC->CR &= ~(RTC_CR_WUCKSEL_MASK);
	RTC->CR |= 4; // Set the WUCKSEL to 1Hz (0x00000004)
	*bb_perip(&RTC->ISR, RTC_ISR_WUTF_BIT) = 0;
    RTC->CR |= RTC_CR_WUTE;
    if (period == 0)
        RTC->CR &= ~(RTC_CR_WUTIE); // if period is 0, turn off periodic wakeup interrupt.
    else {
        PRINTF("before turn ON RTC->CR.WUTIE\r\n");    
        RTC->CR |= (RTC_CR_WUTIE); // turn on WUTIE
    }
    PRINTF("RCC->CR = %08X\r\n", RCC->CR);
    rtc_exit_config_mode();
    rtc_enable_wakeup_event();
    nvic_irq_enable(NVIC_RTC);
    PRINTF("setPeriodicWakeup >\n");
}


void RTClock::attachAlarmAInterrupt(voidFuncPtr function) {
    handlerAlarmA = function;
}

void RTClock::detachAlarmAInterrupt() {
    handlerAlarmA = NULL;
}

void RTClock::attachAlarmBInterrupt(voidFuncPtr function) {
    handlerAlarmB = function;
}

void RTClock::detachAlarmBInterrupt() {
    handlerAlarmB = NULL;
}

void RTClock::attachPeriodicWakeupInterrupt(voidFuncPtr function) {
    handlerPeriodicWakeup = function;
}

void RTClock::detachPeriodicWakeupInterrupt() {
    handlerPeriodicWakeup = NULL;
}



extern "C" {
volatile uint32 rtc_tr, rtc_dr;
//-----------------------------------------------------------------------------
void __irq_rtc(void)
{
	PRINTF("<<__irq_rtc>>\n");
	rtc_enter_config_mode();
	*bb_perip(&RTC->ISR, RTC_ISR_WUTF_BIT) = 0;
	rtc_exit_config_mode();
	EXTI_BASE->PR = EXTI_RTC_WAKEUP;
	if (handlerPeriodicWakeup != NULL) {
		handlerPeriodicWakeup();
	}
}

//-----------------------------------------------------------------------------
void __irq_rtcalarm(void)
{
	bool isAlarmA = false;
	bool isAlarmB = false;
	PRINTF("<<__irq_rtcalarm>>\n");
	rtc_enter_config_mode();
	if (RTC->ISR & BIT(RTC_ISR_ALRAF_BIT)) {
		isAlarmA = true;
		PRINTF(">>AlarmA\n");
		*bb_perip(&RTC->ISR, RTC_ISR_ALRAF_BIT) = 0;
	}
	if (RTC->ISR & BIT(RTC_ISR_ALRBF_BIT)) {
		isAlarmB = true;
		PRINTF(">>AlarmB\n");
		*bb_perip(&RTC->ISR, RTC_ISR_ALRBF_BIT) = 0;
	}
	rtc_exit_config_mode();
	EXTI_BASE->PR = EXTI_RTC_ALARM;
	if (isAlarmA && handlerAlarmA != NULL) {
		handlerAlarmA();
	}
	if (isAlarmB && handlerAlarmB != NULL) {
		handlerAlarmB();
	}
}
//-----------------------------------------------------------------------------
} // extern "C"

