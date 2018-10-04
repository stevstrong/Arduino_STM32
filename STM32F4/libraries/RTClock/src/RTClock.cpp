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


voidFuncPtr handlerAlarmA = NULL;
voidFuncPtr handlerAlarmB = NULL;
voidFuncPtr handlerPeriodicWakeup = NULL;


#ifdef RTC_DEBUG
  char dbg_s[200];
  #define PRINTF(...) { sprintf(dbg_s, __VA_ARGS__); Serial.print(dbg_s); }
#else
  #define PRINTF(...) 
#endif
//-----------------------------------------------------------------------------
// Clear the register synchronized flag. The flag is then set by hardware after a write to PRL/DIV or CNT.
//-----------------------------------------------------------------------------
static inline void rtc_clear_sync() {
	*bb_perip(&RTC->ISR, RTC_ISR_RSF_BIT) = 0;
}

//-----------------------------------------------------------------------------
// Check (wait if necessary) to see RTC registers are synchronized.
//-----------------------------------------------------------------------------
//static void rtc_wait_sync()
//{
//	rtc_debug_printf("< rtc_wait_sync\n");
//	uint32 t = 0;
//	while ( !(RTC->ISR & RTC_ISR_RSF) ) 
//	{
//	    if (++t > 1000000) {
//			rtc_debug_printf("Sync Timeout ! ISR = %08X\n", RTC->ISR);
//			break;
//		}
//	}
//	rtc_debug_printf("rtc_wait_sync >\n");
//}

//-----------------------------------------------------------------------------
// Enter configuration mode.
//-----------------------------------------------------------------------------
static void rtc_enter_config_mode()
{
	PRINTF("< rtc_enter_config_mode\n");
	// Unlock Write Protect
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	PRINTF("RTC->ISR(1) = %08X\n", RTC->ISR);
	*bb_perip(&RTC->ISR, RTC_ISR_INIT_BIT) = 1;
	PRINTF("RTC->ISR(2) = %08X\n", RTC->ISR);
	uint32 t = 0;
	while (!(RTC->ISR & RTC_ISR_INITF))
	{
	    if (++t > 1000000) {
			PRINTF("RTC->ISR.INITF Timeout ! ISR = %08X\n", RTC->ISR);
			break;;
		}
	}
	PRINTF("rtc_enter_config_mode >\n");
}

//-----------------------------------------------------------------------------
// Exit configuration mode.
//-----------------------------------------------------------------------------
static inline void rtc_exit_config_mode() {
	*bb_perip(&RTC->ISR, RTC_ISR_INIT_BIT) = 0;
//	PRINTF("rtc_exit_config_mode done !\r\n");
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

//-----------------------------------------------------------------------------
// @brief Disable the RTC alarm event.
//-----------------------------------------------------------------------------
//static void rtc_disable_wakeup_event()
//{
//	EXTI_BASE->EMR  &= ~(EXTI_RTC_WAKEUP);
//	EXTI_BASE->RTSR &= ~(EXTI_RTC_WAKEUP);
//}

//-----------------------------------------------------------------------------
RTClock::RTClock(rtc_clk_src clk_src, uint16 sync_presc, uint16 async_presc)
{
	src = clk_src;
	sync_prescaler = sync_presc;
	async_prescaler = async_presc;
}
//-----------------------------------------------------------------------------
void RTClock::begin(void)
{
    PRINTF("< RTClock::begin\n");
	
    bkp_init();		// turn on peripheral clocks to PWR and BKP and reset the backup domain via RCC registers.
                        // (we must reset the backup domain here in order to change the rtc clock source).
    PRINTF("bkp_disable_writes\n");
    bkp_disable_writes();
    PRINTF("bkp_enable_writes\n");
    bkp_enable_writes();	// enable writes to the backup registers and the RTC registers via the DBP bit in the PWR control register
    PRINTF("PWR->CR = %08X\n", PWR->CR);
    rcc_set_rtc_prescaler(CRYSTAL_FREQ); // Set the RTCPRE to HSE / 8.
    PRINTF("RCC->CFGR = %08X\n", RCC->CFGR);

	rtc_enter_config_mode();
    switch (src)
	{	
	case RTCSEL_LSE:
	{	
		PRINTF("Preparing RTC for LSE mode, RCC->BDCR = %08X\n", RCC->BDCR);
	    if ((RCC->BDCR & RCC_BDCR_RTCSEL_MASK) != RCC_BDCR_RTCSEL_LSE) {
            RCC->BDCR = RCC_BDCR_BDRST; // Reset the entire Backup domain
			PRINTF("BCKP domain reset\n");
		}
		RCC->BDCR = (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSE | RCC_BDCR_LSEON);
		PRINTF("RCC->BDCR = %08X\n", RCC->BDCR);
		uint32 t = 0;
		while (!(RCC->BDCR & RCC_BDCR_LSERDY)) {
			if (++t > 1000000) {
				PRINTF("RCC LSERDY Timeout ! BDCR = %08X\n", RCC->BDCR);
				goto end0;
			}
		}
		PRINTF("RCC->BDCR = %08X\r\n", RCC->BDCR);
		if (sync_prescaler == 0 && async_prescaler == 0)
			RTC->PRER = 255 | (127 << 16);
		else
			RTC->PRER = sync_prescaler | (async_prescaler << 16);
	}	break;
	case RTCSEL_LSI:
	{
	    PRINTF("Preparing RTC for LSI mode\n");
	    if ((RCC->BDCR & RCC_BDCR_RTCSEL_MASK) != RCC_BDCR_RTCSEL_LSI) {
            RCC->BDCR = RCC_BDCR_BDRST; // Reset the entire Backup domain
			PRINTF("BCKP domain reset\n");
		}
		RCC->BDCR = (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSI | RCC_BDCR_LSEBYP);
		PRINTF("RCC->BDCR = %08X\r\n", RCC->BDCR);
		RCC->CSR = RCC_CSR_LSION;
		uint32 t = 0;
		while (!(RCC->CSR & RCC_CSR_LSIRDY)) {
			if (++t > 1000000) {
				PRINTF("RCC LSIRDY Timeout ! CSR = %08X\n", RCC->CSR);
				goto end0;
			}
		}
		PRINTF("RCC->CSR = %08X\n", RCC->CSR);
		if (sync_prescaler == 0 && async_prescaler == 0)
			RTC->PRER = 249 | (127 << 16);
		else
			RTC->PRER = sync_prescaler | (async_prescaler << 16);
	}   break;
	case RTCSEL_DEFAULT: 
	case RTCSEL_HSE : 
	    PRINTF("Preparing RTC for HSE mode, RCC->BDCR = %08X\n", RCC->BDCR);
	    if ((RCC->BDCR & RCC_BDCR_RTCSEL_MASK) != RCC_BDCR_RTCSEL_HSE) {
            RCC->BDCR = RCC_BDCR_BDRST; // Reset the entire Backup domain
			PRINTF("BCKP domain reset\n");
		}
		RCC->BDCR = (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_HSE | RCC_BDCR_LSEBYP);
		PRINTF("RCC->BDCR = %08X\n", RCC->BDCR);
		if (sync_prescaler == 0 && async_prescaler == 0)
			RTC->PRER = 7999 | (124 << 16);
		else
			RTC->PRER = sync_prescaler | (async_prescaler << 16);
	    break;
	case RTCSEL_NONE:
	    PRINTF("Preparing RTC for NONE mode\n");
	    if ((RCC->BDCR & RCC_BDCR_RTCSEL_MASK) != RCC_BDCR_RTCSEL_NONE)
            RCC->BDCR = RCC_BDCR_BDRST; // Reset the entire Backup domain
	    RCC->BDCR = RCC_BDCR_RTCSEL_NONE;
	    //do nothing. Have a look at the clocks to see the diff between NONE and DEFAULT
	    break;
    }
    RCC->CR = (RTC_CR_FMT | RTC_CR_BYPSHAD); // 24hrs mode +  bypass shadow regs

end0:
    rtc_exit_config_mode();
    PRINTF("RTClock::begin >\n");
}

/*
RTClock::~RTClock() {
    //to implement
}
*/	
	
//-----------------------------------------------------------------------------
void RTClock::setTime (time_t time_stamp)
{
	breakTime(time_stamp, tm); // time will be broken to tm
    setTime(tm);
}

//-----------------------------------------------------------------------------
void RTClock::setTime (tm_t & tm)
{
    if (tm.year > 99)
        tm.year = tm.year % 100;
    rtc_enter_config_mode();
    RTC->TR = BUILD_TIME_REGISTER(tm.hour, tm.minute, tm.second);
    RTC->DR = BUILD_DATE_REGISTER(tm.year, tm.month, tm.day, tm.weekday);
    rtc_exit_config_mode();		                
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
	tm.weekday = ((time + 4) % 7); // Monday is day 1 // + 1;  // Sunday is day 1 

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
    uint32_t dr_reg, tr_reg;
	do { // read multiple time till both readings are equal
		dr_reg = getDReg();
		tr_reg = getTReg();
	} while ( (dr_reg!=getDReg()) || (tr_reg!=getTReg()) );
	tm.year    = _year(dr_reg);
    tm.month   = _month(dr_reg);
    tm.day     = _day(dr_reg);
    tm.weekday = _weekday(dr_reg);
    tm.pm      = _pm(tr_reg);
    tm.hour    = _hour(tr_reg);
    tm.minute  = _minute(tr_reg);
    tm.second  = _second(tr_reg);
}

//-----------------------------------------------------------------------------
time_t RTClock::getTime()
{
	getTime(tm);
	return makeTime(tm);
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
    breakTime(alarm_time, tm);
    setAlarmATime(&tm, hours_match, mins_match, secs_match, date_match);
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
    breakTime(alarm_time, tm);
    setAlarmBTime(&tm, hours_match, mins_match, secs_match, date_match);
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
    RTC->CR &= ~(3); RTC->CR |= 4; // Set the WUCKSEL to 1Hz (0x00000004)
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

