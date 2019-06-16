/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2013 OpenMusicKontrollers.
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
 * @file   wirish/boards/48F3/board.c
 * @author F3-port: Hanspeter Portner <dev@open-music-kontrollers.ch>
 * @brief  F303xx board file (F303CB, F303CC, F303RB, F303RC, F303VB, F303VC).
 */

#include "board/board.h"

#include "libmaple/gpio.h"
#include "libmaple/timer.h"

#include "wirish_debug.h"
#include "wirish_types.h"

/* Since we want the Serial Wire/JTAG pins as GPIOs, disable both SW
 * and JTAG debug support on the packages with low pin-count (e.g. MEDIUM_DENSITY),
 * unless configured otherwise. */
void boardInit(void) {
#if defined(STM32_MEDIUM_DENSITY)
    //disableDebugPorts();
#endif
    enableDebugPorts();

}


//extern const uint8 boardPWMPins[BOARD_NR_PWM_PINS] __FLASH__ = {
//		PA0, PA1, PA2, PA3, PA4, PA6, PA7, PB0, PB1, PB10, PB11, PB14, PB15, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15, PB3, PB4, PB5, PB6, PB7, PB8, PB9,
//#if defined(STM32_HIGH_DENSITY) || defined(STM32_XL_DENSITY)
//		PC6, PC7, PC8, PC9,
//#	if defined(STM32_XL_DENSITY)
//		PE2, PE3, PE4, PE5, PF9, PF10, PE9, PE11, PE13, PE14, PD12, PD13, PD14, PD15, PF6, PD1, PD4, PD6, PD7, PE0, PE1,
//#	endif
//#endif
//};
//
//extern const uint8 boardADCPins[BOARD_NR_ADC_PINS] __FLASH__ = {
//		PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PB2, PB12, PB13, PB14, PB15,
//#if defined(STM32_HIGH_DENSITY) || defined(STM32_XL_DENSITY)
//		PC0, PC1, PC2, PC3, PF4, PC4, PC5, 
//#	if defined(STM32_XL_DENSITY)
//		PF2, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15, PD8, PD9, PD10, PD11, PD12, PD13, PD14, 
//#	endif
//#endif
//};

extern const uint8 boardUsedPins[BOARD_NR_USED_PINS] __FLASH__ = {
    OSC_IN, OSC_OUT, USB_DP, USB_DM //, PB3, PA13, PA14
};
