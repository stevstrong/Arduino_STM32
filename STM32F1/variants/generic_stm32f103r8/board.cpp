/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
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
 * @file   wirish/boards/maple_mini/board.cpp
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief  Maple Mini board file.
 */

#include <board/board.h>

#include <libmaple/gpio.h>
#include <libmaple/timer.h>

/* Roger Clark. Added next to includes for changes to Serial */
#include <libmaple/usart.h>
#include <HardwareSerial.h>

#include <wirish_debug.h>
#include <wirish_types.h>

/* Since we want the Serial Wire/JTAG pins as GPIOs, disable both SW
 * and JTAG debug support, unless configured otherwise. */
void boardInit(void) {
#ifndef CONFIG_MAPLE_MINI_NO_DISABLE_DEBUG
    disableDebugPorts();
#endif
}

// Note. See the enum of pin names in board.h

extern const stm32_pin_info PIN_MAP[BOARD_NR_GPIO_PINS] = {

	
    {&gpioa, &timer2, 0, 1}, /* PA0 */
    {&gpioa, &timer2, 1, 2}, /* PA1 */
    {&gpioa, &timer2, 2, 3}, /* PA2 */
    {&gpioa, &timer2, 3, 4}, /* PA3 */
    {&gpioa,   NULL,  4, 0}, /* PA4 */	
    {&gpioa,   NULL,  5, 0}, /* PA5 */
    {&gpioa, &timer3, 6, 1}, /* PA6 */
    {&gpioa, &timer3, 7, 2}, /* PA7 */
    {&gpioa, &timer1, 8, 1}, /* PA8 */
    {&gpioa, &timer1, 9, 2}, /* PA9 */
    {&gpioa, &timer1,10, 3}, /* PA10 */
    {&gpioa,   NULL, 11, 0}, /* PA11 */
    {&gpioa,   NULL, 12, 0}, /* PA12 */	
    {&gpioa,   NULL, 13, 0}, /* PA13 */
    {&gpioa,   NULL, 14, 0}, /* PA14 */
    {&gpioa,   NULL, 15, 0}, /* PA15 */
	
    {&gpiob, &timer3, 0, 3}, /* PB0 */
    {&gpiob, &timer3, 1, 4}, /* PB1 */
    {&gpiob,   NULL,  2, 0}, /* PB2  */	
    {&gpiob,   NULL,  3, 0}, /* PB3  */
    {&gpiob,   NULL,  4, 0}, /* PB4  */
    {&gpiob,   NULL,  5, 0}, /* PB5 */
    {&gpiob, &timer4, 6, 1}, /* PB6 */
    {&gpiob, &timer4, 7, 2}, /* PB7 */
    {&gpiob, &timer4, 8, 3}, /* PB8 */
    {&gpiob,   NULL,  9, 0}, /* PB9 */
    {&gpiob,   NULL, 10, 0}, /* PB10 */
    {&gpiob,   NULL, 11, 0}, /* PB11 */
    {&gpiob,   NULL, 12, 0}, /* PB12 */
    {&gpiob,   NULL, 13, 0}, /* PB13 */
    {&gpiob,   NULL, 14, 0}, /* PB14 */
    {&gpiob,   NULL, 15, 0}, /* PB15 */

/* Andy Hull - the R8 is similar to the C8 but exposes more GPIO as follows */
    {&gpioc,   NULL,  0, 0}, /* PC0 */
    {&gpioc,   NULL,  1, 0}, /* PC1 */
    {&gpioc,   NULL,  2, 0}, /* PC2 */
    {&gpioc,   NULL,  3, 0}, /* PC3 */
    {&gpioc,   NULL,  4, 0}, /* PC4 */
    {&gpioc,   NULL,  5, 0}, /* PC5 */
    {&gpioc, &timer8, 6, 1}, /* PC6 */	
    {&gpioc, &timer8, 7, 2}, /* PC7 */
    {&gpioc, &timer8, 8, 3}, /* PC8 */
    {&gpioc, &timer8, 9, 4}, /* PC9 */
    {&gpioc,   NULL, 10, 0}, /* PC10 UART4_TX/SDIO_D2 */
    {&gpioc,   NULL, 11, 0}, /* PC11 UART4_RX/SDIO_D3 */
    {&gpioc,   NULL, 12, 0}, /* PC12 UART5_TX/SDIO_CK */	
    {&gpioc,   NULL, 13, 0}, /* PC13 TAMPER-RTC */
    {&gpioc,   NULL, 14, 0}, /* PC14 OSC32_IN */
    {&gpioc,   NULL, 15, 0}, /* PC15 OSC32_OUT */

    {&gpiod,   NULL,  2, 0} , /* PD2  TIM3_ETR/UART5_RX SDIO_CMD */
};

/*  Basically everything that is defined as having a timer us PWM */
const uint8 boardPWMPins[] __FLASH__ = {
    PA0,PA1,PA2,PA3,PA6,PA7,PA8,PA9,PA10,PB0,PB1,PB6,PB7,PB8,PB9,PC6,PC7,PC8,PC9
};

/*  Basically everything that is defined having ADC */
const uint8 boardADCPins[] __FLASH__ = {
    PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,PB0,PB1,PC0,PC1,PC2,PC3,PC4,PC5
};

// Note. These defines are not really used by generic boards. They are for  Maple Serial USB
#define USB_DP PA12
#define USB_DM PA11

// NOte. These definitions are not really used for generic boards, they only relate to boards modified to behave like Maple boards
extern const uint8 boardUsedPins[BOARD_NR_USED_PINS] __FLASH__ = {
     USB_DP, USB_DM
};


/* 
 * Roger Clark
 * 
 * 2015/05/28
 *
 * Moved definitions for Hardware Serial devices from HardwareSerial.cpp so that each board can define which Arduino "Serial" instance
 * Maps to which hardware serial port on the microprocessor
 */

DEFINE_HWSERIAL(Serial1, 1);
DEFINE_HWSERIAL(Serial2, 2);
DEFINE_HWSERIAL(Serial3, 3);
