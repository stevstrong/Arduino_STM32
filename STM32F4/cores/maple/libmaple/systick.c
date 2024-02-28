/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
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
 * @file systick.c
 * @brief System timer interrupt handler and initialization routines
 */

#include "systick.h"
#include "nvic.h"
volatile uint32 systick_uptime_millis;
#define MAX_USR_CALLBACKS 4
static voidFuncPtr systick_user_callbacks[MAX_USR_CALLBACKS];
static uint8_t systick_usr_cb_count;

systick_reg_map * const SYSTICK = SYSTICK_BASE;

/**
 * @brief Initialize and enable SysTick.
 *
 * Clocks the system timer with the core clock, turns it on, and
 * enables interrupts.
 *
 * @param reload_val Appropriate reload counter to tick every 1 ms.
 */
void systick_init(uint32 reload_val) {
    SYSTICK->LOAD = reload_val;
    systick_usr_cb_count = 0;
    systick_enable();
}

/**
 * Clock the system timer with the core clock, but don't turn it
 * on or enable interrupt.
 */
void systick_disable() {
    SYSTICK->CTRL = SYSTICK_CTRL_CLKSOURCE_CORE;
}

/**
 * Clock the system timer with the core clock and turn it on;
 * interrupt every 1 ms, for systick_timer_millis.
 */
void systick_enable() {
    /* re-enables init registers without changing reload val */
    SYSTICK->CTRL = (SYSTICK_CTRL_CLKSOURCE_CORE   |
                         SYSTICK_CTRL_ENABLE           |
                         SYSTICK_CTRL_TICKINT_PENDING);
}

/**
 * @brief Attach a callback to be called from the SysTick exception handler.
 *
 * To detach a callback, call this function again with the callback as parameter.
 * Allows a maximum of 4 callbacks.
 */
int8_t systick_attach_callback(voidFuncPtr callback)
{
    if (!callback) return -1;
    for (uint8_t i = 0; i < MAX_USR_CALLBACKS; i ++) {
        if (systick_user_callbacks[i] == 0) {
            systick_user_callbacks[i] = callback;
            return i;
        }
    }
    return -1;
}

int8_t systick_detach_callback(voidFuncPtr callback)
{
    if (callback == NULL) return -1;
    for (uint8_t i = 0; i < MAX_USR_CALLBACKS; i ++) {
        if (systick_user_callbacks[i] == callback) {
            systick_user_callbacks[i] = NULL;
            return i;
        }
    }
    return -1;
}

/*
 * SysTick ISR
 */

void __exc_systick(void)
{
    nvic_globalirq_disable();
	systick_check_underflow();
	systick_uptime_millis++;
    nvic_globalirq_enable();

    for (uint8_t i = 0; i < 4; i ++) {
        if (systick_user_callbacks[i]) {
            systick_user_callbacks[i]();
        }
    }
}
