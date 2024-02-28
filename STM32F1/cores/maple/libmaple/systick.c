/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2010, 2011 LeafLabs, LLC.
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
 * @file libmaple/systick.c
 * @brief System timer (SysTick).
 */

#include <libmaple/systick.h>

volatile uint32 systick_uptime_millis;
// static void (*systick_user_callback)(void);
#define SYSTICK_CB_COUNT 4
static voidFuncPtr systick_user_callbacks[SYSTICK_CB_COUNT] = {NULL,};

/**
 * @brief Initialize and enable SysTick.
 *
 * Clocks the system timer with the core clock, turns it on, and
 * enables interrupts.
 *
 * @param reload_val Appropriate reload counter to tick every 1 ms.
 */
void systick_init(uint32 reload_val) {
    SYSTICK_BASE->RVR = reload_val;
    systick_enable();
}

/**
 * Clock the system timer with the core clock, but don't turn it
 * on or enable interrupt.
 */
void systick_disable() {
    SYSTICK_BASE->CSR = SYSTICK_CSR_CLKSOURCE_CORE;
}

/**
 * Clock the system timer with the core clock and turn it on;
 * interrupt every 1 ms, for systick_timer_millis.
 */
void systick_enable() {
    /* re-enables init registers without changing reload val */
    SYSTICK_BASE->CSR = (SYSTICK_CSR_CLKSOURCE_CORE   |
                         SYSTICK_CSR_ENABLE           |
                         SYSTICK_CSR_TICKINT_PEND);
}

/**
 * @brief Attach a callback to be called from the SysTick exception handler.
 *
 * To detach the callback, call detach function with the callback as argument.
 */
int8_t systick_attach_callback(voidFuncPtr callback) {
    // systick_user_callback = callback;
    for (uint8_t i = 0; i < SYSTICK_CB_COUNT; i++)
    {
        if (!systick_user_callbacks[i]) {
            systick_user_callbacks[i] = callback;
            return i;
        }
    }
    return -1;
}

int8_t systick_detach_callback(voidFuncPtr callback)
{
    // systick_user_callback = NULL;
    for (uint8_t i = 0; i < SYSTICK_CB_COUNT; i++)
    {
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
__weak void __exc_systick(void) {
    systick_uptime_millis++;
    for (uint8_t i = 0; i < SYSTICK_CB_COUNT; i++)
    {
        voidFuncPtr systick_callback = systick_user_callbacks[i];
        if (systick_callback)
            systick_callback();
    }
}
