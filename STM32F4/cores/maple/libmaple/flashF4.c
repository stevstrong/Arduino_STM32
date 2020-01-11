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
 * @file flash.c
 * @brief Flash management functions
 */

#include "libmaple.h"
#include "flashF4.h"
#include "bitband.h"

/**
 * @brief Turn on the hardware 'ART accelerator' i.e. prefetch + date & instruction cache
 */
void flash_enable_ART(void) {
	/* enable prefetch buffer, instruction cache and data cache */
	FLASH_BASE->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN ;
}

void flash_disable_ART(void) {
	/* disable flash prefetch and instruction, data cache */
	FLASH_BASE->ACR &= ~ (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
}

/**
 * @brief Set flash wait states
 *
 * See ST PM0042, section 3.1 for restrictions on the acceptable value
 * of wait_states for a given SYSCLK configuration.
 *
 * @param wait_states number of wait states (one of
 *                    FLASH_ACR_LATENCY_0WS .. FLASH_ACR_LATENCY_7WS
 */

void flash_set_latency(uint32 wait_states) {
    uint32 val = FLASH_BASE->ACR;

    val &= ~FLASH_ACR_LATENCY_Msk;
    val |= wait_states;

    FLASH_BASE->ACR = val;
}
