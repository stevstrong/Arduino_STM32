/*
 * Modified by LeafLabs, LLC.
 *
 * Part of the Wiring project - http://wiring.org.co Copyright (c)
 * 2004-06 Hernando Barragan Modified 13 August 2006, David A. Mellis
 * for Arduino - http://www.arduino.cc/
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

// #include <stdlib.h>
// #include "math.h"
#include "ext_interrupts.h"

//------------------------------------------------------------------------------
// Custom implementation to avoid exception fault with rand() using newlib.c
// https://github.com/2cats/STM32/blob/master/template/probe/inc/lib_math.h
//------------------------------------------------------------------------------
#define RAND_SEED_INIT_VAL 1u
#define RAND_LCG_PARAM_M 0x7FFFFFFFu   /* See Note #1b2B. */
#define RAND_LCG_PARAM_A 1103515245u   /* See Note #1b1A2. */
#define RAND_LCG_PARAM_B 12345u   /* See Note #1b1A3. */
#define RAND_NBR unsigned int
RAND_NBR Math_RandSeedCur = RAND_SEED_INIT_VAL; // Cur rand nbr seed.

static inline RAND_NBR Math_RandSeed(RAND_NBR  seed)
{
    return (((RAND_NBR)RAND_LCG_PARAM_A * seed) + (RAND_NBR)RAND_LCG_PARAM_B) % ((RAND_NBR)RAND_LCG_PARAM_M + 1u);
}

RAND_NBR  Math_Rand (void)
{
    // CPU_SR_ALLOC();
    nvic_globalirq_disable(); // CPU_CRITICAL_ENTER();
    RAND_NBR seed = Math_RandSeedCur;
    RAND_NBR rand_nbr = Math_RandSeed(seed);
    Math_RandSeedCur = rand_nbr;
    nvic_globalirq_enable(); // CPU_CRITICAL_EXIT();

    return (rand_nbr);
}

//------------------------------------------------------------------------------
void randomSeed(unsigned int seed) {
    if (seed != 0) {
        // srand(seed);
        Math_RandSeed(seed);
    }
}

long random(long howbig) {
    if (howbig == 0) {
        return 0;
    }
    // return rand() % howbig;
    return Math_Rand() % howbig;
}

long random(long howsmall, long howbig) {
    if (howsmall >= howbig) {
        return howsmall;
    }

    long diff = howbig - howsmall;
    return random(diff) + howsmall;
}

