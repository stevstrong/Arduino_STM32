/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Bryan Newbold.
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

#include "HardwareTimer.h"
#include "boards.h"             // for CYCLES_PER_MICROSECOND


#define NR_TIMERS 14

#define MAX_RELOAD ((1 << 16) - 1)

static timer_dev * const devs[] = {
	TIMER1,
	TIMER2,
	TIMER3,
	TIMER4,
	TIMER5,
	TIMER6,
	TIMER7,
	TIMER8,
	TIMER9,
	TIMER10,
	TIMER11,
	TIMER12,
	TIMER13,
	TIMER14,
};

HardwareTimer::HardwareTimer(uint8 timerNum)
{
    if (timerNum > NR_TIMERS) {
        ASSERT(0);
    }
    this->dev = devs[timerNum - 1];
}


uint16 HardwareTimer::setPeriod(uint32 microseconds)
{
    // Not the best way to handle this edge case?
    if (!microseconds) {
        this->setPrescaleFactor(1);
        this->setOverflow(1);
        return this->getOverflow();
    }

    uint32 period_cyc = microseconds * CYCLES_PER_MICROSECOND / 2;
    uint16 prescaler = (uint16)(period_cyc / MAX_RELOAD + 1);
    uint16 overflow = (uint16)round(period_cyc / prescaler);
    this->setPrescaleFactor(prescaler);
    this->setOverflow(overflow);
    return overflow;
}

// Predefined instances

HardwareTimer Timer1(1);
HardwareTimer Timer2(2);
HardwareTimer Timer3(3);
HardwareTimer Timer4(4);
HardwareTimer Timer5(5);
HardwareTimer Timer6(6);
HardwareTimer Timer7(7);
HardwareTimer Timer8(8);
HardwareTimer Timer9(9);
HardwareTimer Timer10(10);
HardwareTimer Timer11(11);
HardwareTimer Timer12(12);
HardwareTimer Timer13(13);
HardwareTimer Timer14(14);
