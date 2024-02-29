/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010, LeafLabs, LLC.
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

#include "Servo.h"

#include <boards.h>
#include <io.h>
#include <pwm.h>
#include <wirish_math.h>

// 20 millisecond period config.  For a 1-based prescaler,
//
//    (prescaler * overflow / CYC_MSEC) msec = 1 timer cycle = 20 msec
// => prescaler * overflow = 20 * CYC_MSEC
//
// This picks the smallest prescaler that allows an overflow < 2^16.
#define MAX_OVERFLOW    ((1 << 16) - 1)
#define CYC_MSEC        (1000 * CYCLES_PER_MICROSECOND)
#define TAU_MSEC        20
#define TAU_USEC        (TAU_MSEC * 1000)
#define TAU_CYC         (TAU_MSEC * CYC_MSEC)
#define SERVO_PRESCALER (TAU_CYC / MAX_OVERFLOW + 1)
#define SERVO_OVERFLOW  ((uint16)round((double)TAU_CYC / SERVO_PRESCALER))

// Unit conversions
#define US_TO_COMPARE(us) ((uint16)map((us), 0, TAU_USEC, 0, SERVO_OVERFLOW))
#define COMPARE_TO_US(c)  ((uint32)map((c), 0, SERVO_OVERFLOW, 0, TAU_USEC))
#define ANGLE_TO_US(a)    ((uint16)(map((a), this->minAngle, this->maxAngle, \
                                        this->minPW, this->maxPW)))
#define US_TO_ANGLE(us)   ((int16)(map((us), this->minPW, this->maxPW,  \
                                       this->minAngle, this->maxAngle)))

Servo::Servo() { this->resetFields(); }

bool Servo::attach(uint8_t pin,
                   uint16_t minPW,
                   uint16_t maxPW,
                   int16_t minAngle,
                   int16_t maxAngle) 
{
    tDev = PinTimerDevice(pin);

    if (tDev == NULL) {
        // don't reset any fields or ASSERT(0), to keep driving any
        // previously attach()ed servo.
        return false;
    }

    if (this->attached()) {
        this->detach();
    }

    this->pin = pin;
    this->minPW = minPW;
    this->maxPW = maxPW;
    this->minAngle = minAngle;
    this->maxAngle = maxAngle;
    this->tChan = (timer_channel_t)PinTimerChannel(pin);

    pinMode(pin, PWM);

    timer_pause(tDev);
    timer_set_prescaler(tDev, SERVO_PRESCALER - 1); // prescaler is 1-based
    timer_set_reload(tDev, SERVO_OVERFLOW);
    timer_generate_update(tDev);
    timer_resume(tDev);

    return true;
}

bool Servo::detach()
{
    if (!this->attached()) {
        return false;
    }

    timer_set_mode(tDev, tChan, TIMER_DISABLED);
    this->resetFields();
    return true;
}

void Servo::write(int degrees)
{
    degrees = constrain(degrees, this->minAngle, this->maxAngle);
    this->writeMicroseconds(ANGLE_TO_US(degrees));
}

int Servo::read() const
{
    int a = US_TO_ANGLE(this->readMicroseconds());
    // map() round-trips in a weird way we mostly correct for here;
    // the round-trip is still sometimes off-by-one for write(1) and
    // write(179).
    return a == this->minAngle || a == this->maxAngle ? a : a + 1;
}

void Servo::writeMicroseconds(uint16_t pulseWidth)
{
    if (!this->attached()) {
        ASSERT(0);
        return;
    }
    pulseWidth = constrain(pulseWidth, this->minPW, this->maxPW);
    pwmWrite(this->pin, US_TO_COMPARE(pulseWidth));
}

uint16_t Servo::readMicroseconds() const
{
    if (!this->attached()) {
        ASSERT(0);
        return 0;
    }
    uint16_t compare = timer_get_compare(tDev, tChan);
    return COMPARE_TO_US(compare);
}

void Servo::resetFields(void)
{
    this->tDev = NULL;
    this->pin = NOT_ATTACHED;
    this->minAngle = SERVO_DEFAULT_MIN_ANGLE;
    this->maxAngle = SERVO_DEFAULT_MAX_ANGLE;
    this->minPW = SERVO_DEFAULT_MIN_PW;
    this->maxPW = SERVO_DEFAULT_MAX_PW;
}
