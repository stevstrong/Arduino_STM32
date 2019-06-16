/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2012 LeafLabs, LLC.
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

/*
 * Arduino-compatible digital I/O implementation.
 */

#include <io.h>

#include <libmaple/gpio.h>
#include <libmaple/timer.h>

#include <wirish_time.h>
#include <boards.h>


void pinMode(uint8 pin, WiringPinMode w_mode) {
    if (pin >= LAST_GPIO_PIN) {
        return;
    }

    gpio_pin_mode mode;
    // People always do the silly pin-toggle speed benchmark, so let's
    // accomodate them:
    bool pwm = false;
    switch(w_mode) {
    case OUTPUT:
        mode = GPIO_OUTPUT;
        break;
    case OUTPUT_OPEN_DRAIN:
        mode = GPIO_OUTPUT_OD;
        break;
    case INPUT:
    case INPUT_FLOATING:
        mode = GPIO_INPUT;
        break;
    case INPUT_ANALOG:
        mode = GPIO_INPUT_ANALOG;
        break;
    case INPUT_PULLUP:
        mode = GPIO_INPUT_PU;
        break;
    case INPUT_PULLDOWN:
        mode = GPIO_INPUT_PD;
        break;
    case PWM:
        mode = GPIO_AF_OUTPUT;
        pwm = true;
        break;
    case PWM_OPEN_DRAIN:
        mode = GPIO_AF_OUTPUT_OD;
        pwm = true;
        break;
    default:
        ASSERT(0);              // Can't happen
        return;
    }

    if (pwm && (pin<=PB15)) { // PC has no timers, timer_map only has PA and PB values
		const timer_info *info = &timer_map[pin];
        /* If enabling PWM, tell the timer to do PWM, and tell the pin
         * to listen to the right timer. */
        if (info->index == NULL) {
            return;
        }
        timer_set_mode(timer_devices[info->index], info->channel, TIMER_PWM);
        gpio_set_af(pin, (gpio_af)info->af);
    }
    gpio_set_mode(pin, mode);
}

uint32 digitalRead(uint8 pin) {
    if (pin >= LAST_GPIO_PIN) {
        return 0;
    }

    return gpio_read_bit(digitalPinToPort(pin), digitalPinToBit(pin)) ?
        HIGH : LOW;
}

void digitalWrite(uint8 pin, uint8 val) {
    if (pin >= LAST_GPIO_PIN) {
        return;
    }

    gpio_write_bit(digitalPinToPort(pin), digitalPinToBit(pin), val);
}

void togglePin(uint8 pin) {
    if (pin >= LAST_GPIO_PIN) {
        return;
    }

    gpio_toggle_bit(digitalPinToPort(pin), digitalPinToBit(pin));
}

#define BUTTON_DEBOUNCE_DELAY 1

uint8 isButtonPressed(uint8 pin, uint32 pressedLevel) {
    if (digitalRead(pin) == pressedLevel) {
        delay(BUTTON_DEBOUNCE_DELAY);
        while (digitalRead(pin) == pressedLevel)
            ;
        return true;
    }
    return false;
}

uint8 waitForButtonPress(uint8 pin, uint32 timeout) {
    uint32 start = millis();
    uint32 time;
    if (timeout == 0) {
        while (!isButtonPressed(pin))
            ;
        return true;
    }
    do {
        time = millis();
        /* properly handle wrap-around */
        if ((start > time && time + (0xffffffffU - start) > timeout) ||
            time - start > timeout) {
            return false;
        }
    } while (!isButtonPressed(pin));
    return true;
}
