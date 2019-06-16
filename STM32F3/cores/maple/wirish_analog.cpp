/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
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
 * @file wirish_analog.cpp
 * @brief Wiring-style analogRead() implementation.
 */

#include <io.h>
#include <libmaple/adc.h>
#include <boards.h>



const adc_info adc_map[] = {
ADC1,  1, // D00/PA0 - ADC1_IN1
ADC1,  2, // D01/PA1 - ADC1_IN2
ADC1,  3, // D02/PA2 - ADC1_IN3
ADC1,  4, // D03/PA3 - ADC1_IN4
ADC2,  1, // D04/PA4 - ADC2_IN1
ADC2,  2, // D05/PA5 - ADC2_IN2
ADC2,  3, // D06/PA6 - ADC2_IN3
ADC2,  4, // D07/PA7 - ADC2_IN4
NULL,  0, // D08/PA8
NULL,  0, // D09/PA9
NULL,  0, // D10/PA10
NULL,  0, // D11/PA11
NULL,  0, // D12/PA12
NULL,  0, // D13/PA13
NULL,  0, // D14/PA14
NULL,  0, // D15/PA15

ADC3, 12, // D16/PB0 - ADC3_IN12
ADC3,  1, // D17/PB1 - ADC3_IN1
ADC2, 12, // D18/PB2 - ADC2_IN12
NULL,  0, // D19/PB3
NULL,  0, // D20/PB4
NULL,  0, // D21/PB5
NULL,  0, // D22/PB6
NULL,  0, // D23/PB7
NULL,  0, // D24/PB8
NULL,  0, // D25/PB9
NULL,  0, // D26/PB10
NULL,  0, // D27/PB11
ADC4,  3, // D28/PB12 - ADC4-IN3
ADC3,  5, // D29/PB13 - ADC3-IN5
ADC4,  4, // D30/PB14 - ADC4-IN4
ADC4,  5, // D31/PB15 - ADC4-IN5
};

// Assumes that the ADC has been initialized and the pin set to INPUT_ANALOG
uint16 analogRead(uint8 pin)
{
	const adc_info * info = &adc_map[pin];
	const adc_dev * dev = info->device;
    uint8 channel = info->channel;
    if ( dev==NULL || channel==NULL )
        return 0;

    return adc_read(dev, channel);
}
