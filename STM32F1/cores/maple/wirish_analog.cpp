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
 * @file wirish/wirish_analog.cpp
 * @brief Wiring-style analogRead() implementation.
 */

#include "io.h"
#include <libmaple/adc.h>
#include "boards.h"
#include "wirish.h"


const uint8 adc_map[] = {
   0, /* D00/PA0  */
   1, /* D01/PA1  */
   2, /* D02/PA2  */
   3, /* D03/PA3  */
   4, /* D04/PA4  */
   5, /* D05/PA5  */
   6, /* D06/PA6  */
   7, /* D07/PA7  */
ADCx, /* D08/PA8  */
ADCx, /* D09/PA9  */
ADCx, /* D10/PA10 */
ADCx, /* D11/PA11 */
ADCx, /* D12/PA12 */
ADCx, /* D13/PA13 */
ADCx, /* D14/PA14 */
ADCx, /* D15/PA15 */

   8, /* D16/PB0  */
   9, /* D17/PB1  */
ADCx, /* D18/PB2  */
ADCx, /* D19/PB3  */
ADCx, /* D20/PB4  */
ADCx, /* D21/PB5  */
ADCx, /* D22/PB6  */
ADCx, /* D23/PB7  */
ADCx, /* D24/PB8  */
ADCx, /* D25/PB9  */
ADCx, /* D26/PB10 */
ADCx, /* D27/PB11 */
ADCx, /* D28/PB12 */
ADCx, /* D29/PB13 */
ADCx, /* D30/PB14 */
ADCx, /* D31/PB15 */

  10, /* D32/PC0  */
  11, /* D33/PC1  */
  12, /* D34/PC2  */
  13, /* D35/PC3  */
  14, /* D36/PC4  */
  15, /* D37/PC5  */
};

/* Unlike Wiring and Arduino, this assumes that the pin's mode is set
 * to INPUT_ANALOG. That's faster, but it does require some extra work
 * on the user's part. Not too much, we think ;). */
uint16 analogRead(uint8 pin)
{
#if STM32_NR_GPIO_PORTS>3
    if ( pin>PC5 ) {
#else
    if ( pin>PB1 ) {
#endif
        return 0;
    }
    uint8 adc_chan = pinToADCChannel(pin);
	if (adc_chan==ADCx)
		return 0;
    return adc_read(ADC1, adc_chan);
}
