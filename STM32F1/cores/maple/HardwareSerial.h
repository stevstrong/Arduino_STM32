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
 * @file wirish/include/wirish/HardwareSerial.h
 * @brief Wirish serial port interface.
 */

#ifndef _WIRISH_HARDWARESERIAL_H_
#define _WIRISH_HARDWARESERIAL_H_

#include <libmaple/libmaple_types.h>

#include "Print.h"
#include "boards.h"
#include "Stream.h"
#include "libmaple/usart.h"
#include "libmaple/timer.h"
/*
 * IMPORTANT:
 *
 * This class documented "by hand" (i.e., not using Doxygen) in the
 * leaflabs-docs/ repository.
 *
 * If you alter the public HardwareSerial interface, you MUST update
 * the documentation accordingly.
 */

 
 
/* Roger Clark
 *
 * Added config defines from AVR 
 * Note. The values will need to be changed to match STM32 USART config register values, these are just place holders.
 */
// Define config for Serial.begin(baud, config);
// Note. STM32 doesn't support as many different Serial modes as AVR or SAM cores.
// The word legth bit M must be set when using parity bit.

#define SERIAL_8N1	0B00000000
#define SERIAL_8N2	0B00100000
#define SERIAL_9N1	0B00001000
#define SERIAL_9N2	0B00101000	

#define SERIAL_8E1	0B00001010
#define SERIAL_8E2	0B00101010
/* not supported:
#define SERIAL_9E1	0B00001010
#define SERIAL_9E2	0B00101010
*/
#define SERIAL_8O1	0B00001011
#define SERIAL_8O2	0B00101011
/* not supported:
#define SERIAL_9O1	0B00001011
#define SERIAL_9O2	0B00101011
*/

/* Roger Clark 
 * Moved macros from hardwareSerial.cpp
 */
 
#define DEFINE_HWSERIAL(name, n)                                   \
	HardwareSerial name(USART##n,                                  \
						BOARD_USART##n##_TX_PIN,                   \
						BOARD_USART##n##_RX_PIN)

#define DEFINE_HWSERIAL_UART(name, n)                             \
	HardwareSerial name(UART##n,                                  \
						BOARD_USART##n##_TX_PIN,                   \
						BOARD_USART##n##_RX_PIN)				


//-----------------------------------------------------------------------------
static inline void disable_timer_if_necessary(timer_dev *dev, uint8 ch)
{
    if (dev != NULL) {
        timer_set_mode(dev, ch, TIMER_DISABLED);
    }
}

//-----------------------------------------------------------------------------
/* Roger clark. Changed class inheritance from Print to Stream.
 * Also added new functions for peek() and availableForWrite()
 * Note. AvailableForWrite is only a stub function in the cpp
 */
//-----------------------------------------------------------------------------
class HardwareSerial : public Stream {

public:
	HardwareSerial(const usart_dev *usart_device, uint8 tx_pin, uint8 rx_pin)
    {
        this->usart_device = usart_device;
        this->tx_pin = tx_pin;
        this->rx_pin = rx_pin;
    }

    void begin(uint32 baud, uint8_t config)
    {
        disable_timer_if_necessary(PinTimerDevice(tx_pin), PinTimerChannel(tx_pin));

        usart_init(usart_device);
        usart_config(usart_device, rx_pin, tx_pin, config);
        usart_set_baud_rate(usart_device, baud);
        usart_enable(usart_device);
    }

    void begin(uint32 baud) {
    	begin(baud,SERIAL_8N1);
    }

    void end(void) {
    	usart_disable(usart_device);
    }

    virtual int available(void) {
        return usart_rx_available(usart_device);
    }

    virtual int peek(void) {
        return 0;//usart_peek(usart_device);
    }

    virtual int read(void) {
    	if(usart_rx_available(usart_device) > 0) {
    		return usart_getc(usart_device);
    	} else {
    		return -1;
    	}
    }

    int availableForWrite(void) {
        return usart_tx_available(usart_device);
    }

    virtual size_t write(uint8_t ch) {
        usart_putc(usart_device, ch);
    	return 1;
    }

    virtual void flush(void) {
        while(!rb_is_empty(usart_device->wb)); // wait for TX buffer empty
        while(!((usart_device->regs->SR) & (1<<USART_SR_TC_BIT))); // wait for TC (Transmission Complete) flag set
    }

    inline int write(unsigned long n) { return write((uint8_t)n); }
    inline int write(long n) { return write((uint8_t)n); }
    inline int write(unsigned int n) { return write((uint8_t)n); }
    inline int write(int n) { return write((uint8_t)n); }
    using Print::write;

    /* Pin accessors */
    int txPin(void) { return this->tx_pin; }
    int rxPin(void) { return this->rx_pin; }
	
	operator bool() { return true; }

    /* Escape hatch into libmaple */
    /* FIXME [0.0.13] documentation */
	const usart_dev* c_dev(void) { return this->usart_device; }
private:
    const usart_dev *usart_device;
    uint8 tx_pin;
    uint8 rx_pin;

};


#if BOARD_HAVE_USART1
extern HardwareSerial Serial1;
#endif
#if BOARD_HAVE_USART2
extern HardwareSerial Serial2;
#endif
#if BOARD_HAVE_USART3
extern HardwareSerial Serial3;
#endif
#if BOARD_HAVE_UART4
extern HardwareSerial Serial4;
#endif
#if BOARD_HAVE_UART5
extern HardwareSerial Serial5;
#endif
#if BOARD_HAVE_USART6
extern HardwareSerial Serial6;
#endif

#endif	//_WIRISH_HARDWARESERIAL_H_
