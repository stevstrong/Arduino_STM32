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
 * @brief USB virtual serial terminal
 */

#include "usb_serial.h"

#include "string.h"
#include "stdint.h"

#include <libmaple/nvic.h>
#include <libmaple/usb/usb.h>
#include <libmaple/iwdg.h>
#include <libmaple/bkp.h>
#include <libmaple/ring_buffer.h>
#include "wirish.h"


#if BOARD_HAVE_SERIALUSB

/*
 * Hooks used for bootloader reset signalling
 */

static void rxHook();
static void ifaceSetupHook();


/*
 * USBSerial interface
 */

#define USB_TIMEOUT 50


void USBSerial::begin(void)
{
    if (_hasBegun)
        return;
    _hasBegun = true;

    // toggle to force re-enumeration
	gpio_set_mode(USB_DP, GPIO_OUTPUT);
	gpio_clear_pin(USB_DP);
	delay_us(10000);

	// setup the GPIOs
	gpio_set_mode(USB_DP, GPIO_AF_OUTPUT);
	gpio_set_mode(USB_DM, GPIO_AF_OUTPUT);
	gpio_set_af(USB_DP, GPIO_AF_USB);
	gpio_set_af(USB_DM, GPIO_AF_USB);

	usb_cdcacm_enable();
	usb_cdcacm_set_hooks(USB_CDCACM_HOOK_RX, rxHook);
	usb_cdcacm_set_hooks(USB_CDCACM_HOOK_IFACE_SETUP, ifaceSetupHook);
}

//Roger Clark. Two new begin functions has been added so that normal Arduino Sketches that use Serial.begin(xxx) will compile.
void USBSerial::begin(unsigned long ignoreBaud)
{
	(void)ignoreBaud;
	begin();
}
void USBSerial::begin(unsigned long ignoreBaud, uint8_t ignore)
{
	(void)ignoreBaud;
	(void)ignore;
	begin();
}

void USBSerial::end(void)
{
    usb_cdcacm_disable();
    usb_cdcacm_remove_hooks(USB_CDCACM_HOOK_RX | USB_CDCACM_HOOK_IFACE_SETUP);
	_hasBegun = false;
}

size_t USBSerial::write(uint8 ch) {
    return this->write(&ch, 1);
}

size_t USBSerial::write(const char *str) {
    return this->write((const uint8*)str, strlen(str));
}

size_t USBSerial::write(const uint8 *buf, uint32 len)
{
#ifdef USB_SERIAL_REQUIRE_DTR
    if (!(bool) *this || !buf) {
        return 0;
    }
#else	
	if (!buf || !(usb_is_connected() && usb_is_configured())) {
        return 0;
    }
#endif	

    uint32 txed = 0;
	if (!isBlocking) 	{
		txed = usb_cdcacm_tx((const uint8*)buf + txed, len - txed);
	}
	else {
		while (txed < len) {
			txed += usb_cdcacm_tx((const uint8*)buf + txed, len - txed);
		}
	}

	return txed;
}

int USBSerial::peek(void)
{
    uint8 b;
	if (usb_cdcacm_peek(&b, 1)==1)
	{
		return b;
	}
	else
	{
		return -1;
	}
}

void USBSerial::flush(void)
{
	usb_cdcacm_rx_flush();
}

// blocks till the number of bytes is received
uint32 USBSerial::read(uint8 * buf, uint32 len) {
    uint32 rxed = 0;
    while (rxed < len) {
        rxed += usb_cdcacm_rx(buf + rxed, len - rxed);
    }

    return rxed;
}

size_t USBSerial::readBytes(char *buf, const size_t len)
{
    size_t rxed=0;
    unsigned long startMillis;
    startMillis = millis();
    if (len <= 0) return 0;
    do {
        rxed += usb_cdcacm_rx((uint8 *)buf + rxed, len - rxed);
        if (rxed == len) return rxed;
    } while( (millis() - startMillis) < _timeout);
    return rxed;
}


USBSerial::operator bool() {
    //return usb_is_connected() && usb_is_configured() && usb_cdcacm_get_dtr();
	return usb_is_ready();
}



#ifdef SERIAL_USB
	USBSerial Serial;
#endif

/*
 * Bootloader hook implementations
 */

enum reset_state_t {
    DTR_UNSET,
    DTR_HIGH,
    DTR_NEGEDGE,
    DTR_LOW
};

static reset_state_t reset_state = DTR_UNSET;

static void ifaceSetupHook()
{
	// We need to see a negative edge on DTR before we start looking
	// for the in-band magic reset byte sequence.
	if ( usb_cdcacm_get_dtr() ) {
		reset_state = DTR_HIGH;
	} else {
		reset_state = (reset_state==DTR_HIGH) ? DTR_NEGEDGE : DTR_LOW;
	}
}

// The magic reset sequence is "1EAF".
static const uint8 magic[4] = {'1', 'E', 'A', 'F'};	
static void rxHook()
{
    /* FIXME this is mad buggy; we need a new reset sequence. E.g. NAK
     * after each RX means you can't reset if any bytes are waiting. */
    if (reset_state == DTR_NEGEDGE) {
        int len = usb_cdcacm_read_available();
        if (len >= 4) 
        {
            uint8 chkBuf[256]; // max USB data buffer, to be sure to get all the data

            // Peek at the waiting bytes, looking for reset sequence,
            // bailing on mismatch.
            usb_cdcacm_peek(chkBuf, len);

			for (uint32 i = 0; i < sizeof(magic); i++) {
				if (chkBuf[len + i - 4] != magic[i])
				{
					reset_state = DTR_LOW; // disable further check till next NEGEDGE
					return;
				}
			}

			// Got the magic sequence -> reset, presumably into the bootloader.
			bkp_init();
			bkp_enable_writes();
			bkp_write(16, 0x424C);
			bkp_disable_writes();
			// prepare to shut down
			extern void USB_power_off(void);
			USB_power_off();
			// reset
			nvic_sys_reset();			
			/* Can't happen. */
			while(1);
        }
    }
}

#endif  // BOARD_HAVE_SERIALUSB
