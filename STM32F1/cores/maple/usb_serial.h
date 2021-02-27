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
 * @brief Wirish USB virtual serial port (SerialUSB).
 */

#ifndef _WIRISH_USB_SERIAL_H_
#define _WIRISH_USB_SERIAL_H_

#include <libmaple/usb/usb.h>
#include "Print.h"
#include "boards.h"
#include "Stream.h"

#if BOARD_HAVE_SERIALUSB

/**
 * @brief Virtual serial terminal.
 */
class USBSerial : public Stream {
public:
    USBSerial(void) {}

    void begin(void);

    // Roger Clark. Added dummy function so that existing Arduino sketches which specify baud rate will compile.
    void begin(unsigned long);
    void begin(unsigned long, uint8_t);
    void end(void);

    virtual int available(void) { return usb_cdcacm_read_available(); }
    int availableForWrite(void) { return usb_cdcacm_write_available(); }

    size_t readBytes(char *buf, const size_t len);
    uint32 read(uint8 * buf, uint32 len);

	// Roger Clark. added functions to support Arduino 1.0 API
	virtual int peek(void);
	// Blocks forever until 1 byte is received
	virtual int read(void) { return usb_cdcacm_read(); } // returns -1 if nothing received
	virtual void flush(void);


    size_t write(uint8);
    size_t write(const char *str);
    size_t write(const uint8*, uint32);

    uint8 getRTS(void) { return usb_cdcacm_get_rts(); }
    uint8 getDTR(void) { return usb_cdcacm_get_dtr(); }
    uint8 pending(void) { return usb_cdcacm_get_pending(); }

	void enableBlockingTx(void)	{ isBlocking=true; }
	void disableBlockingTx(void) { isBlocking=false; }


    /* SukkoPera: This is the Arduino way to check if an USB CDC serial
     * connection is open.

     * Used for instance in cardinfo.ino.
     */
    operator bool();

    /* Old libmaple way to check for serial connection.
     *
     * Deprecated, use the above.
     */
    uint8 isConnected() __attribute__((deprecated("Use !Serial instead"))) { return (bool) *this; }

protected:
	bool _hasBegun = false;
	bool isBlocking = false;
};

#ifdef SERIAL_USB
    extern USBSerial Serial;
#endif

#endif // BOARD_HAVE_SERIALUSB

#endif // _WIRISH_USB_SERIAL_H_
