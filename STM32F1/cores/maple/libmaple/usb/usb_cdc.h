/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
 * @file libmaple/usb_cdcacm.h
 * @brief USB CDC ACM (virtual serial terminal) support
 *
 * IMPORTANT: this API is unstable, and may change without notice.
 */

#ifndef _LIBMAPLE_USB_CDCACM_H_
#define _LIBMAPLE_USB_CDCACM_H_

#include "libmaple/libmaple_types.h"
#include "libmaple/ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif


#define USB_RX_BUF_SIZE		256 // must be power of 2 !!!
#define USB_TX_BUF_SIZE		256 // must be power of 2 !!!
/*
 * CDC ACM interface
 */

void usb_cdcacm_init();
void usb_cdcacm_enable();
void usb_cdcacm_disable();

bool usb_cdcacm_write(char ch);
int usb_cdcacm_read(void);
void usb_cdcacm_rx_flush(void);
uint32 usb_cdcacm_tx(const uint8* buf, uint32 len);
uint32 usb_cdcacm_rx(uint8* buf, uint32 len);
uint32 usb_cdcacm_peek(uint8* buf, uint32 len);
uint32 usb_cdcacm_peek_ex(uint8* buf, uint32 offset, uint32 len);

uint16 usb_cdcacm_read_available(void); /* in RX buffer */
uint16 usb_cdcacm_write_available();
uint16 usb_cdcacm_get_pending(void);
uint8 usb_cdcacm_is_transmitting(void);

uint8 usb_cdcacm_get_dtr(void);
uint8 usb_cdcacm_get_rts(void);

/* Line coding structure
 0-3 baudRate     Data terminal rate (baudrate), in bits per second
 4   bCharFormat  Stop bits: 0 - 1 Stop bit, 1 - 1.5 Stop bits, 2 - 2 Stop bits
 5   bParityType  Parity:    0 - None, 1 - Odd, 2 - Even, 3 - Mark, 4 - Space
 6   bDataBits    Data bits: 5, 6, 7, 8, 16
 */
typedef struct usb_cdcacm_line_coding {
    uint32 baudRate;           /* Baud rate */

#define USB_CDCACM_STOP_BITS_1   0
#define USB_CDCACM_STOP_BITS_1_5 1
#define USB_CDCACM_STOP_BITS_2   2
    uint8 stopBits;          /* Stop bits */

#define USB_CDCACM_PARITY_NONE  0
#define USB_CDCACM_PARITY_ODD   1
#define USB_CDCACM_PARITY_EVEN  2
#define USB_CDCACM_PARITY_MARK  3
#define USB_CDCACM_PARITY_SPACE 4
    uint8 parityType;          /* Parity type */

    uint8 dataBits;            /* Data bits: 5, 6, 7, 8, or 16 */
} __packed usb_cdcacm_line_coding;
extern usb_cdcacm_line_coding lineCoding;

extern uint8_t dtr_rts;

/* Retrieve a copy of the current line coding structure. */
void usb_cdcacm_get_line_coding(usb_cdcacm_line_coding*);

/* Line coding conveniences. */
int usb_cdcacm_get_baud(void);        /* dwDTERate */
int usb_cdcacm_get_stop_bits(void);   /* bCharFormat */
int usb_cdcacm_get_parity(void);      /* bParityType */
int usb_cdcacm_get_n_data_bits(void); /* bDataBits */

extern ring_buffer_t usbRxRB;
extern ring_buffer_t usbTxRB;

/*
 * Hack: hooks for bootloader reset signalling
 */

#define USB_CDCACM_HOOK_RX 0x1
#define USB_CDCACM_HOOK_IFACE_SETUP 0x2

extern voidFuncPtr dataHook;
extern voidFuncPtr ifaceHook;

void usb_cdcacm_set_hooks(int hook_flags, voidFuncPtr hook);

inline void usb_cdcacm_remove_hooks(int hook_flags) {
    usb_cdcacm_set_hooks(hook_flags, 0);
}

#ifdef __cplusplus
}
#endif

#endif
