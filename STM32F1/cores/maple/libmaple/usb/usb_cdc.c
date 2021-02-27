/*
 * usb_cdc.c
 *
 *  Created on: Apr 25, 2020
 *      Author: stevestrong
 */


#include <libmaple/ring_buffer.h>
#include "usb.h"
#include "usb_def.h"
#include "usb_func.h"


usb_cdcacm_line_coding lineCoding;
uint8 dtr_rts;
voidFuncPtr dataHook;
voidFuncPtr ifaceHook;

// ring buffers for USB In und Out data
RING_BUFFER(usbRxRB, USB_RX_BUF_SIZE);
RING_BUFFER(usbTxRB, USB_TX_BUF_SIZE);


void usb_cdcacm_init()
{
	lineCoding.baudRate = BAUD_RATE;
	lineCoding.stopBits = 0;
	lineCoding.parityType = 0;
	lineCoding.dataBits = 8;
	dtr_rts = 0;
	dataHook = NULL;
	ifaceHook = NULL;
}

void usb_cdcacm_enable()
{
	usb_cdcacm_init();
	USB_Init();
}
void usb_cdcacm_disable() { DisableUsbIRQ(); }

// USB serial related routines
// These functions are used on main level

// Returns the number of available bytes received by USB
uint16 usb_cdcacm_read_available(void)
{
    return rb_read_available(&usbRxRB);
}

// Reads a character from the USB receive buffer.
// Returns -1 if none received
int usb_cdcacm_read(void)
{
	int ret = rb_read_safe(&usbRxRB);

	USB_BeginDataRx();

	return ret;
}

void usb_cdcacm_rx_flush(void)
{
	while (rb_read_available(&usbTxRB))
		rb_read(&usbTxRB);
}
// Returns the number of available free slots in the transmit buffer
uint16 usb_cdcacm_write_available(void)
{
    return rb_write_available(&usbTxRB);
}

// Returns true if there is not data to send
bool usb_cdcacm_write_empty(void)
{
    return rb_is_empty(&usbTxRB);
}

// Transmits a character over the USB serial interface
bool usb_cdcacm_write(char c)
{
	if ( !rb_write_safe(&usbTxRB, c) )
		return false;

	USB_BeginDataTx();

    return true;
}

// Transmits a string over the USB serial interface
uint32 usb_cdcacm_tx(const uint8* buf, uint32 len)
{
	uint32 ret = rb_write_safe_n(&usbTxRB, buf, len);

	USB_BeginDataTx();

	return ret;
}

uint32 usb_cdcacm_rx(uint8* buf, uint32 len)
{
	uint32 ret = rb_read_n(&usbRxRB, buf, len);

	USB_BeginDataRx();

	return ret;
}

uint32 usb_cdcacm_peek(uint8* buf, uint32 len)
{
	return rb_peek_n(&usbRxRB, buf, len);
}

uint8 usb_cdcacm_get_dtr(void)
{
	return (dtr_rts & BIT(0));
}

uint8 usb_cdcacm_get_rts(void)
{
	return (dtr_rts & BIT(1));
}

void usb_cdcacm_set_hooks(int hook, voidFuncPtr hook_func)
{
	if (hook & USB_CDCACM_HOOK_RX)
		dataHook = hook_func;
	if (hook & USB_CDCACM_HOOK_IFACE_SETUP)
		ifaceHook = hook_func;
}
