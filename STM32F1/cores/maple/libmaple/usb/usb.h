/*
 * usb.h
 *
 *  Created on: Apr 25, 2020
 *      Author: stevestrong
 */


#ifndef _LIBMAPLE_USB_H_
#define _LIBMAPLE_USB_H_

#include <libmaple/util.h>
#include "usb_cdc.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef union status_t {
	// access as 16 bit in one read instruction
	uint16_t both;
	// -- or as separate 8 bit values --
	struct {
		uint8_t suspended;
		uint8_t configured;
	};
} status_t;
extern status_t usb_state;

/*
 * USB module core
 */

void USB_Init(void);
void USB_power_off(void);

static inline uint8 usb_is_connected() {
    return !usb_state.suspended;
}

static inline uint8 usb_is_configured() {
    return usb_state.configured;
}

// Returns true if the USB has been started and configured.
// Data transmission is only possible after this returns true.
// The main level application should wait for this condition before performing any communication.
static inline bool usb_is_ready(void) {
	status_t state = { usb_state.both }; // read status with one instruction
    return ( state.configured && !state.suspended && (dtr_rts&BIT(0)));
}


#ifdef __cplusplus
}
#endif

#endif
