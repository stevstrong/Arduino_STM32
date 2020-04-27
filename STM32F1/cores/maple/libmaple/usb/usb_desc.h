/*
 * usb_desc.h
 *
 *  Created on: Apr 25, 2020
 *      Author: stevestrong
 */

#ifndef USB_DESC_H_
#define USB_DESC_H_


#if 1
#define USB_VID		0x1EAF // 0x0416   /* Vendor ID (von RealTek) */
#define USB_PID		0x0004 // 0x5011   /* Product ID */
#else
#define USB_VID		0xDEAD
#define USB_PID		0xBEEF
#endif

// assignment of the USB EP numbers - bEndpointAddress
enum { EP_CTRL, EP_COMM, EP_DATA, NUM_EP };

#define NUM_IFACES	2 // COMM + DATA

#define EP_CTRL_ADDR_IN		USB_EP_ADDR_IN(EP_CTRL)
#define EP_CTRL_ADDR_OUT	USB_EP_ADDR_OUT(EP_CTRL)
#define EP_COMM_ADDR_IN		USB_EP_ADDR_IN(EP_COMM)
#define EP_COMM_ADDR_OUT	USB_EP_ADDR_OUT(EP_COMM)
#define EP_DATA_ADDR_IN		USB_EP_ADDR_IN(EP_DATA)
#define EP_DATA_ADDR_OUT	USB_EP_ADDR_OUT(EP_DATA)


#endif /* USB_DESC_H_ */
