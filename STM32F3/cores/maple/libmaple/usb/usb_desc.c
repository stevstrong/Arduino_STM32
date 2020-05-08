//  CDC code from https://github.com/Apress/Beg-STM32-Devel-FreeRTOS-libopencm3-GCC/blob/master/rtos/usbcdcdemo/usbcdc.c
/*
 * Copyright (c) 2016, Devan Lai
 *
 * Adapted for stm32duino.com by stevestrong
 *
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted, provided
 * that the above copyright notice and this permission notice
 * appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "usb_cdc_def.h"
#include "usb_desc.h"
#include "usb_std.h"


#define SERIAL_USB_ONLY // use CDC only. Uncomment for a combined device

#define COMM_PACKET_SIZE		8
#define MAX_USB_PACKET_SIZE		64

// USB interface numbers
#define IFACE_COMM		0
#define IFACE_DATA		1

//-----------------------------------------------------------------------------
// USB string descriptor management
//-----------------------------------------------------------------------------
// USB descriptor strings must sync with the string indexes from "usbstd.h" !!!
//-----------------------------------------------------------------------------

#define lang	0x0409 // english
#define manuf	"LeafLabs"
#define prod	"Maple"
#define serial	"2020.04.12"
#define func	"STM32F1 Serial Port"
#define comm	"Blue Pill COMM"
#define data	"Blue Pill DATA"

// Macros to create usb_string_descriptor from string
#define __CAT(x,y) x ## y
#define CAT(x,y) __CAT(x,y)
#define USB_LANG_DESC(s)		{.bLength = 4,\
								.bDescriptorType = USB_DT_STRING,\
								.wData = {s}}
#define USB_STRING_DESC(s)		{.bLength = sizeof(CAT(u,s)),\
								.bDescriptorType = USB_DT_STRING,\
								.wData = {CAT(u,s)}}

const usb_string_descriptor lang_desc	= USB_LANG_DESC(lang);
const usb_string_descriptor manuf_desc	= USB_STRING_DESC(manuf);
const usb_string_descriptor prod_desc	= USB_STRING_DESC(prod);
const usb_string_descriptor serial_desc	= USB_STRING_DESC(serial);
const usb_string_descriptor func_desc	= USB_STRING_DESC(func);
const usb_string_descriptor comm_desc	= USB_STRING_DESC(comm);
const usb_string_descriptor data_desc	= USB_STRING_DESC(data);

const usb_string_descriptor* const descriptors [USB_STRING_LAST] = {
	&lang_desc,
	&manuf_desc,
	&prod_desc,
	&serial_desc,
	&func_desc,
	&comm_desc,
	&data_desc,
};

//-----------------------------------------------------------------------------
//  USB Device
//-----------------------------------------------------------------------------
const usb_device_descriptor deviceDescriptor = {
    .bLength = sizeof(struct usb_device_descriptor),
    .bDescriptorType = USB_DT_DEVICE,
#ifdef USB21_INTERFACE
    .bcdUSB = 0x0210,  //  USB Version 2.1.  Need to handle special requests e.g. BOS.
#else
    .bcdUSB = 0x0200,  //  USB Version 2.0.  No need to handle special requests e.g. BOS.
#endif  //  USB21_INTERFACE

#ifdef SERIAL_USB_ONLY  //  If we are providing serial interface only...
	.bDeviceClass = USB_CLASS_CDC,  //  Set the class to CDC if it's only serial.  Serial interface will not start on Windows when class = 0.
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
#else  //  If we are providing multiple interfaces...
    .bDeviceClass = USB_CLASS_MISCELLANEOUS, //  Copied from microbit. For composite device, let host probe the interfaces.
    .bDeviceSubClass = 2,  //  Common Class
    .bDeviceProtocol = 1,  //  Interface Association Descriptor
#endif  //  SERIAL_USB_INTERFACE
    .bMaxPacketSize0 = MAX_USB_PACKET_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0220,  //  Device Release number 2.2
    .iManufacturer = USB_STRINGS_MANUFACTURER,
    .iProduct = USB_STRINGS_PRODUCT,
    .iSerialNumber = USB_STRINGS_SERIAL_NUMBER,
    .bNumConfigurations = 1,
};

//-----------------------------------------------------------------------------
typedef struct cdcacm_funct_descriptor {
	usb_cdc_header_descriptor header;
	usb_cdc_call_management_descriptor call_mgmt;
	usb_cdc_acm_descriptor acm;
	usb_cdc_union_descriptor cdc_union;
} __attribute__((packed))  cdcacm_funct_descriptor;

//-----------------------------------------------------------------------------
// Complete configuration descriptor for the composite device
// See: https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-interface-association-descriptor
//-----------------------------------------------------------------------------
typedef struct cfgDescriptor {
	usb_config_descriptor config;
	usb_interface_descriptor comm_iface;
		cdcacm_funct_descriptor cdcacm_functional_descriptors;
		usb_endpoint_descriptor comm_endp;
	usb_interface_descriptor data_iface;
		usb_endpoint_descriptor data_endp_in;
		usb_endpoint_descriptor data_endp_out;
} __attribute__((packed)) cfgDescriptor;

const cfgDescriptor configDescriptor =
{
	.config = {
		.bLength = sizeof(usb_config_descriptor),
		.bDescriptorType = USB_DT_CONFIGURATION,
		.wTotalLength = sizeof(cfgDescriptor),
		.bNumInterfaces = NUM_IFACES, // comm and data
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = 0x80,  //  Bus-powered, i.e. it draws power from USB bus.
		.bMaxPower = 0x32, // 100ma      0xfa,     //  500 mA. Copied from microbit.
	},
	.comm_iface = {
		.bLength = sizeof(usb_interface_descriptor), //USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = IFACE_COMM,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = USB_CLASS_CDC,
		.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
		.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
		.iInterface = USB_STRINGS_COMM,  //  Name of COMM
	},
		.cdcacm_functional_descriptors = {
			.header = {
				.bFunctionLength = sizeof(usb_cdc_header_descriptor),
				.bDescriptorType = CS_INTERFACE,
				.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
				.bcdCDC = 0x0110,
			},
			.call_mgmt = {
				.bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
				.bDescriptorType = CS_INTERFACE,
				.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
				.bmCapabilities = 0,
				.bDataInterface = IFACE_DATA,  //  DATA Interface
			},
			.acm = {
				.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
				.bDescriptorType = CS_INTERFACE,
				.bDescriptorSubtype = USB_CDC_TYPE_ACM,
				.bmCapabilities = 0, //0x02, // supports Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding, and Serial_State
			},
			.cdc_union = {
				.bFunctionLength = sizeof(usb_cdc_union_descriptor),
				.bDescriptorType = CS_INTERFACE,
				.bDescriptorSubtype = USB_CDC_TYPE_UNION,
				.bControlInterface = IFACE_COMM,       //  COMM Interface
				.bSubordinateInterface0 = IFACE_DATA,  //  DATA Interface
			 },
		},
		.comm_endp = {
			.bLength = sizeof(usb_endpoint_descriptor),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = EP_COMM_ADDR_IN,
			.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
			.wMaxPacketSize = COMM_PACKET_SIZE,  //  Smaller than others
			.bInterval = 255,
		},
	.data_iface = {
		.bLength = sizeof(usb_interface_descriptor),
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = IFACE_DATA,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_DATA,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = USB_STRINGS_DATA,  //  Name of DATA
	},
		.data_endp_in = {
			.bLength = sizeof(usb_endpoint_descriptor),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = EP_DATA_ADDR_IN,
			.bmAttributes = USB_ENDPOINT_ATTR_BULK,
			.wMaxPacketSize = MAX_USB_PACKET_SIZE,
			.bInterval = 0,
		},
		.data_endp_out = {
			.bLength = sizeof(usb_endpoint_descriptor),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = EP_DATA_ADDR_OUT,
			.bmAttributes = USB_ENDPOINT_ATTR_BULK,
			.wMaxPacketSize = MAX_USB_PACKET_SIZE,
			.bInterval = 0,
		},
};
