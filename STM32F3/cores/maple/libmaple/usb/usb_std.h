/** @defgroup usb_type_defines USB Standard Structure Definitions

@brief <b>Defined Constants and Types for the USB Standard Structure
Definitions</b>

@ingroup USB_defines

@version 1.0.0

@author @htmlonly &copy; @endhtmlonly 2010
Gareth McMullin <gareth@blacksphere.co.nz>

@date 10 March 2013

A set of structure definitions for the USB control structures
defined in chapter 9 of the "Universal Serial Bus Specification Revision 2.0"
Available from the USB Implementers Forum - http://www.usb.org/

LGPL License Terms @ref lgpl_license
*/



#ifndef __USBSTD_H
#define __USBSTD_H

#include <stdint.h>


//-----------------------------------------------------------------------------
// Index of USB strings. Must sync with the string definitions !!!
//-----------------------------------------------------------------------------
enum usb_strings_index {
    USB_STRINGS_LANGUAGE = 0,
    USB_STRINGS_MANUFACTURER,
    USB_STRINGS_PRODUCT,
    USB_STRINGS_SERIAL_NUMBER,
    USB_STRINGS_SERIAL_PORT,
    USB_STRINGS_COMM,
    USB_STRINGS_DATA,
    USB_STRINGS_HID,
	USB_STRING_LAST // dummy
};

/*
#define 	USB_CONFIG_ATTR_DEFAULT   0x80 // always required (USB2.0 table 9-10)
#define 	USB_CONFIG_ATTR_SELF_POWERED   0x40
#define 	USB_CONFIG_ATTR_REMOTE_WAKEUP   0x20
#define 	USB_DT_IAD_SIZE   sizeof(struct usb_iface_assoc_descriptor)
*/
#define 	USB_EP_ADDR_OUT(x)   (x)
#define 	USB_EP_ADDR_IN(x)   (0x80 | (x))


/*
 * This file contains structure definitions for the USB control structures
 * defined in chapter 9 of the "Universal Serial Bus Specification Revision 2.0"
 * Available from the USB Implementers Forum - http://www.usb.org/
 */

/* USB Setup Data structure - Table 9-2 */
struct usb_setup_data {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} __attribute__((packed));

/* Class Definition */
#define USB_CLASS_HID				0x03
#define USB_CLASS_MISCELLANEOUS		0xEF
#define USB_CLASS_VENDOR			0xFF

/* bmRequestType bit definitions */
/* bit 7 : direction */
#define USB_REQ_TYPE_DIRECTION		0x80
#define USB_REQ_TYPE_IN				0x80
/* bits 6..5 : type */
#define USB_REQ_TYPE_TYPE			0x60
#define USB_REQ_TYPE_STANDARD		0x00
#define USB_REQ_TYPE_CLASS			0x20
#define USB_REQ_TYPE_VENDOR			0x40
/* bits 4..0 : recipient */
#define USB_REQ_TYPE_RECIPIENT		0x1F
#define USB_REQ_TYPE_DEVICE			0x00
#define USB_REQ_TYPE_INTERFACE		0x01
#define USB_REQ_TYPE_ENDPOINT		0x02
#define USB_REQ_TYPE_OTHER			0x03

/* USB Standard Request Codes - Table 9-4 */
#define USB_REQ_GET_STATUS			0
#define USB_REQ_CLEAR_FEATURE		1
/* Reserved for future use: 2 */
#define USB_REQ_SET_FEATURE			3
/* Reserved for future use: 4 */
#define USB_REQ_SET_ADDRESS			5
#define USB_REQ_GET_DESCRIPTOR		6
#define USB_REQ_SET_DESCRIPTOR		7
#define USB_REQ_GET_CONFIGURATION	8
#define USB_REQ_SET_CONFIGURATION	9
#define USB_REQ_GET_INTERFACE		10
#define USB_REQ_SET_INTERFACE		11
#define USB_REQ_SET_SYNCH_FRAME		12

/* USB Descriptor Types - Table 9-5 */
#define USB_DT_DEVICE						1
#define USB_DT_CONFIGURATION				2
#define USB_DT_STRING						3
#define USB_DT_INTERFACE					4
#define USB_DT_ENDPOINT						5
#define USB_DT_DEVICE_QUALIFIER				6
#define USB_DT_OTHER_SPEED_CONFIGURATION	7
#define USB_DT_INTERFACE_POWER				8
/* From ECNs */
#define USB_DT_OTG							9
#define USB_DT_DEBUG						10
#define USB_DT_INTERFACE_ASSOCIATION		11
/* USB 3.1 Descriptor Types - Table 9-6 */
#define USB_DT_BOS							15
#define USB_DT_DEVICE_CAPABILITY			16
#define USB_DT_HID							0x21
#define USB_DT_HID_REPORT					0x22
#define USB_DT_SUPERSPEED_USB_ENDPOINT_COMPANION 48
#define USB_DT_SUPERSPEEDPLUS_ISOCHRONOUS_ENDPOINT_COMPANION 49

/* USB Standard Feature Selectors - Table 9-6 */
#define USB_FEAT_ENDPOINT_HALT				0
#define USB_FEAT_DEVICE_REMOTE_WAKEUP		1
#define USB_FEAT_TEST_MODE					2

/* Information Returned by a GetStatus() Request to a Device - Figure 9-4 */
#define USB_DEV_STATUS_SELF_POWERED			0x01
#define USB_DEV_STATUS_REMOTE_WAKEUP		0x02

/* USB Standard Device Descriptor - Table 9-8 */
typedef struct usb_device_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
} __attribute((packed)) usb_device_descriptor;
//#define USB_DT_DEVICE_SIZE sizeof(struct usb_device_descriptor)


/* USB Standard Configuration Descriptor - Table 9-10 */
typedef struct usb_config_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
//	uint8_t iInterface;
} __attribute((packed)) usb_config_descriptor;
//#define USB_DT_CONFIGURATION_SIZE		9

/* USB Configuration Descriptor bmAttributes bit definitions */
#define USB_CONFIG_ATTR_DEFAULT			0x80	/** always required (USB2.0 table 9-10) */
#define USB_CONFIG_ATTR_SELF_POWERED	0x40
#define USB_CONFIG_ATTR_REMOTE_WAKEUP	0x20


typedef struct usb_iad_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bFirstInterface;
	uint8_t bInterfaceCount;
	uint8_t bFunctionClass;
	uint8_t bFunctionSubClass;
	uint8_t bFunctionProtocol;
	uint8_t iFunction;
} __attribute((packed)) usb_iad_descriptor;


/* Other Speed Configuration is the same as Configuration Descriptor.
 *  - Table 9-11
 */

/* USB Standard Interface Descriptor - Table 9-12 */
typedef struct usb_interface_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
} __attribute((packed)) usb_interface_descriptor;

/* USB Standard Endpoint Descriptor - Table 9-13 */
typedef struct usb_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
} __attribute((packed)) usb_endpoint_descriptor;

typedef struct usb_device_capability_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDevCapabilityType;
} __attribute((packed)) usb_device_capability_descriptor;

typedef struct usb_bos_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumDeviceCaps;
} __attribute((packed)) usb_bos_descriptor;


/* USB Device Capability Types - USB 3.1 Table 9-14 */
#define USB_DC_WIRELESS_USB 1
#define USB_DC_USB_2_0_EXTENSION 2
#define USB_DC_SUPERSPEED_USB 3
#define USB_DC_CONTAINER_ID 4
#define USB_DC_PLATFORM 5
#define USB_DC_POWER_DELIVERY_CAPABILITY 6
#define USB_DC_BATTERY_INFO_CAPABILITY 7
#define USB_DC_PD_CONSUMER_PORT_CAPABILITY 8
#define USB_DC_PD_PROVIDER_PORT_CAPABILITY 9
#define USB_DC_SUPERSPEED_PLUS 10
#define USB_DC_PRECISION_TIME_MEASUREMENT 11
#define USB_DC_WIRELESS_USB_EXT 12

/* USB bEndpointAddress helper macros */
#define USB_EP_ADDR_OUT(x) (x)
#define USB_EP_ADDR_IN(x) (0x80 | (x))

/* USB Endpoint Descriptor bmAttributes bit definitions - Table 9-13 */
/* bits 1..0 : transfer type */
#define USB_ENDPOINT_ATTR_CONTROL		0x00
#define USB_ENDPOINT_ATTR_ISOCHRONOUS	0x01
#define USB_ENDPOINT_ATTR_BULK			0x02
#define USB_ENDPOINT_ATTR_INTERRUPT		0x03
#define USB_ENDPOINT_ATTR_TYPE			0x03
/* bits 3..2 : Sync type (only if ISOCHRONOUS) */
#define USB_ENDPOINT_ATTR_NOSYNC		0x00
#define USB_ENDPOINT_ATTR_ASYNC			0x04
#define USB_ENDPOINT_ATTR_ADAPTIVE		0x08
#define USB_ENDPOINT_ATTR_SYNC			0x0C
#define USB_ENDPOINT_ATTR_SYNCTYPE		0x0C
/* bits 5..4 : usage type (only if ISOCHRONOUS) */
#define USB_ENDPOINT_ATTR_DATA			0x00
#define USB_ENDPOINT_ATTR_FEEDBACK		0x10
#define USB_ENDPOINT_ATTR_IMPLICIT_FEEDBACK_DATA 0x20
#define USB_ENDPOINT_ATTR_USAGETYPE		0x30

/* Table 9-15 specifies String Descriptor Zero.
 * Table 9-16 specified UNICODE String Descriptor.
 */
typedef struct usb_string_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wData[];
} __attribute((packed)) usb_string_descriptor;


enum usb_language_id {
	USB_LANGID_ENGLISH_US = 0x409,
};
#endif

