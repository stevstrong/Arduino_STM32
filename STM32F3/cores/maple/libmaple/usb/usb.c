/*
 * usb.h
 *
 *  Created on: Apr 25, 2020
 *      Author: stevestrong
 */
//-----------------------------------------------------------------------------
// Use USB as virtual COM Port
//-----------------------------------------------------------------------------

#include <stddef.h>
#include <libmaple/rcc.h>
#include <libmaple/nvic.h>
#include "usb_std.h"
#include "usb_func.h"
#include "usb_desc.h"
#include "usb_cdc.h"


//-----------------------------------------------------------------------------
//  Variables
//-----------------------------------------------------------------------------
command_t CMD;


typedef union status_t {
	// access as 16 bit in one read instruction
	uint16_t both;
	// -- or as separate 8 bit values --
	struct {
		uint8_t suspended;
		uint8_t configured;
	};
} status_t;
status_t usb_state;

uint8_t deviceAddress;

const epTableAddress_t epTableAddr[NUM_EP] = { // number of EPs
	{ .txAddr = (uint32*)EP_CTRL_TX_BUF_ADDRESS, .rxAddr = (uint32*)EP_CTRL_RX_BUF_ADDRESS },
	{ .txAddr = (uint32*)EP_COMM_TX_BUF_ADDRESS, .rxAddr = (uint32*)EP_COMM_RX_BUF_ADDRESS },
	{ .txAddr = (uint32*)EP_DATA_TX_BUF_ADDRESS, .rxAddr = (uint32*)EP_DATA_RX_BUF_ADDRESS },
};
volatile bool receiving;
volatile bool transmitting;

// constant to send zero byte packets
const uint8_t ZERO = 0;

//-----------------------------------------------------------------------------
void USB_power_off(void)
{
//	noInterrupts();
	nvic_irq_disable(NVIC_USB_LP_CAN_RX0);
	USB_CNTR = USB_CNTR_FRES;
	USB_ISTR = 0;
	USB_CNTR = USB_CNTR_FRES + USB_CNTR_PDWN;
 }
//-----------------------------------------------------------------------------
// Function to initialize the VCP
//-----------------------------------------------------------------------------
void USB_Start(void)
{
	CMD.configuration = 0;
	deviceAddress = 0;
	receiving = true;
	transmitting = false;
	usb_state.suspended = true;
	usb_state.configured = false;
}
//-----------------------------------------------------------------------------
void USB_Init(void)
{
	// Enable USB
	rcc_clk_enable(RCC_USB);

	USB_power_off();

	uint32_t* P = (uint32_t*) USB_RAM; // clear USB RAM memory
	while ((uint32_t) P < (USB_RAM + 1024))
	{
		*P++ = 0;
	}

	USB_Start();

	// turn on USB, activate interrupts
	USB_CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;

	EnableUsbIRQ();
}
//-----------------------------------------------------------------------------
// helper routines
//--------------------------------------------------------------------------
void EnableUsbIRQ (void)
{
    NVIC_ISER[USB_IRQ_NUMBER/32] = ((uint32_t) 1) << (USB_IRQ_NUMBER % 32);
}

void DisableUsbIRQ (void)
{
    NVIC_ICER[USB_IRQ_NUMBER/32] = ((uint32_t) 1) << (USB_IRQ_NUMBER % 32);
}
//-----------------------------------------------------------------------------
static inline void USB_SetAddress(uint8_t adr)
{
	USB_DADDR = 0x80 | adr;
}

//--------------------------------------------------------------------------
static inline bool IsStandardRequest(void)
{
    return (CMD.setupPacket.bmRequestType & 0x60) == 0;
}

//--------------------------------------------------------------------------
static inline bool IsClassRequest(void)
{
    return (CMD.setupPacket.bmRequestType & 0x60) == 0x20;
}

//--------------------------------------------------------------------------
static inline void OnEpIntIn(void) /* Int-EP IN */
{
	// nothing to do
}

//--------------------------------------------------------------------------
static inline void OnEpIntOut(void) /* Int-EP IN */
{
	// nothing to do
}

//-----------------------------------------------------------------------------
static inline bool USB_ConfigDevice(bool obConf)
{
	(void)obConf;
	return true;  // nothing to do
}
//-----------------------------------------------------------------------------
void Stall_EPAddr(int epNum)
{
	uint32_t shift, mask;

	if ( epNum & 0x80 ) // IN EP
	{
		epNum &= 0x7F; // remove bit 7
		mask = EP_MASK_NoToggleBits | STAT_RX; // without STAT_TX and without both DTOG_x
		shift = 1 << 12;
	}
	else
	{
		mask = EP_MASK_NoToggleBits | STAT_TX; // without STAT_RX and without both DTOG_x
		shift = 1 << 4;
	}
	uint32_t data = USB_EpRegs(epNum);
	USB_EpRegs(epNum) = (data ^ shift) & mask;
}
//-----------------------------------------------------------------------------
void Stall(int ep)
{
	Stall_EPAddr(USB_EP_ADDR_OUT(ep));
	Stall_EPAddr(USB_EP_ADDR_IN(ep));
}
//-----------------------------------------------------------------------------
void UnStall_EPAddr(int epNum)
{
	uint32_t shift, mask;

	if ( epNum & 0x80 )
	{
		epNum &= 0x7F; // remove bit 7
		mask = EP_MASK_NoToggleBits | STAT_RX; // without STAT_TX and without both DTOG_x
		shift = 3 << 12; // RX = VALID
	}
	else
	{
		mask = EP_MASK_NoToggleBits | STAT_TX; // without STAT_RX and without both DTOG_x
		shift = 2 << 4; // TX = NAK
	}
	uint32_t data = USB_EpRegs(epNum);
	USB_EpRegs(epNum) = (data ^ shift) & mask;
}
//-----------------------------------------------------------------------------
void UnStall(int ep)
{
	UnStall_EPAddr(USB_EP_ADDR_OUT(ep));
	UnStall_EPAddr(USB_EP_ADDR_IN(ep));
}

//-----------------------------------------------------------------------------
// mark EP ready to receive, set STAT_RX to "11" by toggling
//-----------------------------------------------------------------------------
void MarkBufferRxDone(int ep)
{
	receiving = true;
	uint32_t mask = EP_MASK_NoToggleBits | STAT_RX; // without STAT_TX and without both DTOG_x
	uint32_t data = USB_EpRegs(ep);
	USB_EpRegs(ep) = (data ^ STAT_RX) & mask;
}

//-----------------------------------------------------------------------------
// mark EP ready to transmit, set STAT_TX to "11" by toggling
//-----------------------------------------------------------------------------
void MarkBufferTxReady(int ep)
{
	uint32_t mask = EP_MASK_NoToggleBits | STAT_TX; // without STAT_RX and without both DTOG_x
	uint32_t data = USB_EpRegs(ep);
	USB_EpRegs(ep) = (data ^ STAT_TX) & mask;
}

//-----------------------------------------------------------------------------
// initialize the EPs
//-----------------------------------------------------------------------------
void InitEndpoints(void)
{
	USB_CNTR = 1;          // Reset and clear Ints
	CMD.configuration = 0;
	CMD.transferLen = 0;   // no transfers
	CMD.packetLen = 0;
	CMD.transferPtr = 0;
	USB_CNTR = 0;          // release reset
	usb_state.suspended = false;
	usb_state.configured = false;
	transmitting = false;
	receiving = true;

	// EP0 ist always reserved for control
	// the other endpoints must match the numbers written in the descriptors
	// see usb_desc.c: EP_DATA, EP_COMM

	// EP0 = Control, IN und OUT
	EpTable[EP_CTRL].txOffset = EP_CTRL_TX_OFFSET;
	EpTable[EP_CTRL].txCount = 0;
	EpTable[EP_CTRL].rxOffset = EP_CTRL_RX_OFFSET;
	EpTable[EP_CTRL].rxCount = EP_RX_LEN_ID;

	// EP1 = Int IN and OUT, needed but not used
	EpTable[EP_COMM].txOffset = EP_COMM_TX_OFFSET;
	EpTable[EP_COMM].txCount = 0;
	EpTable[EP_COMM].rxOffset = EP_COMM_RX_OFFSET;
	EpTable[EP_COMM].rxCount = EP_RX_LEN_ID;

	// EP2 = Bulk IN and OUT
	EpTable[EP_DATA].txOffset = EP_DATA_TX_OFFSET;
	EpTable[EP_DATA].txCount = 0;
	EpTable[EP_DATA].rxOffset = EP_DATA_RX_OFFSET;
	EpTable[EP_DATA].rxCount = EP_RX_LEN_ID;

	USB_BTABLE = EP_TABLE_OFFSET;

	// CTRL EP
	USB_EP0R =			// EP0 = Control, IN and OUT
		(3 << 12) |		// STAT_RX = 3, Rx enabled
		(2 << 4) |		// STAT_TX = 2, NAK
		(1 << 9) |		// EP_TYPE = 1, Control
		EP_CTRL;
	// COMM EP
	USB_EP1R =			// EP1 = Int, IN und OUT
		(3 << 12) |		// STAT_RX = 3, Rx enabled
		(2 << 4) |		// STAT_TX = 2, NAK
		(3 << 9) |		// EP_TYPE = 3, INT
		EP_COMM;
	// DATA EP
	USB_EP2R =			// EP2 = Bulk IN and OUT
		(3 << 12) |		// STAT_RX = 3, Rx enabled
		(2 << 4) |		// STAT_TX = 2, NAK
		(0 << 9) |		// EP_TYPE = 0, Bulk
		EP_DATA;

	USB_ISTR = 0;          // clear pending Interrupts
	USB_CNTR =
		CTRM |             // Irq by ACKed Pakets
		RESETM |           // Irq by Reset
		SUSPM | WKUPM | ESOFM |
		SOFM;              // Irq by 1 ms Frame

	USB_SetAddress(0);
}

//-----------------------------------------------------------------------------
// reads up to a given number of bytes from EP0 receive buffer
//-----------------------------------------------------------------------------
int ReadCTRLBlock(uint8_t* pBuffer, int maxlen)
{
    int count = EpTable[EP_CTRL].rxCount & 0x3FF;

	if (count)
	{
		if (count > maxlen)
			count = maxlen;

		uint32* ptr = (uint32*) EP_CTRL_RX_BUF_ADDRESS;
		int i = count/2;
		while (i--)
		{
			register uint32 val = *ptr++;
			*pBuffer++ = val;
			*pBuffer++ = val>>8;
		}
		if (count&1) { // read last odd byte if any
			register uint32 val = *ptr;
			*pBuffer = (uint8)val;
		}
	}
    MarkBufferRxDone(EP_CTRL); // release EP0 Rx
    return count;
}
//-----------------------------------------------------------------------------
// writes up to 64 bytes into the EP0 transmit buffer
//-----------------------------------------------------------------------------
int WriteCTRLBlock(uint8_t * src, int count)
{
	if (count > EP_DATA_LEN)
		count = EP_DATA_LEN;

	if (count)
	{
		register uint32* dest = (uint32*) EP_CTRL_TX_BUF_ADDRESS;
		register int j = count/2;
		while (j--)
		{
		    register uint32 val = *src++;
		    val |= (*src++) << 8;
		    *dest++ = val;
		}
		if (count&1) // write last odd byte if any
		{
		    register uint32 val = *src;
		    *dest = val;
		}
	}
	EpTable[EP_CTRL].txCount = count;
	MarkBufferTxReady(EP_CTRL); // mark Tx buffer ready to be sent
	return count;
}

//-----------------------------------------------------------------------------
static inline void ACK(void)
{
	WriteCTRLBlock((uint8_t*) &ZERO, 0);
}

//-----------------------------------------------------------------------------
// Control-Transfers block transmission start
//-----------------------------------------------------------------------------
void TransmitSetupPacket(void)
{
	if ((CMD.setupPacket.bmRequestType & 0x80) == 0)
	{
		return;
	}
	int i = CMD.transferLen;
	if (i > CMD.packetLen)
		i = CMD.packetLen;
	uint8_t* Q = CMD.transferPtr; // source
	int j = WriteCTRLBlock(Q, i);
	CMD.transferPtr = Q + j; // update pointer for any remaining data
	CMD.transferLen = CMD.transferLen - j; // number of remaining data
	if (CMD.transferLen <= 0)
		CMD.transferLen = 0;
}

/**********************************************************************/
/************    Handling of incoming Requests   **********************/
/**********************************************************************/
//-----------------------------------------------------------------------------
// USB-Request "SET FEATURE" and "CLEAR FEATURE"
//-----------------------------------------------------------------------------
void DoSetClearFeature(bool value)
{
    int feature = CMD.setupPacket.wValue;
    int reqType = CMD.setupPacket.bmRequestType;
    int ep = CMD.setupPacket.wIndex;

    switch (reqType)
    {
    case USB_REQ_TYPE_DEVICE: // for Device
        if (feature == 1)
            CMD.remoteWakeup = value;
        break;

    case USB_REQ_TYPE_INTERFACE: // for Interface
         break;

    case USB_REQ_TYPE_ENDPOINT: // for an Endpoint
        if (feature == 0)
        {
            if (value == false)
                Stall(ep);
            else
                UnStall(ep);
        }
        break;

    default: // should not happen
        Stall_EPAddr(EP_CTRL_ADDR_OUT);
        break;
    }
}

//-----------------------------------------------------------------------------
// CDC specific functions
//-----------------------------------------------------------------------------
static inline bool Class_Compare(uint16_t aValue) // always true, not used
{
	(void) aValue;
    return true;
}

//-----------------------------------------------------------------------------
// store DTR and RTS values
//-----------------------------------------------------------------------------
static void CDC_Set_DTR_RTS(void)
{
    dtr_rts = (uint8)CMD.setupPacket.wValue;
    if (ifaceHook!=NULL) ifaceHook();
}
//-----------------------------------------------------------------------------
// read line coding parameters from the EP buffer
//-----------------------------------------------------------------------------
static void CDC_SetLineCoding(void)
{
	ReadCTRLBlock((uint8_t*) &lineCoding, sizeof(usb_cdcacm_line_coding));
	// set the USART parameters according to the new data
	// usart_config_line_coding(USART1, &lineCoding);
}
//-----------------------------------------------------------------------------
// send line coding parameters to the host
//-----------------------------------------------------------------------------
static void CDC_GetLineCoding(void)
{
	CMD.packetLen = EP_DATA_LEN;
	CMD.transferLen = 7;
	CMD.transferPtr = (uint8_t*) &lineCoding;
	TransmitSetupPacket();
}

//-----------------------------------------------------------------------------
// Setup request functions
//-----------------------------------------------------------------------------
void Req_GetStatus()
{
	uint8_t buf[2];
	buf[0] = 0;
	buf[1] = 0;

	int reqType = CMD.setupPacket.bmRequestType;

	switch (reqType&0x7F)
	{
	case USB_REQ_TYPE_DEVICE: // for Device
		if (CMD.remoteWakeup)
			buf[0] |= 2;
		if (CMD.selfPowered)
			buf[0] |= 1;
		break;

	case USB_REQ_TYPE_INTERFACE: // for Interface
		break;

	case USB_REQ_TYPE_ENDPOINT: // for an Endpoint
	{
		int ep = CMD.setupPacket.wIndex;
		if ( (ep >= EP_CTRL) && (ep <= EP_DATA) )
			buf[0] = 1;
		break;
	}

	default:
		Stall_EPAddr(EP_CTRL_ADDR_OUT); // send STAL
		return;
	}

	CMD.packetLen = EP_DATA_LEN;
	CMD.transferLen = 2;
	CMD.transferPtr = buf;
	TransmitSetupPacket();
}
//-----------------------------------------------------------------------------
void Req_ClearFeature()
{
    DoSetClearFeature(false);
}
//-----------------------------------------------------------------------------
void Req_SetFeature()
{
    DoSetClearFeature(true);
}
//-----------------------------------------------------------------------------
void Req_SetAddress()
{
	deviceAddress = CMD.setupPacket.wValue;
	ACK();
}
//-----------------------------------------------------------------------------
// Send descriptors to the host
//-----------------------------------------------------------------------------
void Req_GetDescriptor()
{
	const uint8_t* ptr;
	int aLen = -1;
	uint16_t ind = CMD.setupPacket.wValue & 0xFF;
	uint16_t type = CMD.setupPacket.wValue >> 8;

	switch (type)
	{
	case USB_DT_DEVICE: // Get Device Descriptor
		{
			// descriptor index
			extern const uint8_t deviceDescriptor[];
			ptr = deviceDescriptor;
			aLen = ptr[0]; // bLength
			break;
		}
	case USB_DT_CONFIGURATION: // Get Configuration Descriptor
		{
			extern const uint8_t configDescriptor[];
			ptr = configDescriptor;
			aLen = ptr[2];			// wTotalLength low byte
			aLen |= (ptr[3] << 8);	// wTotalLength high byte
			break;
		}
	case USB_DT_STRING: // Get String Descriptor
		{
			extern const usb_string_descriptor* const descriptors [];

			if (ind<USB_STRING_LAST) {
				ptr = (const uint8_t*)descriptors[ind];
				aLen = ptr[0];
			} else {
				goto doStall; // out of range index
			}
			break;
		}
	case USB_DT_DEVICE_QUALIFIER: // fall through
	default:
		{
doStall:
			Stall_EPAddr(EP_CTRL_ADDR_OUT); // unknown. send STAL
			break;
		}
	}

	if (aLen < 0)
		return;

	// limit the number to that requested by the host
	if (aLen > CMD.setupPacket.wLength)
		aLen = CMD.setupPacket.wLength;
	CMD.packetLen = EP_DATA_LEN;
	CMD.transferLen = aLen;
	CMD.transferPtr = (uint8_t*) ptr;
	TransmitSetupPacket();
}
//-----------------------------------------------------------------------------
void Req_GetConfiguration()
{
	CMD.packetLen = EP_DATA_LEN;
	CMD.transferLen = 1;
	CMD.transferPtr = (uint8_t*) &CMD.configuration;
	TransmitSetupPacket();
}
//-----------------------------------------------------------------------------
void Req_SetConfiguration()
{
	bool config = Class_Compare(CMD.setupPacket.wValue);
	if (CMD.setupPacket.wValue == 0)
	{
		CMD.configuration = CMD.setupPacket.wValue & 0xFF;
		usb_state.configured = false;
	}
	else if (config)
	{
		USB_ConfigDevice(true);
		USB_Start();
		CMD.configuration = CMD.setupPacket.wValue & 0xFF;
		usb_state.configured = true;
	}
	else
	{	// should not happen
		CMD.configuration = 0;
		usb_state.configured = false;
		Stall(EP_CTRL_ADDR_OUT); // send STAL
		return;
	}
	ACK();
}
//-----------------------------------------------------------------------------
void Req_GetInterface()
{
	CMD.transferLen = 1;
	CMD.transferPtr = (uint8_t*) &ZERO;
	TransmitSetupPacket();
}
//-----------------------------------------------------------------------------
void Req_SetInterface()
{
	USB_Start();
	ACK();
}
//-----------------------------------------------------------------------------
int sofCounter;
//-----------------------------------------------------------------------------
void Req_SOF()
{
	sofCounter++;
	ACK();
}
//-----------------------------------------------------------------------------
//------------------------ Setup-Event ----------------------------------------
//-----------------------------------------------------------------------------
/*
 Note:
 1. take packet and clear buffer.
 2. Setup packets
 - if nothing follows, e.g. if there is not data phase, ACK is sent.
 - if something has to be sent to the host, this will be done right away.
   If the data length is greater than the EP buffer, only the first part is sent.
   The rest of the packets are be sent on the following Control-In stage(s).
   After the last data packet an ACK (0 length packet) is sent.
 - if the host is transmitting something, this is signalized in the setup packet.
   The data itself is retrieved in the IN and CTRL OUT packets.
   The read process is terminated by sending an ACK packet.
*/
//-----------------------------------------------------------------------------
voidFuncPtr const stdRequests[] = {
		Req_GetStatus,			// USB_REQ_GET_STATUS = 0
		Req_ClearFeature,		// USB_REQ_CLEAR_FEATURE = 1
		NULL,					// RESERVED = 2
		Req_SetFeature,			// USB_REQ_SET_FEATURE = 3
		NULL,					// 4
		Req_SetAddress,			// USB_REQ_SET_ADDRESS = 5
		Req_GetDescriptor,		// USB_REQ_GET_DESCRIPTOR = 6
		NULL, //Req_SetDescriptor,		// USB_REQ_SET_DESCRIPTOR = 7
		Req_GetConfiguration,	// USB_REQ_GET_CONFIGURATION = 8
		Req_SetConfiguration,	// USB_REQ_SET_CONFIGURATION = 9
		Req_GetInterface,		// USB_REQ_GET_INTERFACE = 10
		Req_SetInterface,		// USB_REQ_SET_INTERFACE = 11
		Req_SOF,				// USB_REQ_SET_SYNCH_FRAME = 12
};
//-----------------------------------------------------------------------------
void OnSetup(void)
{
	// read the setup packet from CTRL EP buffer
	ReadCTRLBlock((uint8_t*)&CMD.setupPacket, 8);

	if (IsStandardRequest()) // Type = Standard
	{
		int bReq = CMD.setupPacket.bRequest;
		if (bReq<=USB_REQ_SET_SYNCH_FRAME)
		{
			voidFuncPtr func = stdRequests[bReq];
			if (func) {
				func();
				return;
			}
		}
        // any other request will receive Stall, see below
    }
	else if (IsClassRequest()) // Type = Class
	{
		int bReq = CMD.setupPacket.bRequest;
		switch (bReq)
		{
		// CDC requests
		case USB_CDC_REQ_GET_LINE_CODING:
			CDC_GetLineCoding(); // will send the requested data
			return;
		case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
            CDC_Set_DTR_RTS(); // extract DTR & RTS data
			break;
		case USB_CDC_REQ_SET_LINE_CODING: // will process next out packet
		case USB_CDC_REQ_SEND_BREAK: // not implemented
		default:
			break;

		// handling of any other class specific requests should come here

		}
		ACK();
		return;
	}
    // Type = Vendor or any other else are not implemented

    // for any not recognized nor supported request
    Stall_EPAddr(EP_CTRL_ADDR_OUT);
}

//-----------------------------------------------------------------------------
//  specific EP interrupt service routines in case of received data
//-----------------------------------------------------------------------------
void OnEpCtrlOut(void) // Control-EP OUT
{
	if (IsStandardRequest()) // reqType = Standard
	{
		// usually only zero length packets, ACK from Host, but also possible (although not yet seen) bRequest=7 = SET_DESCRIPTOR
		// nothing to do, the buffer was already read out
	}
	else if (IsClassRequest()) // reqType = Class
	{
		switch (CMD.setupPacket.bRequest)
		{
		// not implemented, but send anyway an ACK
		case USB_CDC_REQ_SET_LINE_CODING:
			CDC_SetLineCoding(); //  store transmitted data
			break;
		case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		case USB_CDC_REQ_SEND_BREAK:
		default:
			break;
		}
	}
	// we do not handle Vendor-Request
	//ACK(); // not necessary, it is just for us to know that everything went ok.
}
//-----------------------------------------------------------------------------
// finished control packet transmitted from device to host
//-----------------------------------------------------------------------------
void OnEpCtrlIn(void) // Control-EP IN
{
	if (IsStandardRequest()) // if Type = Standard
	{
		if (CMD.transferLen > 0) // continue transmitting the rest of data if any
		{
			TransmitSetupPacket();
			return;
		}
	}
	else if (IsClassRequest()) // reqType = Class
	{
		// nothing to do, this is just an ACK from host
	}
	//ACK(); // not necessary, is just for us to know that packet has been sent.
}

//-----------------------------------------------------------------------------
// Send data to USB host from ring buffer. Can send zero packets.
// Special handling because of ring buffer pointer characteristics which
// have to safeguard the read pointer against going out of boundary.
//-----------------------------------------------------------------------------
void USB_BeginDataTx(void)
{
	if ( transmitting==true ) return;

	uint16_t count = rb_read_available(&usbTxRB);
	if (count==0) return;

    transmitting = true;
	if (count > EP_DATA_LEN)
		count = EP_DATA_LEN;

    EpTable[EP_DATA].txCount = count;

    register uint32* wrPtr = (uint32*) EP_DATA_TX_BUF_ADDRESS;
	register uint16_t temp_tail = usbTxRB.ptrs.tail;
	register uint16 cap = usbTxRB.CAPACITY;
	int j = count/2;
	while (j--)
	{
	    register uint32 val;
	    val = usbTxRB.buffer[temp_tail++];
	    temp_tail &= cap;
	    val |= (usbTxRB.buffer[temp_tail++] << 8);
	    temp_tail &= cap;
	    *wrPtr++ = val;
	}
	if (count&1) // send last odd byte if any
	{
		register uint32 val = usbTxRB.buffer[temp_tail++];
		temp_tail &= cap;
		*wrPtr = val;
	}
	usbTxRB.ptrs.tail = temp_tail; // update volatile ptr

    MarkBufferTxReady(EP_DATA);
}
//-----------------------------------------------------------------------------
// Request for data to be transmitted to host via EP_DATA IN
//-----------------------------------------------------------------------------
void OnEpBulkIn(void)
{
	transmitting = false;
	USB_BeginDataTx();
}
//-----------------------------------------------------------------------------
// re-starts Rx process if previously NAK-ed
//-----------------------------------------------------------------------------
void USB_BeginDataRx(void)
{
	if ( ( receiving==false ) && ( rb_write_available(&usbRxRB) >= EP_DATA_LEN ) )
	{
		MarkBufferRxDone(EP_DATA); // set Rx buffer free for next data
	}
}
//-----------------------------------------------------------------------------
// Read received bytes and write them directly into the ring buffer
// This need special handling due to the "ring" characteristic,
// to safeguard the write pointer against going out of boundary
//-----------------------------------------------------------------------------
void OnEpBulkOut(void)
{
	receiving = false;
	// read number of available bytes
	uint16_t rxd = EpTable[EP_DATA].rxCount & 0x3FF;

	register rb_ptrs_t temp = { usbRxRB.ptrs.both };
	register uint16_t cap = usbRxRB.CAPACITY;
	uint16_t room = (temp.tail - temp.head - 1) & cap; // available space in buffer
	if (room < rxd)
	{ // not enough space for data
		return;
	}
	register uint16 temp_head = temp.head;
	// store the received bytes directly into the ring buffer
	// special handling due to nature of the ring buffer (size=power of 2)
	register uint32 * rdPtr = (uint32*) EP_DATA_RX_BUF_ADDRESS;
	uint16_t i = rxd/2;
	while (i--) {
		register uint32 val = *rdPtr++;
		usbRxRB.buffer[temp_head++] = val;
		temp_head &= cap;
		usbRxRB.buffer[temp_head++] = val>>8;
		temp_head &= cap;
	}
	if (rxd&1) { // for a last odd byte
		register uint32 val = *rdPtr;
		usbRxRB.buffer[temp_head++] = (uint8)val;
		temp_head &= cap;
	}
	usbRxRB.ptrs.head = temp_head; // update volatile ptr

    if ( room >= (rxd + EP_DATA_LEN) ) // is there enough room for a next full data packet
    {
		MarkBufferRxDone(EP_DATA); // set Rx buffer free for next data packet
    }
    else
    {
        // EP reply NAK (automatically set by HW)
    }
    if (dataHook!=NULL) dataHook();
}

//-----------------------------------------------------------------------------
//--------------- USB-Interrupt-Handler ---------------------------------------
//-----------------------------------------------------------------------------
void NAME_OF_USB_IRQ_HANDLER(void)
{
    uint32_t irqStatus = USB_ISTR; // Interrupt-Status

	if (irqStatus & WKUP) // Suspend-->Resume
	{
		USB_CNTR &= ~(FSUSP | LP_MODE);
		usb_state.suspended = false;
	} else
	if (irqStatus & SUSP) // after 3 ms break -->Suspend
	{
		usb_state.suspended = true;
		USB_CNTR |= (FSUSP | LP_MODE);
	}

	if (irqStatus & SOF) // Start of Frame, every 1 ms
	{
		usb_state.suspended = false;
	}
	// clear here the other interrupt bits
	USB_ISTR = ~(PMAOVR|ERR|WKUP|SUSP|RESET|SOF|ESOF); // clear diverse IRQ bits

	if (irqStatus & RESET) // Bus Reset
	{
		CMD.configuration = 0;
		InitEndpoints();
	}
	else
	{	// Endpoint Interrupts
		while ( (irqStatus = USB_ISTR) & CTR )
		{
			USB_ISTR = ~CTR; // clear IRQ bit
			uint8_t ep = irqStatus & MASK_EP;
			uint16_t epStatus = USB_EpRegs(ep);

			if (irqStatus & DIR) // OUT packet sent by host and received by device
			{
				USB_EpRegs(ep) = epStatus & ~CTR_RX & EP_MASK_NoToggleBits;

				if (ep == EP_CTRL)
				{
					if (epStatus & SETUP)
					{
						OnSetup(); // Handle the Setup-Packet
					}
					else
					{
						OnEpCtrlOut(); // finished TX on CTRL endpoint
					}
				}
				else if (ep == EP_DATA)
				{
					OnEpBulkOut();
				}
				else if (ep == EP_COMM)
				{
					OnEpIntOut();
				}
			}
			else // IN, finished packet transmitted from device to host
			{
				// Apply new device address here
				if (deviceAddress)
				{
					USB_SetAddress(deviceAddress);
					deviceAddress = 0;
				}

				if (!(epStatus&CTR_TX))
					return;
				USB_EpRegs(ep) = epStatus & ~CTR_TX & EP_MASK_NoToggleBits;

				if (ep == EP_CTRL)
				{
					OnEpCtrlIn();
				}
				else if (ep == EP_DATA)
				{
					OnEpBulkIn();
				}
				else if (ep == EP_COMM)
				{
					OnEpIntIn();
				}
			}
		}
	}
}

