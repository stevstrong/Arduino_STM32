
#ifndef USB_FUNC_H
#define USB_FUNC_H

#include <stdint.h>
#include "usb_def.h"
#include "usb_cdc_def.h"

// Structure of setup packet
typedef struct
{
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;       // number of bytes for next OUT stage
} setupPaket_t;

// Structure of mixed command and data block
typedef struct
{
    setupPaket_t setupPacket; // the last received setup packet
    int transferLen;          // bytes still to be transmitted
    int packetLen;            // length of packet
    uint8_t* transferPtr;     // pointer to data structure to be sent

    bool remoteWakeup;
    bool selfPowered;
    uint8_t configuration;
} command_t;


extern command_t CMD;
extern const uint8_t ZERO;

extern void USB_Start(void);
extern void EnableUsbIRQ();
extern void DisableUsbIRQ();
extern void USB_BeginDataTx(); // called when Tx data has to be sent
extern void USB_BeginDataRx(); // called when Rx data can be received again



#define BAUD_RATE 9600

//-----------------------------------------------------------------------------



#endif // USB_FUNC_H
