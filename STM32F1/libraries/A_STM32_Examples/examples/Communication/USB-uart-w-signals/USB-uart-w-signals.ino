/* distribution: MIT lic, ag123*/
#include "Arduino.h"
#include <usb_serial.h>
#include <HardwareSerial.h>
#include <libmaple/usart.h>
#include <libmaple/usb/usb_cdc_def.h>

void docmd();
bool chkcmd();
void setserial();
void configserial();
void bufsend();

// bluepill
//#define LED_BUILTIN PC13

uint8_t ledPin = LED_BUILTIN;
uint8_t dtrPin = PB12;
uint8_t rtsPin = PB13;

// this is the character used for the +++ escape sequence
// this would interfere with the escape sequence char of esp8266
// change this to another character of your preference to prevent issues
#define ESCSEQCHAR '+'

bool bcmd;
uint8_t fsendlf;
uint8_t dtr;
uint8_t rts;

#define INVERT(x) (~x)&1


bool bsetlinecoding = false;

//setup hook flags bsetlinecoding when SET_LINE_CODING is received
static void usbSetupHook(void)
{
	bsetlinecoding = true;
}


// the setup() method runs once when the sketch starts
void setup()
{
   //initialize the digital pin as an output:
   pinMode(ledPin, OUTPUT);
   digitalWrite(ledPin, HIGH);
   Serial.begin();

   //disable DTR checks, note this disable DTR "LEAF" magic sequence
   //replace the interface setup hook
   usb_cdcacm_set_hooks(USB_CDCACM_HOOK_IFACE_SETUP, usbSetupHook);

   //AN3155: usart boot loader requires even parity
   // but literally those parity stuff is not (yet) implemented in the core
   // and somehow it works! ;)
   //Serial1.begin(115200,SERIAL_8E1);
   Serial1.begin(115200,SERIAL_8N1);
   //Serial1.begin(74880,SERIAL_8N1);
   bcmd = false;
   fsendlf = 1;

   dtr = usb_cdcacm_get_dtr();
   rts = usb_cdcacm_get_rts();

   pinMode(dtrPin, OUTPUT);
   //TTL levels are inverted !
   digitalWrite(dtrPin, INVERT(dtr));
   pinMode(rtsPin, OUTPUT);
   digitalWrite(rtsPin, INVERT(rts));

}


//the loop() method runs over and over again,
//as long as maple has power
void loop() {

	if(bsetlinecoding) {
		//configure uart based on baud and line discipline from host
		configserial();
		bsetlinecoding = false;
	}

	if (usb_cdcacm_get_dtr() != dtr) {
		dtr = INVERT(dtr);
		digitalWrite(dtrPin, INVERT(dtr));
	}

	if (usb_cdcacm_get_rts() != rts) {
		rts = INVERT(rts);
		digitalWrite(rtsPin, INVERT(rts));
	}

	if(Serial1.available()) {
		digitalWrite(ledPin, HIGH); // blink the led for traffic
		while(Serial1.available()) {
			char c = Serial1.read();
			Serial.write(c);
		}
		digitalWrite(ledPin, LOW);
	}

	if(Serial.available()) {
		digitalWrite(ledPin, HIGH); // blink the led for traffic
		// command processor
		chkcmd();
		while(Serial.available()) {
			char c = Serial.read();
			Serial1.write(c);
			if(c == 13 && fsendlf) Serial1.write(10);
		}
	}
	digitalWrite(ledPin, LOW);

	//delay(1);//sleep for 1ms
	asm("wfi");
}

static byte cmdline[20];

//commands here run after the "+++"
//any non matching commands exits the 'command mode'
//update the commands according to your needs
void docmd() {
	uint8_t len = 0;
	memset(&cmdline,0,20);
	while(true) {
		if(Serial.available()) {
			char c = Serial.read();
			Serial.write(c);
			if(c == '\n' || c == '\r' )
				break;
			else if (c == 127 || c == 8) {
				len--;
			} else {
				cmdline[len++] = c;
			}
		}
		asm("wfi");
	}
	if(len==0) return;
	switch(cmdline[0]) {
	case 'R': //reset
		digitalWrite(rtsPin, 0);
		delay(500);
		digitalWrite(rtsPin, 1);
		break;
	case 'P': //program mode esp-01 gpio0 low
		digitalWrite(dtrPin, 0);
		break;
	case 'p': //run mode esp01 gpio0 high
		digitalWrite(dtrPin, 1);
		break;
	case 'D': //disconnect from AP
		Serial1.print("AT+CWQAP\r\n");
		break;
	case 'i': //print ip address
		Serial1.print("AT+CIFSR\r\n");
		break;
	case 'M': //configure MDNS
		Serial1.print("AT+MDNS=1,\"esp8266\",\"http\",80\r\n");
		break;
	case 'L': //logon to AP L"ssid","password"
		Serial1.print("AT+CWJAP_CUR=");
		Serial1.print((const char *)(cmdline + 1));
		Serial1.print("\r\n");
		break;
	case 'W': //start the server mode
		// softAP+station mode
		//Serial1.print("AT+CWMODE=3\r\n");
		//enable multi connections
		Serial1.print("AT+CIPMUX=1\r\n");
		delay(50);
		//enable server
		Serial1.print("AT+CIPSERVER=1,80\r\n");
		break;
	case 's': //send some characters, s{channel}{message}
		//e.g. s0type_your_message, note cmdline is 20-2 chars!
		Serial1.print("AT+CIPSEND=");
		Serial1.print((char) cmdline[1]);
		Serial1.print(',');
		Serial1.print(strlen((const char *)(cmdline + 2)));
		Serial1.print("\r\n");
		delay(50);
		Serial1.print((const char *)(cmdline + 2));
		break;
	case 'S': //send S{channel},{length} e.g. S0,100
		Serial1.print("AT+CIPSEND=");
		Serial1.print((const char *)(cmdline + 1));
		Serial1.print("\r\n");
		break;
	case 'T'://send a bunch of characters, it computes length automatically
		//T{channel} e.g. T0
		bufsend();
		break;
	case 'c'://close channel c{channel} e.g. c0
		Serial1.print("AT+CIPCLOSE=");
		Serial1.print((char) cmdline[1]);
		Serial1.print("\r\n");
		break;
	case 'k':
		fsendlf = INVERT(fsendlf);
		Serial.println(fsendlf);
		break;
	case 'b':
		setserial();
		break;
	default:
		Serial.println("<+++");
		bcmd = false;
	}
}

//configures serial baud and flags from configuration received from host
void configserial()
{
usb_cdcacm_line_coding_t lcData;
	usb_cdcacm_get_line_coding(&lcData);

// only following configurations are supported:
// SERIAL_9N1
// SERIAL_9N2
// SERIAL_8O1
// SERIAL_8O2
// SERIAL_8E1
// SERIAL_8E2
// SERIAL_8N1
// SERIAL_8N2
	uint8_t cfg = 0;

	uint8_t stopBits = lcData.stopBits; // 0: 1 Stop bit; 1: 1.5 Stop bits; 2: 2 Stop bits
	uint8_t parity = lcData.parityType; // 0:None; 1: Odd; 2: Even; 3: Mark; 4: Space
	uint8_t dataBits = lcData.dataBits; // Data bits (5, 6, 7, 8 or 16)
	if (dataBits == 9) { // 9 data bits
		if (stopBits == 2) {
			cfg = SERIAL_9N2;
		} else {
			cfg = SERIAL_9N1;
		}
	} else { // otherwise only 8 bit is supported 
		if (parity == 1) { // odd
			if (stopBits == 2) {
				cfg = SERIAL_8O2;
			} else {
				cfg = SERIAL_8O1;
			}
			
		} else if (parity == 2) { // even
			if (stopBits == 2) {
				cfg = SERIAL_8E2;
			} else {
				cfg = SERIAL_8E1;
			}
		} else { // no parity or other
			if (stopBits == 2) {
				cfg = SERIAL_8N2;
			} else {
				cfg = SERIAL_8N1;
			}
		}
	}

	Serial1.begin(lcData.baudRate, cfg);

}

void setserial() {
	uint32 baud = atoi((const char *) (cmdline+1));
	Serial.println(baud);
	Serial1.begin(baud, SERIAL_8N1);
}

//send a bunch of characters, it computes length automatically
//T{channel} e.g. T0
void bufsend() {
	char buf[512];
	uint16_t timeout = 5000;
	uint16_t len=0;

	memset(buf,0,512);
	while(timeout > 0) {
		if(Serial.available()){
			char c = Serial.read();
			buf[len++] = c;
			//if(c == 13 && fsendlf) buf[len++] = 10;
			Serial.print(c);
		}
		timeout--;
		delay(1);
	}
	Serial1.print("AT+CIPSEND=");
	Serial1.print((char) cmdline[1]);
	Serial1.print(',');
	Serial1.print(len);
	Serial1.print("\r\n");
	delay(50);
	for(uint16_t i=0;i<len;i++)
		Serial1.print(buf[i]);
}
// this checks for the sequence "+++" and pause for 1/2 second
// once detected it falls into command mode
bool chkcmd() {
	uint16_t pluscount=0;
	uint16_t timeout=0;
	char c = 0;

	if(bcmd) {
		docmd();
		return false;
	}

	//escape sequence enter "+++" and wait 1/2 second
	while(timeout < 500 && pluscount < 4) {
		if(Serial.available()) {
			c = Serial.read();
			if(c == ESCSEQCHAR) {
				pluscount++;
				timeout = 0;
			} else {
				break;
			}
		}
		//asm("wfi"); //delay 1ms
		delay(1);
		timeout++;
	}
	if(pluscount==3 && timeout==500) {
		bcmd = true;
		Serial.println("+++");
		return true;
	} else {
		for(uint16_t i=0; i<pluscount; i++)
			Serial1.write(ESCSEQCHAR);
		if(c != ESCSEQCHAR) Serial1.write(c);
		if(c == 13 && fsendlf) Serial1.write(10);
	}
	return false;
}
