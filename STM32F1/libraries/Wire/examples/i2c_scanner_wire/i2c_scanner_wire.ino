// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//


#if 0

#include <SoftWire.h>
SoftWire SWire;
#define Wire SWire

#else

#include <Wire.h>

#endif
//use IIC2
//TwoWire WIRE2 (2,I2C_FAST_MODE);
//#define Wire WIRE2

uint32_t blinkTime;
void yield(void)
{
	Blink();
}
//-----------------------------------------------------------------------------
void Blink(void)
{
	if ( (millis()-blinkTime)<1000 ) return;
	blinkTime = millis();
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
	Serial.print("I2C1 SR1: "); Serial.print(I2C1->regs->SR1, HEX);
	Serial.print(", SR2: "); Serial.print(I2C1->regs->SR2, HEX);
	Serial.println();
//	Serial.print("I2C2 SR1: "); Serial.print(I2C2->regs->SR1, HEX);
//	Serial.print(", SR2: "); Serial.print(I2C2->regs->SR2, HEX);
//	Serial.println();
}
//-----------------------------------------------------------------------------
void setup()
{
 delay(100);
 Wire.begin();
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
 // while (!Serial);
 delay(100);
  Serial.println("\nI2C Scanner");
}

//-----------------------------------------------------------------------------
void loop()
{
  Serial.println("Scanning >>");

  uint8_t nDevices = 0;
  for(uint8_t address = 0x76; address < 0x79; address++)
  {
	Blink();
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("0x"); Serial.print(address, HEX);
      Serial.print(" - OK");
      nDevices++;
    }
//    else if (error == 4) {
//      Serial.println(" - unknown error.");
//    }
	else {
		//Serial.print(" ! Error: "); Serial.print(error);
	}
    Serial.println();
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else {
    Serial.print(">> done. Found "); Serial.print(nDevices); Serial.println(" device(s).");
  }

  delay(3000);           // wait 5 seconds for next scan
  Blink();
}
