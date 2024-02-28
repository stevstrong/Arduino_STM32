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
// Version 6, August 1, 2015
//    Modified to support HardWire for STM32duino
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>
#define SCL_PIN PB6 // BOARD_I2C1_SCL_PIN
#define SDA_PIN PB9 // BOARD_I2C1A_SDA_PIN, alternate pin

// TwoWire HWire(1, I2C_FAST_MODE); // I2c1
TwoWire HWire(1, SCL_PIN, SDA_PIN, I2C_FAST_MODE); // I2c1
// optionally, it is possible to use the default declared Wire(1) instance with normal speed
//#define HWire Wire

void setup()
{
  Serial.begin(115200);
  while(!Serial); delay(1000);
  HWire.begin();
  Serial.println("\nI2C Scanner");
}

uint32_t counter;

void loop() {
  byte error, address;
  int nDevices;

  Serial.print(counter++);
  Serial.println(": Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    HWire.beginTransmission(address);
    error = HWire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");

  delay(5000);           // wait 5 seconds for next scan
}
