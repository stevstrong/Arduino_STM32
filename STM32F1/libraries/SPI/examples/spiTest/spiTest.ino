
#include "SPI.h"
uint8_t data[4];

volatile uint8_t cbDone; 
void txCallback()
{
  cbDone = 1;
}

void setup() {
  // put your setup code here, to run once:
	cbDone = 0;
	pinMode(PC13, OUTPUT);
	digitalWrite(PC13, HIGH);
	while (!Serial); delay(10);
	Serial.println("SPI begin");
  SPI.begin();
  SPI.onTransmit(txCallback);
  SPI.dmaSendAsync(data,4,1); 
  delay(50);
  if (cbDone) Serial.println("cb done");
  else Serial.println("cb not done");
 	cbDone = 0;
 SPI.onTransmit(NULL);   
  Serial.println("dmaSend");
  SPI.dmaSend(data,4);
  delay(50);
  Serial.println("Setup end");
}

void loop() 
{
	// put your main code here, to run repeatedly:
	delay(1000);
	if (cbDone)
	{
		cbDone = 0;
		Serial.println("cbDone");
	}
	Serial.println("loop");
}
