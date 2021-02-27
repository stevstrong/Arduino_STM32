
#include <Tasker.h>
#include "Streaming.h"


Tasker tasker;

#define LED PC13 //LED_BUILTIN

int32 overflow, comp_r, comp_g, comp_b, step, limit;
uint32 ctrl;
uint32 stage, prg;

uint32 funct[] = {
	0b000001,
	0b000010,
	0b000100,
	0b001000,
	0b010000,
	0b100000,
	0b000101,
	0b001010,
	0b010100,
	0b101000,
	0b010001,
	0b100010,
	0b010101,
	0b101010,
};
//-----------------------------------------------------------------------------
void blink()
{
	digitalWrite(LED, !digitalRead(LED));
}

//-----------------------------------------------------------------------------
void LED_Control()
{
	uint32 status = 0;
	if (ctrl&BIT0)
	{
		comp_r += step;
		if (comp_r>=limit)
		{
			comp_r = limit;
			status |= BIT0;
		}
	}
	else if (ctrl&BIT1)
	{
		comp_r -= step;
		if (comp_r<=0)
		{
			comp_r = 0;
			status |= BIT1;
		}
	}
	if (ctrl&BIT2)
	{
		comp_g += step;
		if (comp_g>=limit)
		{
			comp_g = limit;
			status |= BIT2;
		}
	}
	else if (ctrl&BIT3)
	{
		comp_g -= step;
		if (comp_g<=0)
		{
			comp_g = 0;
			status |= BIT3;
		}
	}
	if (ctrl&BIT4)
	{
		comp_b += step;
		if (comp_b>=limit)
		{
			comp_b = limit;
			status |= BIT4;
		}
	}
	else if (ctrl&BIT5)
	{
		comp_b -= step;
		if (comp_b<=0)
		{
			comp_b = 0;
			status |= BIT5;
		}
	}
	
	Timer2.setCompare(TIMER_CH1, comp_r);
	Timer2.setCompare(TIMER_CH2, comp_g);
	Timer2.setCompare(TIMER_CH3, comp_b);
	
	Serial << "r = " << comp_r << ", g = " << comp_g << ", b = " << comp_b << endl;
	
	if (status)
	{
		if ( (++prg)>=(sizeof(funct)/sizeof(uint32)) )
			prg = 0;
		
		ctrl = funct[prg];
	}
}

//-----------------------------------------------------------------------------
void setup()
{
	pinMode(LED, OUTPUT);
  // put your setup code here, to run once:
	Timer2.init();
	pinMode(PA0, PWM); // Tim2 ch1
	pinMode(PA1, PWM); // Tim2 ch2
	pinMode(PA2, PWM); // Tim2 ch3
	Timer2.setPeriod(10000);
	overflow = Timer2.getOverflow();
	step = overflow/100;
	limit = overflow/2;
	while (!Serial); delay(10);
	Serial << "Overflow = " << overflow << endl;
	Timer2.setCompare(TIMER_CH1, comp_r);
	Timer2.setCompare(TIMER_CH2, comp_g);
	Timer2.setCompare(TIMER_CH3, comp_b);
	Timer2.refresh();
	Timer2.resume();
	
	tasker.setInterval(blink, 500);
	tasker.setInterval(LED_Control, 100);
	
	ctrl = funct[0];
}

//-----------------------------------------------------------------------------
void loop()
{
double n = min(2,3);
isnan(n);

	tasker.loop();

}
