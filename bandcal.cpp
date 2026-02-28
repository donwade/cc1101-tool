#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <Arduino.h>
#include "pretty.h"
#include <ArduinoOTA.h>
#include "keyboard.h"

//-------------------------------------------------------------

void beaconByCW (uint32_t freq, bool bCalMode)
{
	radio.EnterIdleMode();
	radio.setCCMode(BEACON);
	
	//normal becon op has NO Cal tweak.
	radio.setFreqHz(freq, false, bCalMode);  // be quiet and skip band cal.
	radio.setModulation(2); //ook

	radio.setPA(-30);
    radio.setGDO0_hostpinMode(OUTPUT);

    radio.SpiStrobe(CC1101_STX);      //start send

	for (int i= 0; i < 10; i++)
	{
		if (i & 1)
		{
			radio.writeGDO0pin(true);
		}
		else
		{
			radio.writeGDO0pin(false);
		}
		Serial.printf("%c", i&1 ? '+':'-');
		delay(250);
	}

	
	radio.writeGDO0pin(false);
	
    radio.SpiStrobe(CC1101_SIDLE);
    radio.setGDO0_hostpinMode(INPUT);
	
}

//-------------------------------------------------------------
void beaconByFifo(uint32_t freq, bool bCalMode)
{
	byte binaryArray[50];
	
    // convert hex array to set of bytes
    int sendLen = sizeof(binaryArray);
    
	radio.EnterIdleMode();
	
	radio.setCCMode(GDO0_isSYNC_TXEND);  //gdO = SYNC+Sent
	radio.setModulation(2); //ook
	radio.setBaudRate(300);
	
	//normal becon op has NO Cal tweak.
	radio.setFreqHz(freq, false, bCalMode);  // be quiet and skip band cal.

	radio.setPA(-30);
	
    for (int cnt= 0; cnt < 1;  cnt++)  //////////////////// 5
    {
    	char temp[sendLen * 2 + 1];

		// scramble the data    	
		for (int i= 0; i < sendLen; i++) binaryArray[i] = 0x00;		// all zeros- CW

        binToAscii(binaryArray, temp, sendLen);
        Serial.printf("%s:%d send %s\n", __FUNCTION__, __LINE__, temp);

		radio.SendBinaryData(binaryArray, sendLen);

    	delay(100);
	}
	
	radio.EnterIdleMode();
	
}

//-------------------------------------------------------------

uint32_t pwr(uint8_t exp)
{
	uint32_t ret = 1;
	while (exp--) ret *= 10;
	return ret;
}

//-------------------------------------------------------------

int32_t REMAP( int32_t freqHzIn, int32_t freqLeft, int32_t calLeft, int32_t freqRight, int32_t calRight)
{
	int32_t dFreq= freqRight - freqLeft;
	int32_t dCal=  calRight -  calLeft;
	double slope = (double)dCal/(double)dFreq;

	double y = calLeft - (slope * (double)(freqLeft - freqHzIn)) ; // + (double) calLeft;

#if 0
	Serial.printf("dFreq = %d\n", dFreq);
	Serial.printf("dCal = %d\n", dCal);
	Serial.printf("slope = %f\n", slope);
	

#endif

	Serial.printf("IN %d  LHS=[%d %d] RHS=[%d %d] OUT= %f\n",  
			freqHzIn, freqLeft, calLeft, freqRight, calRight, y);

	return y;	

}

//-------------------------------------------------------------

void bandCalKnob(int32_t startFreq)
{
	int32_t tweaker = 0;
	uint8_t digitSel = 3;
	bool bCalMode = true;	// normal val for doing band cal.
	
	if (startFreq < 0) 
	{
		bCalMode = false;
		startFreq = -startFreq;
	}
	
	// specified Mhz or Hz?
	if (startFreq < 1000) startFreq *= 1000000;


	/*
	int32_t foo = REMAP( 10, 1,1, 2,2);
	
	 foo = REMAP( -10,   1, 1,  2, 2);
	 foo = REMAP( 100, 100,15, 200,25);
	 foo = REMAP(  70, 100,15, 200,25);
	 foo = REMAP( 200, 100,15, 200,25);
	 
	 foo = REMAP(  15, 100,15, 200,25);
 	 foo = REMAP(  30, 100,15, 200,25);
 	*/

 	
	uint32_t frozen = startFreq;
	Serial.printf("\tuse left/right arrows to adj\n\tuse up/down to adj 10's\n\tspace to exit\n");
	delay(2000);
	
	Serial.printf(">>> initial >>> %d offset=%d %d decade=%d\n", frozen, startFreq - frozen, startFreq, digitSel);
	beaconByCW(startFreq, bCalMode);

	Serial.printf(FG_CYAN "Cal Ranges 300-348Mhz 378-464Mhz 779-899Mhz 900-928Mhz\n");
	Serial.printf("see Band_779_899 etc\n\n" FG_DONE);
	
	while (true)
	{
		
		KEYS keystroke = getKey();
		if (keystroke == 0x20) break;  //exit if needed

		
		switch (keystroke)
		{
			case UP:
				if (digitSel >= 5)
				{
					Serial.printf("digit 5 is highest\n");
				}
				else
				{
					digitSel++;
				}
			break;
			
			case DOWN:
				if (digitSel > 1)
				{
					digitSel--;
				}
				else
				{
					Serial.printf("tens digit is the smallest\n");
				}
			break;
			
			case LEFT:
				startFreq -= pwr(digitSel);
				beaconByCW(startFreq, bCalMode);
			break;
			
			case RIGHT:
				startFreq += pwr(digitSel);
				beaconByCW(startFreq, bCalMode);
			break;
			
		}
		
		Serial.printf(">>>>>>>>>>>>>>>> %d offset=%d %d decade=%d\n", frozen, startFreq - frozen, startFreq, digitSel);
		
	}
}


