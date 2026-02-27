#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <Arduino.h>
#include "pretty.h"
#include <ArduinoOTA.h>
#include "keyboard.h"

//-------------------------------------------------------------
void beacon(uint32_t freq)
{
	byte binaryArray[50];
	
    // convert hex array to set of bytes
    int sendLen = sizeof(binaryArray);
    
	radio.EnterIdleMode();
	
	radio.setCCMode(GDO0_isSYNC_TXEND);  //gdO = SYNC+Sent
	radio.setModulation(2); //ook
	radio.setBaudRate(300);
	radio.setFreqHz(freq, true, true);  // be quiet and skip band cal.
	radio.setPA(-30);
	
    for (int cnt= 0; cnt < 1;  cnt++)  //////////////////// 5
    {
    	char temp[sendLen * 2 + 1];

		// scramble the data    	
		for (int i= 0; i < sendLen; i++) binaryArray[i] = random(255);

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

void bandCal(uint32_t startFreq)
{
	int32_t tweaker = 0;
	uint8_t digitSel = 3;

	// specified Mhz or Hz?
	if (startFreq < 1000) startFreq *= 1000000;

	uint32_t frozen = startFreq;

	Serial.printf(">>>>>>>>>>>>>>>> %d offset=%d %d decade=%d\n", frozen, startFreq - frozen, startFreq, digitSel);
	beacon(startFreq);

	Serial.printf(FG_CYAN "Cal Ranges 300-348Mhz 378-464Mhz 779-899Mhz 900-928Mhz\n");
	Serial.printf("see hwTweakHz_779_899Mhz etc\n\n" FG_DONE);
	
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
				beacon(startFreq);
			break;
			
			case RIGHT:
				startFreq += pwr(digitSel);
				beacon(startFreq);
			break;
			
		}
		
		Serial.printf(">>>>>>>>>>>>>>>> %d offset=%d %d decade=%d\n", frozen, startFreq - frozen, startFreq, digitSel);
		
	}
}
