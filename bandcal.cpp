#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <Arduino.h>
#include "pretty.h"
#include <ArduinoOTA.h>


void beacon(uint32_t freq)
{
	byte binaryArray[50];

    // convert hex array to set of bytes
    int iCnt = sizeof(binaryArray);
    
	/////radio.setCCMode(I_DUNNO);

	radio.EnterIdleMode();
	
	radio.setCCMode(GDO0_isSYNC_TXEND);  //gdO = SYNC+Sent
	radio.setModulation(2); //ook
	radio.setBaudRate(300);
	radio.setMHZ(freq);
	radio.setPA(-30);
	
	Serial.println("wait for 5 seconds");
	delay(5000);
	
    for (int cnt= 0; cnt < 3;  cnt++)  //////////////////// 5
    {
        Serial.printf("\r\nTransmitting RF packet %d of 10\r\n", cnt);
		if (Serial.available())
		{
			Serial.read();
			break;
		}
		
		for (int i= 0; i < iCnt; i++) binaryArray[i] = random(255);

		radio.SendBinaryData(binaryArray, iCnt);

    	delay(500);
    	char temp[iCnt * 2 + 1];
    	
        binToAscii(binaryArray, temp, iCnt);
        
        Serial.print(F("Sent frame: "));
        Serial.print((char *)temp);
        Serial.print(F("\r\n"));
		// for DEBUG only
	}
	
	radio.EnterIdleMode();
	radio.setModulation(DEFAULT_MODULATION); //4fsk
	radio.setBaudRate(DEFAULT_BAUD);
	radio.setPA(0);

}
