#include <Arduino.h>
#include "keyboard.h"

//#define LINE Serial.printf("%s:%d ---- \n", __FUNCTION__, __LINE__);

void dumpBinary(uint8_t *input, uint16_t len, uint8_t width)
{
	int numLines = (len + width/2)/ width; // round up.

	for (int down = 0; down < numLines; down++)
	{
		int index;
		
		Serial.printf("[%04X] : ", down * numLines);
		for (int across = 0; across < width; across++)
		{
			index = down * numLines + across;
			if (index > len) break;
			Serial.printf("%02X ", input[ index]);
		}
		Serial.printf("    ");

		for (int across = 0; across < width; across++)
		{
			index = down * numLines + across;
			if (index > len) break;
			char c = input[ index];
			Serial.printf("%c", ( c < 0x20 || c > 0x7F) ? '.' : c);
		}
		Serial.println();
	}
	Serial.println();
	
}

KEYS getKey(bool bBlocking)
{
	static uint8_t keystroke;
	static uint8_t kstate = 0;
	bool press;
	static uint32_t last_time;
	
	while(true)
	{
		// not blocking.
		if (bBlocking)
		{
			while (!Serial.available()) delay(1);
			press = true;
		}
		else
		{
			press = Serial.available();
			if (!press) return NOPRESS;
		}
		
		if (!bBlocking && !press) return NOPRESS;

		// keystroke available
		keystroke = Serial.read();

		uint32_t diff = millis() - last_time;
		last_time = millis();

		if (diff > 20 && kstate > 0)
		{
			kstate = 0;
			return (KEYS) keystroke;
		}
		
		if (kstate == 0 && keystroke == 0x1B)
		{
			kstate = 1;
			if (!bBlocking) return NOPRESS;
		}
		else if (kstate == 1 && keystroke == 0x5B)
		{
			kstate = 2;
			if (!bBlocking) return NOPRESS;
		}
		else if (kstate == 2 && keystroke < 0x45 && keystroke > 40)
		{
			kstate = 0;
			switch (keystroke)
			{
				case 0x44:
					return LEFT;
					
				case 0x43:
					return RIGHT;
					
				case 0x41:
					return UP;
					
				case 0x42:
					return DOWN;
			}
			if (bBlocking) return NOPRESS;
		}
		else
		{
			kstate = 0;
			return (KEYS) keystroke;
		}
	}		
}

/*
while(true)
{
	KEYS test = getKey(false);
	if (test == NOPRESS) continue;
	Serial.printf("HI DON %d 0x%02X %c\n", test, test, test);
}
*/


