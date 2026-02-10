
// fix for m5core gpio not defined
//#include "soc/gpio_struct.h"
//#include "hal/gpio_ll.h"
//----

#include <Arduino.h>

#include <M5Unified.h>
#include <Wire.h>
#include <ArduinoOTA.h>


#define LINE Serial.printf("%s:%d %s\n", __FILE__, __LINE__, __FUNCTION__)

#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <EEPROM.h>
#include <SPI.h>


typedef enum eP25FrameStates
{
	LOOK4SYNC
};

byte localBuffer[RECORDINGBUFFERSIZE];
char lclTextBuffer[RECORDINGBUFFERSIZE *2 + 1];


eP25FrameStates P25Frame = LOOK4SYNC;

#define SYNC_FRAME_HI 0xDDDDDDDDDDDDDDDD
#define SYNC_FRAME_LO 0xDDDDDDDD

uint64_t bigSync;

//-----------------------------------------------------------------------------
bool DetectSync(void)
{
	if (bigSync == SYNC_FRAME_HI) Serial.printf("FRAME\n");
}

void runP25(void)
{

	// setup async mode on CC1101 with GDO0 pin processing
	ELECHOUSE_cc1101.setCCMode(SYMBOL_TICK);
	ELECHOUSE_cc1101.setModulation(DEFAULT_MODULATION); //fsk-4
	ELECHOUSE_cc1101.EnterRxMode();
	
	//start recording to the buffer with bitbanging of GDO0 pin state
	Serial.print(F("\r\n New Sniffer enabled...\r\n"));

	// GD02 is constant clock at di-bit rate. (half the baud rate)
	ELECHOUSE_cc1101.setGDO2_hostpinMode(INPUT);

	ELECHOUSE_cc1101.enableRisingIRQ_GDO2(true);

	// Any received char over Serial port stops printing  RF received bytes

	uint32_t start = micros();
	uint32_t bitTime = start;
	
	while (!Serial.available())
	{
		
		// we have to use the buffer not to introduce delays
		for (int i = 0; i < RECORDINGBUFFERSIZE ; i++)
		{
		
			byte receivedbyte = 0;

			// di-bit count, move by 2 bits per symbol.
			for (int j = 7; j > 0 ; j -=2)						  // 8 bits in a byte
			{
				/*
					00 +600
					01 +1800
					10 -600
					11 -1800

					therefore a sync is +/-1800 hz
					therefore sync bit pattern is 01 11 or 0111
				*/
				bool ret = ELECHOUSE_cc1101.wait4RisingIRQ_GDO2();
				uint32_t now = micros();
				
				if (ret == true)
				{
					uint32_t signalTime = now - ulastTimeRisingGO2;
		
					if ( signalTime > 50)
					{
						Serial.printf("semaphore too long !!!!! = %d\n", signalTime);
						delay(-1);
					}

					uint32_t bitDelta = now - bitTime;
					bitTime = now;

					Serial.printf("bit time = %d\n", bitTime);
					
					
					ulastTimeRisingGO2 = 0; // just for fun
					
					bool bitHi, bitLo;
					
					// GDO0 points to one part of the di-bit.
					ELECHOUSE_cc1101.setGDOxPinConfig(CC1101_IOCFG0, 0x16, true);
					bitHi = digitalRead(PIN_GDO0);
					bitWrite(receivedbyte, j, bitHi);	// Capture GDO0 state into the byte

					// GDO0 points to the OTHER part of the di-bit.
					ELECHOUSE_cc1101.setGDOxPinConfig(CC1101_IOCFG0, 0x17, true);
					bitLo = digitalRead(PIN_GDO0);
					bitWrite(receivedbyte, j-1, bitLo); // Capture GDO0 state into the byte

					bigSync <<= 1; bigSync |= bitHi;
					bigSync <<= 1; bigSync |= bitLo;
    
					//DetectSync();
					if (bigSync == SYNC_FRAME_HI) Serial.print('.');
					
				}
				else
				{
					// stall timers until bitstream appears.
					ulastTimeRisingGO2 = now;
					bitTime = now;
					Serial.println("Error: GDO02 did not move in 3 seconds\n"); //should never happen.
				}	
			 }

			;
			// store the output into recording buffer
			localBuffer[i] = receivedbyte;
		}


		// when buffer full print the ouptput to serial port
		for (int i = 0; i < RECORDINGBUFFERSIZE ; i = i + 32)
		{
			binToAscii(&localBuffer[i], lclTextBuffer, 32);
			Serial.print((char *)lclTextBuffer);

		}
		
		ulastTimeRisingGO2 = micros(); // this print wrecks timing
		Serial.println();
		
	}; // end of While loop
	Serial.read();
	
	uint32_t deltaT = micros() - start;

	Serial.printf("\nStopping the new sniffer. up=%d dn=%d \n", irqUpCtrGDO2, irqDnCtrGDO2);
	Serial.printf("\nbitrate = %f\n", (float) irqUpCtrGDO2 / (float) deltaT);
	
	ELECHOUSE_cc1101.enableRisingIRQ_GDO2(false);


	// setting normal pkt format again
	ELECHOUSE_cc1101.setCCMode(GDO0_isSYNC_TXEND);
	ELECHOUSE_cc1101.setPktFormat(0);
	ELECHOUSE_cc1101.EnterRxMode();
}
