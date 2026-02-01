/*
 * ELECHOUSE_CC1101.cpp - CC1101 module library
 * Copyright (c) 2010 Michael.
 *  Author: Michael, <www.elechouse.com>
 *  Version: November 12, 2010
 *
 * This library is designed to use CC1101/CC1100 module on Arduino platform.
 * CC1101/CC1100 module is an useful wireless module.Using the functions of the
 * library, you can easily send and receive data by the CC1101/CC1100 module.
 * Just have fun!
 * For the details, please refer to the datasheet of CC1100/CC1101.
 * ----------------------------------------------------------------------------------------------------------------
 * cc1101 Driver for RC Switch. Mod by Little Satan. With permission to modify and publish Wilson Shen (ELECHOUSE).
 * ----------------------------------------------------------------------------------------------------------------
 */
#include <SPI.h>
#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <Arduino.h>
#include "pretty.h"
#include <ArduinoOTA.h>

extern ArduinoOTAClass ArduinoOTA;


#define LINE Serial.printf(">>> %s:%d %s\n", __FILE__, __LINE__, __FUNCTION__)

SemaphoreHandle_t sem_GDO0_UP = xSemaphoreCreateBinary();
SemaphoreHandle_t sem_GDO0_DN = xSemaphoreCreateBinary();
SemaphoreHandle_t sem_GDO2_UP = xSemaphoreCreateBinary();
SemaphoreHandle_t sem_GDO2_DN = xSemaphoreCreateBinary();

bool GDO0_waitFalling();
bool GDO0_waitRising();
bool GDO2_waitFalling();
bool GDO2_waitRising();

bool bGDO0_HasFallingCallback;
bool bGDO0_HasRisingCallback;
bool bGDO2_HasFallingCallback;
bool bGDO2_HasRisingCallback;


uint32_t irqUpCtrGDO0;
uint32_t irqDnCtrGDO0;
uint32_t irqUpCtrGDO2;
uint32_t irqDnCtrGDO2;

static uint32_t irqLastTimeGDO0;
static uint32_t irqLastTimeGDO2;

uint32_t irqDeltaTimeGDO0;
uint32_t irqDeltaTimeGDO2;


#ifdef ARDUINO_M5STACK_CORES3
  SPIClass MY_SPI( FSPI);
#else
  SPIClass MY_SPI( VSPI);
#endif

/****************************************************************/
#define   WRITE_BURST       0x40            //write burst
#define   READ_SINGLE       0x80            //read single
#define   READ_BURST        0xC0            //read burst
#define   BYTES_IN_RXFIFO   0x7F            //byte number in RXfifo
#define   max_modul 6

byte modulation = 2;
byte logical_chan = 0;
int pa = 12;
byte last_pa;
byte SCK_PIN;
byte MISO_PIN;
byte MOSI_PIN;
byte SS_PIN;
byte GDO0;
byte GDO2;
bool spi = 0;
eGDIO_MODES ccmode = LEGACY_0;
eMODEM_STATE trxstate = MODEM_IDLE;
float gMHz = 905.0;
float tweakFreqHz =  0;

byte pc0PktForm;
byte pc0LenConf;


// NOTE: this is now expressed in hertz, not Smartnet vals
int32_t hwTweakHz_300_348Mhz[2] = {  2000,    3000 };	// made up
int32_t hwTweakHz_378_464Mhz[2] = {  3000,    4000 };	// made up
int32_t hwTweakHz_779_899Mhz[2] = { -25261, -25261 };	// CAL'd
int32_t hwTweakHz_900_928Mhz[2] = { 52666, 52666 }; // CAL'd


int16_t mirror[64];


static const double XTAL_Mhz=26.0;
static const double XTAL_Hz=( 26.0 * 1e6);

/****************************************************************/
uint8_t PA_TABLE[8]     { 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//                       -30  -20  -15  -10   0    5    7    10
uint8_t PA_TABLE_315[8] { 0x12, 0x0D, 0x1C, 0x34, 0x51, 0x85, 0xCB, 0xC2, };                //300 - 348
uint8_t PA_TABLE_433[8] { 0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0, };                //387 - 464
//                        -30  -20  -15  -10  -6    0    5    7    10   12
uint8_t PA_TABLE_868[10] { 0x03, 0x17, 0x1D, 0x26, 0x37, 0x50, 0x86, 0xCD, 0xC5, 0xC0, };   //779 - 899.99
//                        -30  -20  -15  -10  -6    0    5    7    10   11
uint8_t PA_TABLE_915[10] { 0x03, 0x0E, 0x1E, 0x27, 0x38, 0x8E, 0x84, 0xCC, 0xC3, 0xC0, };   //900 - 928


template <typename T> void binary( T input)
{
	T copy = input;
	for(int i = sizeof(T)*8 - 1; i > -1; i--)
	{
		Serial.printf("%d", !!(input & (1 << i)));
	}
}




template <typename T> T regMask( T &final, T newField, uint8_t lhs, uint8_t rhs)
{
	T original = final;
	T wide = (lhs - rhs) + 1;
	T mask = 0;
	T oldField;
	
	assert (lhs >= rhs);
	// make a bunch of ones
	for (int i = 0; i < wide; i++) 
	{	
		mask *= 2;
		mask |=1;
	}
	mask = mask << rhs;

	oldField = (final & mask) >> rhs;
	
	final &= ~mask;
	final |= newField << rhs;

#if 0
	Serial.printf("\n\tfield %d:%d  oldField=%02X newField=%02X\n",
					lhs, rhs, oldField, newField);
	Serial.printf("\tmask     = "); binary(mask); 		Serial.printf(" 0x%02X\n", mask);
	Serial.printf("\toriginal = "); binary(original);	Serial.printf(" 0x%02X\n", original);
	Serial.printf("\tfinal    = "); binary(final);		Serial.printf(" 0x%02X\n", final);
#endif
	return final;
}


#define SpiWriteReg(name, value) _SpiWriteReg(#name, name, value)
#define regRMW(name, val, lhs, rhs) _regRMW(#name, name, val, lhs, rhs)

//---------------------------------------------------------------------

void ELECHOUSE_CC1101::_regRMW(const char *regName, uint8_t regNum, uint8_t bits, uint8_t LHS, uint8_t RHS)
{
	uint8_t orig = SpiReadReg(regNum);
	
	if ( mirror[regNum] >= 0)
	{
		if (mirror[regNum] != orig)
		{
			Serial.printf(FG_BRED "\n[0x%X] %s REG ERROR\n" _DONE, __FUNCTION__, regNum, regName);
			Serial.printf(FG_BRED "\t expect ");
			binary((uint8_t) mirror[regNum]);
			Serial.printf("but found  ");
			binary(orig);
			Serial.println(_DONE);
		}
		else
		{
			// Serial.printf(FG_BGREEN "\n%s REG PASS %s [0x%X]\n" _DONE, __FUNCTION__, regName, regNum);
			// Serial.print("\t expect = found ");
			// binary((uint8_t) mirror[regNum]);
			// Serial.println();
		}
	}

	uint8_t temp = orig;
	uint8_t want = regMask<uint8_t> ( temp, bits, LHS, RHS);

	Serial.printf("\n[0x%02X] %s 0x%02X\n", regNum, regName, want ); 
	
	
	if(orig != want)
	{
		int x;
		for (x = 0; x < 10; x++)
		{
			_SpiWriteReg(regName, regNum, want, 1); //silent
			orig = SpiReadReg(regNum);
			if (orig == want) break;
		}
		
		if (x == 10) Serial.printf(FG_RED "\n%s FAIL TO WRITE %s want 0x%X found 0x%X\n" _DONE, __FUNCTION__, regName, want, orig);
	}	
}	
//---------------------------------------------------------------------

void bin (unsigned char byte) {
    for (int i = 7; i >= 0; i--) {
        // Use bitwise AND (&) and right shift (>>) to check each bit
        Serial.printf("%d", (byte >> i) & 1);
    }
    
}

//---------------------------------------------------------------------

void ELECHOUSE_CC1101::DumpRegs(void)
{
	int8_t regs;
	Serial.println("-----------------------------------------");
	
	for (regs = 0 ; regs < 0x30; regs++)
	{	
		uint8_t read = SpiReadReg(regs);
		Serial.printf("\t0x%02X    0x%02X  ", regs, read);
		bin(read);
		Serial.println();
	}
}
//-------------------------------------------------------------

ICACHE_RAM_ATTR void onGDO0_IRQ(void)
{
	uint32_t now = micros();
	
	irqDeltaTimeGDO0 = now - irqLastTimeGDO0;
	irqLastTimeGDO0 = now;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if (digitalRead(GDO0))
	{
		if (bGDO0_HasRisingCallback)
		{
			irqUpCtrGDO0++;
			xSemaphoreGiveFromISR( sem_GDO0_UP, &xHigherPriorityTaskWoken );
		}
	}
	else
	{
		if (bGDO0_HasFallingCallback)
		{
			irqDnCtrGDO0++;
			xSemaphoreGiveFromISR( sem_GDO0_DN, &xHigherPriorityTaskWoken );
		}
	}
	
	// wake up task that needs it.
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//-------------------------------------------------------------

ICACHE_RAM_ATTR void onGDO2_IRQ(void)
{
	uint32_t now = micros();
	
	irqDeltaTimeGDO2 = now - irqLastTimeGDO2;
	irqLastTimeGDO2 = now;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if (digitalRead(GDO2))
	{
		if (bGDO2_HasRisingCallback)
		{
			irqUpCtrGDO2++;
			xSemaphoreGiveFromISR( sem_GDO2_UP, &xHigherPriorityTaskWoken );
		}
	}
	else
	{
		if (bGDO2_HasFallingCallback)
		{
			irqDnCtrGDO2++;
			xSemaphoreGiveFromISR( sem_GDO2_DN, &xHigherPriorityTaskWoken );
		}
	}
	
	// wake up task that needs it.
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/****************************************************************
* FUNCTION NAME:SpiStart
* FUNCTION     :spi communication start
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SpiStart(void)
{

   return;   // quit restarting the spi engine.
   
   // enable MY_SPI
#ifdef ESP32
    //MY_SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
#else
    MY_SPI.begin();
	#error NOPE
#endif
}


/****************************************************************
* FUNCTION NAME:SpiEnd
* FUNCTION     :spi communication disable
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SpiEnd(void)
{
    // disable MY_SPI
    MY_SPI.endTransaction();
    //MY_SPI.end();   // DWADE DO NOT DO THIS!!! 
}


/****************************************************************
* FUNCTION NAME: GDOx_SetPinMode()
* FUNCTION     : set GDO0,GDO2 pin for serial pinmode.
* INPUT        : none
* OUTPUT       : none
****************************************************************/
void ELECHOUSE_CC1101::GDOx_SetPinMode(void)
{
    setGDO0_hostpinMode(OUTPUT);
    setGDO2_hostpinMode(INPUT);
    
    irqDirGDO0 = -1;
    irqDirGDO2 = -1;
    
}


/****************************************************************
* FUNCTION NAME: GDO0_SetPinMode()
* FUNCTION     : set GDO0 for internal transmission mode.
* INPUT        : none
* OUTPUT       : none
****************************************************************/
void ELECHOUSE_CC1101::setGDO0_hostpinMode(int8_t direction)
{
	Serial.printf("\nGDO0 pin %d set to %s\n", GDO0, direction == INPUT? "INPUT":"OUTPUT");
    pinMode(GDO0, direction);
}

void ELECHOUSE_CC1101::setGDO2_hostpinMode(int8_t direction)
{
	Serial.printf("\nGDO2 pin %d set to %s\n", GDO2, direction == INPUT? "INPUT":"OUTPUT");
    pinMode(GDO2, direction);
}


/****************************************************************
* FUNCTION NAME:Reset
* FUNCTION     :CC1101 reset //details refer datasheet of CC1101/CC1100//
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::Reset(void)
{
    digitalWrite(SS_PIN, LOW);
    delay(1);

    digitalWrite(SS_PIN, HIGH);
    delay(1);

    digitalWrite(SS_PIN, LOW);

    MY_SPI.transfer(CC1101_SRES);

    digitalWrite(SS_PIN, HIGH);
    Serial.printf(FG_FYELLOW "%s: RESET !!!! \n", __FUNCTION__);
	memset(mirror, 0xFF, sizeof(mirror));
}


/****************************************************************
* FUNCTION NAME:Init
* FUNCTION     :CC1101 initialization
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::Init(void)
{
    setSpi();

    pinMode(SS_PIN, OUTPUT);
    pinMode(SCK_PIN, OUTPUT);
    pinMode(MOSI_PIN, OUTPUT);
    pinMode(MISO_PIN, INPUT);

    digitalWrite(SS_PIN, HIGH);
    digitalWrite(SCK_PIN, HIGH);
    digitalWrite(MOSI_PIN, LOW);
    

    MY_SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);

    //SpiStart();     //spi initialization

    Reset();                  //CC1101 reset

    RegConfigSettings();          //CC1101 register config

    SpiEnd();
}


/****************************************************************
* FUNCTION NAME:SpiWriteReg
* FUNCTION     :CC1101 write data to register
* INPUT        :addr: register address; value: register value
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::_SpiWriteReg(const char*name , byte addr, byte value, bool bQuiet)
{
    SpiStart();

    
	mirror[addr] = value;
    digitalWrite(SS_PIN, LOW);
    digitalWrite(SS_PIN, LOW);

    MY_SPI.transfer(addr);
    MY_SPI.transfer(value);
    digitalWrite(SS_PIN, HIGH);
    SpiEnd();
    
    if (!bQuiet) Serial.printf(FG_WHITE "\n%s [0x%02X] %s = 0x%02X\n" _DONE, __FUNCTION__, addr, name, value);
}


/****************************************************************
* FUNCTION NAME:SpiWriteBurstReg
* FUNCTION     :CC1101 write burst data to register
* INPUT        :addr: register address; buffer:register value array; num:number to write
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SpiWriteBurstReg(byte addr, byte *buffer, byte num)
{
    byte i, temp;

    SpiStart();
    temp = addr | WRITE_BURST;
    digitalWrite(SS_PIN, LOW);
    digitalWrite(SS_PIN, LOW);
    
    MY_SPI.transfer(temp);

    for (i = 0; i < num; i++)
        MY_SPI.transfer(buffer[i]);

    digitalWrite(SS_PIN, HIGH);
    digitalWrite(SS_PIN, HIGH);
    SpiEnd();
}


/****************************************************************
* FUNCTION NAME:SpiStrobe
* FUNCTION     :CC1101 Strobe
* INPUT        :strobe: command; //refer define in CC1101.h//
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SpiStrobe(byte strobe)
{
    SpiStart();
    digitalWrite(SS_PIN, LOW);
    digitalWrite(SS_PIN, LOW);

    MY_SPI.transfer(strobe);
    digitalWrite(SS_PIN, HIGH);
    digitalWrite(SS_PIN, HIGH);

    SpiEnd();
}


/****************************************************************
* FUNCTION NAME:SpiReadReg
* FUNCTION     :CC1101 read data from register
* INPUT        :addr: register address
* OUTPUT       :register value
****************************************************************/
byte ELECHOUSE_CC1101::SpiReadReg(byte addr)
{
    byte temp, value;

    SpiStart();
    temp = addr | READ_SINGLE;
    digitalWrite(SS_PIN, LOW);

    MY_SPI.transfer(temp);
    value = MY_SPI.transfer(0);
    digitalWrite(SS_PIN, HIGH);

    SpiEnd();
    return value;
}


/****************************************************************
* FUNCTION NAME:SpiReadBurstReg
* FUNCTION     :CC1101 read burst data from register
* INPUT        :addr: register address; buffer:array to store register value; num: number to read
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SpiReadBurstReg(byte addr, byte *buffer, byte num)
{
    byte i, temp;

    SpiStart();
    temp = addr | READ_BURST;
    digitalWrite(SS_PIN, LOW);
    digitalWrite(SS_PIN, LOW);

    MY_SPI.transfer(temp);

    for (i = 0; i < num; i++)
        buffer[i] = MY_SPI.transfer(0);

    digitalWrite(SS_PIN, HIGH);
    digitalWrite(SS_PIN, HIGH);
    
    SpiEnd();
}


/****************************************************************
* FUNCTION NAME:SpiReadStatus
* FUNCTION     :CC1101 read status register
* INPUT        :addr: register address
* OUTPUT       :status value
****************************************************************/
byte ELECHOUSE_CC1101::SpiReadStatus(byte addr)
{
    byte value, temp;

    SpiStart();
    temp = addr | READ_BURST;
    digitalWrite(SS_PIN, LOW);

    MY_SPI.transfer(temp);
    value = MY_SPI.transfer(0);

    digitalWrite(SS_PIN, HIGH);
    SpiEnd();
    return value;
}


/****************************************************************
* FUNCTION NAME:MY_SPI pin Settings
* FUNCTION     :Set Spi pins
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setSpi(void)
{
    if (spi == 0)
    {
#if defined __AVR_ATmega168__ || defined __AVR_ATmega328P__
        SCK_PIN = 13; MISO_PIN = 12; MOSI_PIN = 11; SS_PIN = 10;
#elif defined __AVR_ATmega1280__ || defined __AVR_ATmega2560__
        SCK_PIN = 52; MISO_PIN = 50; MOSI_PIN = 51; SS_PIN = 53;
#elif ESP8266
        SCK_PIN = 14; MISO_PIN = 12; MOSI_PIN = 13; SS_PIN = 15;
#elif defined ARDUINO_M5STACK_CORES3
        SCK_PIN = 36; MISO_PIN = 35; MOSI_PIN = 37; SS_PIN = 5;
#elif defined ARDUINO_M5STACK_CORE2
        SCK_PIN = 18; MISO_PIN = 38; MOSI_PIN = 23; SS_PIN = 27;
#elif ESP32
        SCK_PIN = 18; MISO_PIN = 19; MOSI_PIN = 23; SS_PIN = 5;
#else
        SCK_PIN = 13; MISO_PIN = 12; MOSI_PIN = 11; SS_PIN = 10;
#endif
    }
}


/****************************************************************
* FUNCTION NAME:CUSTOM MY_SPI
* FUNCTION     :set custom spi pins.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setSpiPin(byte sck, byte miso, byte mosi, byte ss)
{
    spi = 1;
    SCK_PIN = sck;
    MISO_PIN = miso;
    MOSI_PIN = mosi;
    SS_PIN = ss;
}


/****************************************************************
* FUNCTION NAME:GDO0 IRQ falling callback
****************************************************************/
void ELECHOUSE_CC1101::enableFallingIRQ_GDO0(bool bEnable)
{
	Serial.printf(FG_FYELLOW);
	
	if (bEnable)
	{
		bGDO0_HasFallingCallback = true;
	    if (irqDirGDO0 == FALLING || irqDirGDO0 == CHANGE )
	    {
			Serial.printf("%s no change\n", __FUNCTION__);
			Serial.printf(_DONE);
	    	return;
	    }

	    irqDnCtrGDO0 = 0;

	    if (irqDirGDO0 == RISING) 
	    {
	    	// rising in use.
	    	attachInterrupt(GDO0, onGDO0_IRQ, CHANGE);
			irqDirGDO0 = CHANGE;
			
			irqDnCtrGDO0 = irqUpCtrGDO0 = 0;
			Serial.printf("%s CHANGE mode\n", __FUNCTION__);
			Serial.printf(_DONE);
	    	return;
	    }

	   	attachInterrupt(GDO0, onGDO0_IRQ, FALLING);
	   	irqDirGDO0 = FALLING;
		Serial.printf("%s FALLING mode\n", __FUNCTION__);
	   	
	}
	else
	{
		bGDO0_HasFallingCallback = false;
		//disconnecting.
		if (irqDirGDO0 == FALLING )
		{
			detachInterrupt(GDO0);
			
			Serial.printf("%s DETACHED\n", __FUNCTION__);
			Serial.printf(_DONE);
			return;
		}
		attachInterrupt(GDO0, onGDO0_IRQ, RISING);
		Serial.printf("%s RISING mode\n", __FUNCTION__);
		irqDirGDO0 = RISING;
	}
	Serial.printf(_DONE);
}

/****************************************************************
* FUNCTION NAME:GDO0 IRQ rising callback
****************************************************************/
void ELECHOUSE_CC1101::enableRisingIRQ_GDO0(bool bEnable)
{
	Serial.printf(FG_FYELLOW);
	
	bGDO0_HasRisingCallback = bEnable;
	if (bEnable)
	{
	    if (irqDirGDO0 == RISING || irqDirGDO0 == CHANGE ) 
	    {
	    	Serial.printf("%s no change\n", __FUNCTION__);
			Serial.printf(_DONE);
	    	return;
	    }
		
	    irqUpCtrGDO0 = 0;

	    if (irqDirGDO0 == FALLING) 
	    {
	    	// rising in use.
	    	attachInterrupt(GDO0, onGDO0_IRQ, CHANGE);
			irqDnCtrGDO0 = irqUpCtrGDO0 = 0;
			irqDirGDO0 = CHANGE;
			Serial.printf("%s CHANGE mode\n", __FUNCTION__);
			Serial.printf(_DONE);
	    	return;
	    }

	   	attachInterrupt(GDO0, onGDO0_IRQ, RISING);
	   	irqDirGDO0 = RISING;
		Serial.printf("%s RISING mode\n", __FUNCTION__);
	}
	else
	{
		//disconnecting.
		if (irqDirGDO0 == RISING )
		{
	    	Serial.printf("%s DETACHING\n", __FUNCTION__);
			detachInterrupt(GDO0);
			Serial.printf(_DONE);
			return;
		}
		attachInterrupt(GDO0, onGDO0_IRQ, FALLING);
		irqDirGDO0 = FALLING;
		Serial.printf("%s no change\n", __FUNCTION__);
	}
	
	Serial.printf(_DONE);
}


/****************************************************************
* FUNCTION NAME:GDO2 IRQ falling callback
****************************************************************/
void ELECHOUSE_CC1101::enableFallingIRQ_GDO2(bool bEnable)
{
	Serial.printf(FG_FYELLOW);
	
	if (bEnable)
	{
		bGDO2_HasFallingCallback = true;
	    if (irqDirGDO2 == FALLING || irqDirGDO2 == CHANGE )
	    {
			Serial.printf("%s no change\n", __FUNCTION__);
			Serial.printf(_DONE);
	    	return;
	    }

	    irqDnCtrGDO2 = 0;

	    if (irqDirGDO2 == RISING) 
	    {
	    	// rising in use.
	    	attachInterrupt(GDO2, onGDO2_IRQ, CHANGE);
			irqDirGDO2 = CHANGE;
			
			irqDnCtrGDO2 = irqUpCtrGDO2 = 0;
			Serial.printf("%s CHANGE mode\n", __FUNCTION__);
			Serial.printf(_DONE);
	    	return;
	    }

	   	attachInterrupt(GDO2, onGDO2_IRQ, FALLING);
	   	irqDirGDO2 = FALLING;
		Serial.printf("%s FALLING mode\n", __FUNCTION__);
	   	
	}
	else
	{
		bGDO2_HasFallingCallback = false;
		//disconnecting.
		if (irqDirGDO2 == FALLING )
		{
			detachInterrupt(GDO2);
			
			Serial.printf("%s DETACHED\n", __FUNCTION__);
			Serial.printf(_DONE);
			return;
		}
		attachInterrupt(GDO2, onGDO2_IRQ, RISING);
		Serial.printf("%s RISING mode\n", __FUNCTION__);
		irqDirGDO2 = RISING;
	}
	Serial.printf(_DONE);
}

/****************************************************************
* FUNCTION NAME:GDO2 IRQ rising callback
****************************************************************/
void ELECHOUSE_CC1101::enableRisingIRQ_GDO2(bool bEnable)
{
	Serial.printf(FG_FYELLOW);
	
	bGDO2_HasRisingCallback = bEnable;
	if (bEnable)
	{
	    if (irqDirGDO2 == RISING || irqDirGDO2 == CHANGE ) 
	    {
	    	Serial.printf("%s no change\n", __FUNCTION__);
			Serial.printf(_DONE);
	    	return;
	    }
		
	    irqUpCtrGDO2 = 0;

	    if (irqDirGDO2 == FALLING) 
	    {
	    	// rising in use.
	    	attachInterrupt(GDO2, onGDO2_IRQ, CHANGE);
			irqDnCtrGDO2 = irqUpCtrGDO2 = 0;
			irqDirGDO2 = CHANGE;
			Serial.printf("%s CHANGE mode\n", __FUNCTION__);
			Serial.printf(_DONE);
	    	return;
	    }

	   	attachInterrupt(GDO2, onGDO2_IRQ, RISING);
	   	irqDirGDO2 = RISING;
		Serial.printf("%s RISING mode\n", __FUNCTION__);
	}
	else
	{
		//disconnecting.
		if (irqDirGDO2 == RISING )
		{
	    	Serial.printf("%s DETACHING\n", __FUNCTION__);
			detachInterrupt(GDO2);
			Serial.printf(_DONE);
			return;
		}
		attachInterrupt(GDO2, onGDO2_IRQ, FALLING);
		irqDirGDO2 = FALLING;
		Serial.printf("%s no change\n", __FUNCTION__);
	}
	
	Serial.printf(_DONE);
}


bool ELECHOUSE_CC1101::wait4RisingIRQ_GDO0(void)
{
	int ret1 = xSemaphoreTake( sem_GDO0_UP, pdMS_TO_TICKS(3000));
	return (ret1 == pdTRUE) ? true : false;
}

bool ELECHOUSE_CC1101::wait4FallingIRQ_GDO0(void)
{
	int ret1 = xSemaphoreTake( sem_GDO0_DN, pdMS_TO_TICKS(3000));
	return (ret1 == pdTRUE) ? true : false;
}

bool ELECHOUSE_CC1101::wait4RisingIRQ_GDO2(void)
{
	int ret1 = xSemaphoreTake( sem_GDO2_UP, pdMS_TO_TICKS(3000));
	return (ret1 == pdTRUE) ? true : false;
}

bool ELECHOUSE_CC1101::wait4FallingIRQ_GDO2(void)
{
	int ret1 = xSemaphoreTake( sem_GDO2_DN, pdMS_TO_TICKS(3000));
	return (ret1 == pdTRUE) ? true : false;
}




/****************************************************************
* FUNCTION NAME:GDO Pin settings
* FUNCTION     :set GDO Pins
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setGDOx(byte gdo0, byte gdo2)
{
    GDO0 = gdo0;
    GDO2 = gdo2;
    GDOx_SetPinMode();
}


/****************************************************************
* FUNCTION NAME:GDO0 Pin setting
* FUNCTION     :set GDO0 Pin
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::defineGDO0_pinNum(byte gdo0)
{
    GDO0 = gdo0;
    setGDO0_hostpinMode(INPUT);
}

//------------------
typedef struct { 
	int opcode;
	char *msg;
}PIN_DEF;

PIN_DEF pin_defs[] =
{
  { 0 ,"Associated to the RX FIFO\n\tAsserts when RX FIFO is filled at or above the RX FIFO threshold\n\tDe-asserts when RX FIFO is drained below the same"},
  { 1 ,"Associated to the RX FIFO\n\tAsserts when RX FIFO is filled at or above the RX FIFO threshold or the end of packet is reached\n\tDe-asserts when t"},
  { 2 ,"Associated to the TX FIFO\n\tAsserts when the TX FIFO is filled at or above the TX FIFO threshold\n\tDe-asserts when the TX FIFO is below the same"},
  { 3 ,"Associated to the TX FIFO\n\tAsserts when TX FIFO is full\n\tDe-asserts when the TX FIFO is drained below the TX FIFO threshold."},
  { 4 ,"Asserts when the RX FIFO has overflowed\n\tDe-asserts when the FIFO has been flushed."},
  { 5 ,"Asserts when the TX FIFO has underflowed\n\tDe-asserts when the FIFO is flushed."},

  { 6 ,"Asserts when sync word has been sent/received\n"
  		"\tDe-asserts at the end of the packet.\n\tIn RX, the pin will also de-assert when a packet is discarded due to\n"
  		"\ta bad address\n\tmaximum length filtering\n"
  		"\tthe radio enters RXFIFO_OVERFLOW state.\n\n"
  		"\tIn TX the pin will de-assert if\n"
  		"\tTX FIFO underflows."},

  { 7 ,"Asserts when a packet has been received with CRC OK\n\tDe-asserts when the first byte is read from the RX FIFO."},
  { 8 ,"Preamble Quality Reached\n\tAsserts when the PQI is above the programmed PQT value\n\tDe-asserted when the chip re- enters RX state (MARCSTATE=0x0"},
  { 9 ,"Clear channel assessment\n\tHigh when RSSI level is below threshold (dependent on the current CCA_MODE setting)."},
  {10 ,"Lock detector output\n\tThe PLL is in lock if the lock detector output has a positive transition or is constantly logic high\n\tTo check for PLL"},
  {11 ,"Serial Clock\n\tSynchronous to the data in synchronous serial mode.\n\tIn RX mode, data is set up on the falling edge by CC1101 when GDOx_INV=0."},
  {12 ,"Serial Synchronous Data Output\n\tUsed for synchronous serial mode."},
  {13 ,"Serial Data Output\n\tUsed for asynchronous serial mode."},
  {14 ,"Carrier sense\n\tHigh if RSSI level is above threshold\n\tCleared when entering IDLE mode."},
  {15 ,"CRC_OK\n\tThe last CRC comparison matched\n\tCleared when entering/restarting RX mode."},
  {22 ,"RX_HARD_DATA[1]\n\tCan be used together with RX_SYMBOL_TICK for alternative serial RX output."},
  {23 ,"RX_HARD_DATA[0]\n\tCan be used together with RX_SYMBOL_TICK for alternative serial RX output."},
  {27 ,"PA_PD\n\t Control an external PA or RX/TX switch (see pdf)"},
  {28 ,"LNA_PD\n\tControl an external LNA or RX/TX switch (see pdf)"},
  {29 ,"RX_SYMBOL_TICK\n\tCan be used together with RX_HARD_DATA for alternative serial RX output."},
};



//------------------
void ELECHOUSE_CC1101::setGDOxPinConfig(uint8_t pinRegNum, uint8_t value, bool bSilent)
{
	int i;
	uint8_t end = sizeof(pin_defs)/sizeof(pin_defs[0]);

	assert(CC1101_IOCFG2 == pinRegNum || CC1101_IOCFG0 == pinRegNum);

	for (i = 0; i < end; i++)
	{
		if (pin_defs[i].opcode != value) continue;
		if (!bSilent) Serial.printf(FG_BCYAN "\n%s [0x%02X] %s\n" _DONE, pinRegNum == CC1101_IOCFG2 ? "GDO2":"GDO0", value, pin_defs[i].msg);
		break;
	}
	
	if (i == end) Serial.printf(FG_BCYAN "\n%s ERROR ?? [0x%02X] %s\n" _DONE, pinRegNum , value, "see documentation"); 
	
	_SpiWriteReg("CC1101_IOCFGx", pinRegNum, value, 1);  //silent please.

	
}
/****************************************************************
* FUNCTION NAME:CCMode
* FUNCTION     :Format of RX and TX data
* INPUT        :none
* OUTPUT       :none
****************************************************************/

void callme(void)
{
}

void ELECHOUSE_CC1101::setCCMode(eGDIO_MODES s)
{
    ccmode = s;

    if (ccmode == LEGACY_1)
    {
    	Serial.printf(FG_RED "%s: ccmode = LEGACY1 ---------------\n" _DONE, __FUNCTION__);

		setGDO0_hostpinMode(INPUT);
		setGDO2_hostpinMode(INPUT);

        setGDOxPinConfig(CC1101_IOCFG0, 0x06); // + sync sent .... - packet send
        setGDOxPinConfig(CC1101_IOCFG2, 0x0B); // serial data OUT on GDO2

        //SpiWriteReg(CC1101_PKTCTRL0, 0x05);
        setPktFormat(0);
        setLengthConfig(1);

        setDataRateKhz(0.097);
    }
    else if (ccmode == LEGACY_0)
    {
		setGDO0_hostpinMode(INPUT);
		setGDO2_hostpinMode(INPUT);
		
    	Serial.printf(FG_RED "%s: ccmode = LEGACY0 ---------------\n" _DONE, __FUNCTION__);
        setGDOxPinConfig(CC1101_IOCFG2, 0x0D); 	// serial data out
        setGDOxPinConfig(CC1101_IOCFG0, 0x0D);	// serial data out
        
        //SpiWriteReg(CC1101_PKTCTRL0, 0x32);
        setPktFormat(3);
        setLengthConfig(2);		// infinite

		setDataRateKhz(4.800);
    }
    else if (ccmode == SYMBOL_TICK)
    {
    	Serial.printf(FG_RED "%s: ccmode = SYMBOL_TICK ---------------\n" _DONE, __FUNCTION__);

		setGDO0_hostpinMode(INPUT);
		setGDO2_hostpinMode(INPUT);
	
        setGDOxPinConfig(CC1101_IOCFG2, 0x1D); // SYMBOL TICK
        setGDOxPinConfig(CC1101_IOCFG0, 0x0D);
        
        setPktFormat(3);		//data in on GDO0 data out on GDOx
        setLengthConfig(2);  	// infinite

		setDataRateKhz(4.800);
		enableRisingIRQ_GDO2(callme);
	}
	else
		assert(ccmode != ccmode);
		
  

    setModulation(modulation);
}


/****************************************************************
* FUNCTION NAME:Modulation
* FUNCTION     :set CC1101 Modulation
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setModulation(byte m)
{
    if (m > 4)
        m = 4;

#if OEM_CODE

    modulation = m;
    Split_MDMCFG2();

    switch (m)
    {
    case 0: m2MODFM = 0x00; frend0 = 0x10; break;   // 2-FSK

    case 1: m2MODFM = 0x10; frend0 = 0x10; break;   // GFSK

    case 2: m2MODFM = 0x30; frend0 = 0x11; break;   // ASK

    case 3: m2MODFM = 0x40; frend0 = 0x10; break;   // 4-FSK

    case 4: m2MODFM = 0x70; frend0 = 0x10; break;   // MSK
    }

    SpiWriteReg(CC1101_MDMCFG2, m2DCOFF + m2MODFM + m2MANCH + m2SYNCM);
    SpiWriteReg(CC1101_FREND0, frend0);
#else
	uint8_t modulation;

	// common across all selections.
	Serial.printf(FG_MAGENTA "%s: set PA lo current\n" _DONE, __FUNCTION__);
	regRMW(CC1101_FREND0, 1, 5, 4);

	char type[20];
	
	switch (m)
	{
		case 0:
			strcpy(type, "2-FSK");
			modulation = 0;
			break;	// 2-FSK

		case 1: 
			strcpy(type,"GFSK");
			modulation = 1; 
			break;	// GFSK

		case 2: 
			strcpy(type, "OOK");
			modulation = 3; 

			//Serial.printf(FG_FRED "\n%s: todo ook p/a levels?\n" _DONE, __FUNCTION__);
			Serial.printf(FG_MAGENTA "%s: PA power table index = %d\n" _DONE, __FUNCTION__, 1);
			regRMW(CC1101_FREND0, 1, 2, 0);
			break;	// OOK

		case 3: 
			strcpy(type, "4-FSK");
			modulation = 4;
			break;	// 4-FSK

		case 4: 
			strcpy(type, "MSK");
			modulation = 7; 
			break;	// MSK
	}

	Serial.printf(FG_MAGENTA "%s: modulation %s 0x%X\n" _DONE, __FUNCTION__, type, modulation);
	regRMW(CC1101_MDMCFG2, modulation, 6, 4);


    setPA(pa);

#endif
}


/****************************************************************
* FUNCTION NAME:PA Power
* FUNCTION     :set CC1101 PA Power
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPA(int p)
{
    int a;

    pa = p;

    if (gMHz >= 300 && gMHz <= 348)
    {
        if (pa <= -30)
            a = PA_TABLE_315[0];
        else if (pa > -30 && pa <= -20)
            a = PA_TABLE_315[1];
        else if (pa > -20 && pa <= -15)
            a = PA_TABLE_315[2];
        else if (pa > -15 && pa <= -10)
            a = PA_TABLE_315[3];
        else if (pa > -10 && pa <= 0)
            a = PA_TABLE_315[4];
        else if (pa > 0 && pa <= 5)
            a = PA_TABLE_315[5];
        else if (pa > 5 && pa <= 7)
            a = PA_TABLE_315[6];
        else if (pa > 7)
            a = PA_TABLE_315[7];

        last_pa = 1;
    }
    else if (gMHz >= 378 && gMHz <= 464)
    {
        if (pa <= -30)
            a = PA_TABLE_433[0];
        else if (pa > -30 && pa <= -20)
            a = PA_TABLE_433[1];
        else if (pa > -20 && pa <= -15)
            a = PA_TABLE_433[2];
        else if (pa > -15 && pa <= -10)
            a = PA_TABLE_433[3];
        else if (pa > -10 && pa <= 0)
            a = PA_TABLE_433[4];
        else if (pa > 0 && pa <= 5)
            a = PA_TABLE_433[5];
        else if (pa > 5 && pa <= 7)
            a = PA_TABLE_433[6];
        else if (pa > 7)
            a = PA_TABLE_433[7];

        last_pa = 2;
    }
    else if (gMHz >= 779 && gMHz <= 899.99)
    {
        if (pa <= -30)
            a = PA_TABLE_868[0];
        else if (pa > -30 && pa <= -20)
            a = PA_TABLE_868[1];
        else if (pa > -20 && pa <= -15)
            a = PA_TABLE_868[2];
        else if (pa > -15 && pa <= -10)
            a = PA_TABLE_868[3];
        else if (pa > -10 && pa <= -6)
            a = PA_TABLE_868[4];
        else if (pa > -6 && pa <= 0)
            a = PA_TABLE_868[5];
        else if (pa > 0 && pa <= 5)
            a = PA_TABLE_868[6];
        else if (pa > 5 && pa <= 7)
            a = PA_TABLE_868[7];
        else if (pa > 7 && pa <= 10)
            a = PA_TABLE_868[8];
        else if (pa > 10)
            a = PA_TABLE_868[9];

        last_pa = 3;
    }
    else if (gMHz >= 900 && gMHz <= 928)
    {
        if (pa <= -30)
            a = PA_TABLE_915[0];
        else if (pa > -30 && pa <= -20)
            a = PA_TABLE_915[1];
        else if (pa > -20 && pa <= -15)
            a = PA_TABLE_915[2];
        else if (pa > -15 && pa <= -10)
            a = PA_TABLE_915[3];
        else if (pa > -10 && pa <= -6)
            a = PA_TABLE_915[4];
        else if (pa > -6 && pa <= 0)
            a = PA_TABLE_915[5];
        else if (pa > 0 && pa <= 5)
            a = PA_TABLE_915[6];
        else if (pa > 5 && pa <= 7)
            a = PA_TABLE_915[7];
        else if (pa > 7 && pa <= 10)
            a = PA_TABLE_915[8];
        else if (pa > 10)
            a = PA_TABLE_915[9];

        last_pa = 4;
    }

    if (modulation == 2)
    {
        PA_TABLE[0] = 0;
        PA_TABLE[1] = a;
    }
    else
    {
        PA_TABLE[0] = a;
        PA_TABLE[1] = 0;
    }

    SpiWriteBurstReg(CC1101_PATABLE, PA_TABLE, 8);
}

/****************************************************************
* FUNCTION NAME:setOSCdrift
* INPUT        : target miss on freq adj
****************************************************************/
float  ELECHOUSE_CC1101::setOSCdrift(float hz)
{
	float ret = tweakFreqHz;
	tweakFreqHz = hz;
	setMHZ(getMHZ());  	// reload frequency.
	
	return ret;
}

/****************************************************************
* FUNCTION NAME:Frequency Calculator
* FUNCTION     :Calculate the basic frequency.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
float ELECHOUSE_CC1101::getMHZ(void)
{
	return gMHz;
}

void ELECHOUSE_CC1101::setMHZ(float mhz)
{

   	uint32_t  temp;

	if (mhz == 0.0 ) mhz = gMHz;
	

	float adjFreq = mhz + tweakFreqHz/1e6;
	
	temp = (( adjFreq  * (float)(1 << 16))/ XTAL_Mhz);

 	Serial.printf(FG_CYAN "\n%s: tgt=%7.3f -> %f  (delta = %7.3f)\n"  _DONE, 
 			__FUNCTION__, mhz, adjFreq, tweakFreqHz);
	
	SpiWriteReg(CC1101_FREQ2, (temp >>16) & 0xFF);
	SpiWriteReg(CC1101_FREQ1, (temp >> 8) & 0xFF);
	SpiWriteReg(CC1101_FREQ0,  temp       & 0xFF);
	
	gMHz= mhz;

    Calibrate();

#if 0
	// verify.
	uint32_t tweaked = SpiReadReg(CC1101_FREQ2) << 16 | SpiReadReg(CC1101_FREQ1)  << 8 | SpiReadReg(CC1101_FREQ0);

               
	double retest;
	retest = (XTAL_Mhz / (double)(1<<16)) * (double) tweaked;
	Serial.printf("%s VERIFY = %f mhz \n", __FUNCTION__, (float) retest);

	double err = adjFreq - retest;

	Serial.printf("%s error = %d hz\n\n", __FUNCTION__, (int) err);
#endif
}


/****************************************************************
* FUNCTION NAME:Calibrate
* FUNCTION     :Calibrate frequency
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::Calibrate(void)
{

	//CC1101_FSCTRL0 = add offset to any setMHZ command BY HARDWARE!
	//CC1101_TEST0 = no clue. Too obtuse.

	const int32_t hzPerStep = (XTAL_Mhz * 1e6)/(float) (1<<14);
	Serial.printf(FG_GREEN "%s hz/step = %d\n" _DONE, __FUNCTION__, hzPerStep); 
	
	
    if (gMHz >= 300 && gMHz <= 348)
    {
    	
        int32_t offset =(CC1101_FSCTRL0, map(gMHz, 300, 348, hwTweakHz_300_348Mhz[0], hwTweakHz_300_348Mhz[1]));
		Serial.printf(FG_GREEN "%s 300->348 a %d hz internal HW offset to %f -> %f \n" _DONE, __FUNCTION__, offset, gMHz, gMHz+ (float) offset/1000000. ); 
        
        SpiWriteReg(CC1101_FSCTRL0, offset / hzPerStep);

        if (gMHz < 322.88)
        {
            SpiWriteReg(CC1101_TEST0, 0x0B);
        }
        else
        {
            SpiWriteReg(CC1101_TEST0, 0x09);
            int s = ELECHOUSE_cc1101.SpiReadStatus(CC1101_FSCAL2);

            if (s < 32)
                SpiWriteReg(CC1101_FSCAL2, s + 32);

            if (last_pa != 1)
                setPA(pa);
        }
    }
    else if (gMHz >= 378 && gMHz <= 464)
    {
        int32_t offset =(CC1101_FSCTRL0, map(gMHz, 378, 464, hwTweakHz_378_464Mhz[0], hwTweakHz_378_464Mhz[1]));
		Serial.printf(FG_GREEN "%s 378->464 a %d hz internal HW offset to %f -> %f \n" _DONE, __FUNCTION__, offset, gMHz, gMHz+ (float) offset/1000000. ); 
        
        SpiWriteReg(CC1101_FSCTRL0, offset / hzPerStep);

        if (gMHz < 430.5)
        {
            SpiWriteReg(CC1101_TEST0, 0x0B);
        }
        else
        {
            SpiWriteReg(CC1101_TEST0, 0x09);
            int s = ELECHOUSE_cc1101.SpiReadStatus(CC1101_FSCAL2);

            if (s < 32)
                SpiWriteReg(CC1101_FSCAL2, s + 32);

            if (last_pa != 2)
                setPA(pa);
        }
    }
    else if (gMHz >= 779 && gMHz <= 899.99)
    {
    
		int32_t offset =(CC1101_FSCTRL0, map(gMHz, 779, 899, hwTweakHz_779_899Mhz[0], hwTweakHz_779_899Mhz[1]));
		Serial.printf(FG_GREEN "%s 779->899 a %d hz internal HW offset to %f -> %f \n" _DONE, __FUNCTION__, offset, gMHz, gMHz+ (float) offset/1000000. ); 
		
		SpiWriteReg(CC1101_FSCTRL0, offset / hzPerStep);
	
        if (gMHz < 861)
        {
            SpiWriteReg(CC1101_TEST0, 0x0B);
        }
        else
        {
            SpiWriteReg(CC1101_TEST0, 0x09);
            int s = ELECHOUSE_cc1101.SpiReadStatus(CC1101_FSCAL2);

            if (s < 32)
                SpiWriteReg(CC1101_FSCAL2, s + 32);

            if (last_pa != 3)
                setPA(pa);
        }
    }
    else if (gMHz >= 900 && gMHz <= 928)
    {

		int32_t offset =(CC1101_FSCTRL0, map(gMHz, 900, 928, hwTweakHz_900_928Mhz[0], hwTweakHz_900_928Mhz[1]));
		Serial.printf(FG_GREEN "%s 900->928 a %d hz internal HW offset to %f -> %f \n" _DONE, __FUNCTION__, offset, gMHz, gMHz+ (float) offset/1000000. ); 

		Serial.printf("note: %d %d\n", offset / hzPerStep,  (uint8_t)( offset / hzPerStep));
		SpiWriteReg(CC1101_FSCTRL0, (uint8_t)(offset / hzPerStep));
		
        SpiWriteReg(CC1101_TEST0, 0x09);
        int s = ELECHOUSE_cc1101.SpiReadStatus(CC1101_FSCAL2);

        if (s < 32)
            SpiWriteReg(CC1101_FSCAL2, s + 32);

        if (last_pa != 4)
            setPA(pa);
    }
}


/****************************************************************
* FUNCTION NAME:Calibration offset
* FUNCTION     :Set calibration offset
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setCalibrationOffset(byte b, int32_t low, int32_t  high)
{
    if (b == 1)
    {
        hwTweakHz_300_348Mhz[0] = low;
        hwTweakHz_300_348Mhz[1] = high;
    }
    else if (b == 2)
    {
        hwTweakHz_378_464Mhz[0] = low;
        hwTweakHz_378_464Mhz[1] = high;
    }
    else if (b == 3)
    {
        hwTweakHz_779_899Mhz[0] = low;
        hwTweakHz_779_899Mhz[1] = high;
    }
    else if (b == 4)
    {
        hwTweakHz_900_928Mhz[0] = low;
        hwTweakHz_900_928Mhz[1] = high;
    }
}


/****************************************************************
* FUNCTION NAME:getCC1101
* FUNCTION     :Test Spi connection and return 1 when true.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
bool ELECHOUSE_CC1101::getCC1101(void)
{
    setSpi();
	
    uint8_t foo = SpiReadStatus(0x31);
    Serial.printf("h/w version %d\n", foo); 
    delay(2000);
    
    if (foo > 0)
        return 1;
    else
        return 0;
}
/****************************************************************
* FUNCTION NAME:getMode
* FUNCTION     :Return the Mode. Sidle = 0, TX = 1, Rx = 2.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
eMODEM_STATE ELECHOUSE_CC1101::getMode(void)
{
    return trxstate;
}

/****************************************************************
* FUNCTION NAME:Set Sync_Word
* FUNCTION     :Sync Word
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setSyncWord(byte sh, byte sl)
{
	Serial.printf(FG_MAGENTA "\n%s: set sync word 0x%8X\n" _DONE, __FUNCTION__, (sh << 8) + sl);
    SpiWriteReg(CC1101_SYNC1, sh);
    SpiWriteReg(CC1101_SYNC0, sl);
}


/****************************************************************
* FUNCTION NAME:Set ADDR
* FUNCTION     :Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setAddr(byte v)
{
	Serial.printf(FG_MAGENTA "\n%s: set network addr = %d\n" _DONE, __FUNCTION__, v);
    SpiWriteReg(CC1101_ADDR, v);
}


/****************************************************************
* FUNCTION NAME:Set PQT
* FUNCTION     :Preamble quality estimator threshold
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPQT(byte v)
{
#if OEM_CODE

    Split_PKTCTRL1();
    pc1PQT = 0;

    if (v > 7)
        v = 7;

    pc1PQT = v * 32;
    SpiWriteReg(CC1101_PKTCTRL1, pc1PQT + pc1CRC_AF + pc1APP_ST + pc1ADRCHK);
#else
	Serial.printf(FG_MAGENTA "\n%s: setting preamble quality = %d\n" _DONE, __FUNCTION__, v);
	regRMW(CC1101_PKTCTRL1,v, 7, 5);
#endif

}


/****************************************************************
* FUNCTION NAME:Set CRC_AUTOFLUSH
* FUNCTION     :Enable automatic flush of RX FIFO when CRC is not OK
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setCRC_AF(bool v)
{
#if OEM_CODE
    Split_PKTCTRL1();
    pc1CRC_AF = 0;

    if (v == 1)
        pc1CRC_AF = 8;

    SpiWriteReg(CC1101_PKTCTRL1, pc1PQT + pc1CRC_AF + pc1APP_ST + pc1ADRCHK);
#else
	Serial.printf(FG_MAGENTA "\n%s: auto flush is %s\n" _DONE, __FUNCTION__, v ? "ENABLED":"DISABLED");
	regRMW(CC1101_PKTCTRL1,v, 3, 3);
#endif
}


/****************************************************************
* FUNCTION NAME:Set APPEND_STATUS
* FUNCTION     :When enabled, two status bytes will be appended to the payload of the packet
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setAppendStatus(bool v)
{

#if OEM_CODE
    Split_PKTCTRL1();
    pc1APP_ST = 0;

    if (v == 1)
        pc1APP_ST = 4;

    SpiWriteReg(CC1101_PKTCTRL1, pc1PQT + pc1CRC_AF + pc1APP_ST + pc1ADRCHK);
#else
	Serial.printf(FG_MAGENTA "\n%s: %s\n" _DONE, __FUNCTION__, v ? "ON":"OFF");
    regRMW(CC1101_PKTCTRL1, v, 2, 2);
#endif
}


/****************************************************************
* FUNCTION NAME:Set ADR_CHK
* FUNCTION     :Controls address check configuration of received packages
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setAdrChk(byte v)
{
#if OEM_CODE
    Split_PKTCTRL1();
    pc1ADRCHK = 0;

    if (v > 3)
        v = 3;

    pc1ADRCHK = v;
    SpiWriteReg(CC1101_PKTCTRL1, pc1PQT + pc1CRC_AF + pc1APP_ST + pc1ADRCHK);
#else
	const char *msg[] = 
	{
		"(00)No address check",
		"(01)Address check, no broadcast",
		"(10)Address check and 0 (0x00) broadcast",
		"(11)Address check and 0 (0x00) and 255 (0xFF)"
	};
	
   if (v > 3) v = 3;

   Serial.printf(FG_BMAGENTA "\n%s %s\n" _DONE, __FUNCTION__, msg[v]);
   
   regRMW(CC1101_PKTCTRL1, v, 1, 0);
#endif

}


/****************************************************************
* FUNCTION NAME:Set WHITE_DATA
* FUNCTION     :Turn data whitening on / off.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setWhiteData(bool v)
{
#if OEM_CODE
    Split_PKTCTRL0();
    pc0WDATA = 0;

    if (v == 1)
        pc0WDATA = 64;

    SpiWriteReg(CC1101_PKTCTRL0, pc0WDATA + pc0PktForm + pc0CRC_EN + pc0LenConf);
#else
	regRMW(CC1101_PKTCTRL0, v, 6,6);
#endif
}


/****************************************************************
* FUNCTION NAME:Set PKT_FORMAT
* FUNCTION     :Format of RX and TX data
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPktFormat(byte v)
{

#if OEM_CODE
    Split_PKTCTRL0();
    pc0PktForm = 0;

    if (v > 3)
        v = 3;

    pc0PktForm = v * 16;
    SpiWriteReg(CC1101_PKTCTRL0, pc0WDATA + pc0PktForm + pc0CRC_EN + pc0LenConf);
#else
   if (v > 3) v = 3;

	Serial.println(FG_BMAGENTA);
    switch(v)
    {
    	case 0:
    		Serial.printf("%s: (00)Normal mode, use FIFOs for RX and TX\n", __FUNCTION__);
    	break;

    	case 1:
    		Serial.printf("%s: (01)Synchronous serial mode, Data in on GDO0 and "
						  "data out on either of the GDOx pins", __FUNCTION__);
		break;

		case 2:
			Serial.printf("%s: (02)random TX mode; sends random data using PN9\n", __FUNCTION__);
		break;

		case 3:
			Serial.printf("%s: (03)Asynchronous serial mode\n\tdata in on GDO0 and "
						  "data out on either of the GDOx pins\n", __FUNCTION__);
		break;

		default:
			assert (pc0PktForm =! pc0PktForm);
		break;
	}
	Serial.print(_DONE);
	
	regRMW(CC1101_PKTCTRL0, v , 5, 4);
#endif
}


/****************************************************************
* FUNCTION NAME:Set CRC
* FUNCTION     :CRC calculation in TX and CRC check in RX
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setCrc(bool v)
{
#if OEM_CODE
    Split_PKTCTRL0();
    pc0CRC_EN = 0;

    if (v == 1)
        pc0CRC_EN = 4;

    SpiWriteReg(CC1101_PKTCTRL0, pc0WDATA + pc0PktForm + pc0CRC_EN + pc0LenConf);
#else
	Serial.printf(FG_MAGENTA "\n%s is %s\n" _DONE, __FUNCTION__, v ? "ENABLED" : "DISABLED");
	regRMW(CC1101_PKTCTRL0, v , 2, 2);
#endif
}


/****************************************************************
* FUNCTION NAME:Set LENGTH_CONFIG
* FUNCTION     :Configure the packet length
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setLengthConfig(byte v)
{

#if OEM_CODE
    Split_PKTCTRL0();
    pc0LenConf = 0;

    if (v > 3)
        v = 3;

    pc0LenConf = v;
    SpiWriteReg(CC1101_PKTCTRL0, pc0WDATA + pc0PktForm + pc0CRC_EN + pc0LenConf);
#else
	Serial.println(FG_BMAGENTA);
	
      switch(v)
    {
    	case 0:
    		Serial.printf("%s: (00)Fixed packet length mode.\n\tLength configured in PKTLEN register\n", __FUNCTION__);
    	break;

    	case 1:
    		Serial.printf("%s: (01)Variable packet length mode.\n\tPacket length configured by the first byte after sync word\n", __FUNCTION__);
		break;

		case 2:
			Serial.printf("%s: (02)Infinite packet length mode\n", __FUNCTION__);
		break;

		case 3:
			Serial.printf("%s: (03)Reserved\n", __FUNCTION__);
		break;

		default:
			assert (pc0LenConf =! pc0LenConf);
		break;
	}
	
	Serial.print(_DONE);
	
	if (v > 3) v = 3;
	regRMW(CC1101_PKTCTRL0, v, 1, 0);

#endif
}


/****************************************************************
* FUNCTION NAME:Set PACKET_LENGTH
* FUNCTION     :Indicates the packet length
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPacketLength(byte v)
{
    Serial.printf(FG_BMAGENTA "\n%s: packet length = %d\n", __FUNCTION__, v);

    Serial.printf("\tWhen ***FIXED*** packet lengths ARE ENABLED.\n\n"
    			  "\tIf variable packet length mode is used,\n"
    			  "\tthis value indicates the maximum packet length allowed.\n"
    			  "\tThis value must be different from 0.\n\n"
    			  "\tIf INFINITE packet length is enabled this may be 0\n" _DONE);

    SpiWriteReg(CC1101_PKTLEN, v);
}


/****************************************************************
* FUNCTION NAME:Set fifo trigger level
* FUNCTION     :bytes before tx underflow or rx overflow 
* INPUT        :none
* OUTPUT       :none
		  TX   RX
0 (0000)  61    4
1 (0001)  57    8
2 (0010)  53   12
3 (0011)  49   16
4 (0100)  45   20
5 (0101)  41   24
6 (0110)  37   28
8 (1000)  29   36
9 (1001)  25   40
10 (1010) 21   44
11 (1011) 17   48
12 (1100) 13   52
13 (1101) 9    56
14 (1110) 5    60
15 (1111) 1    64
*/
const uint8_t tx_lvl[] = {61,57,53,49,45,41,37,33,29,25,21,17,13, 9, 5, 1};

const uint8_t rx_lvl[] = { 4, 8,12,16,20,24,28,32,36,40,44,48,52,56,60,64};


/****************************************************************/
void ELECHOUSE_CC1101::setTxFifoThreshold(uint8_t v)
{
	int i;
	int test;
	for (i = 0; i < sizeof(tx_lvl); i++) 
	{
		test = v - tx_lvl[i];
		//Serial.printf("%d  %d > %d x %d\n", i, v, tx_lvl[i], test);
		if ( v > tx_lvl[i] ) break;
	}
	i = i - 1;
	
	Serial.printf(FG_MAGENTA "%s : fifo warn RX @ %d or TX @ %d\n" _DONE, __FUNCTION__, rx_lvl[i], tx_lvl[i]);

    SpiWriteReg(CC1101_FIFOTHR, i);

    Serial.printf(FG_FRED " need to set GD0x if used\n" _DONE);
    
    //SpiWriteReg(CC1101_IOCFG0, 2);  // GD00 signal on tx low
}

/****************************************************************
* FUNCTION NAME:Set DCFILT_OFF
* FUNCTION     :Disable digital DC blocking filter before demodulator
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setDcFilterOff(bool v)
{
#if OEM_CODE
    Split_MDMCFG2();
    m2DCOFF = 0;

    if (v == 1)
        m2DCOFF = 128;

    SpiWriteReg(CC1101_MDMCFG2, m2DCOFF + m2MODFM + m2MANCH + m2SYNCM);
#else
    regRMW(CC1101_MDMCFG2, v, 7, 7);
#endif
}


/****************************************************************
* FUNCTION NAME:Set MANCHESTER
* FUNCTION     :Enables Manchester encoding/decoding
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setManchester(bool v)
{
#if OEM_CODE
    Split_MDMCFG2();
    m2MANCH = 0;

    if (v == 1)
        m2MANCH = 8;

    SpiWriteReg(CC1101_MDMCFG2, m2DCOFF + m2MODFM + m2MANCH + m2SYNCM);
#else
    regRMW(CC1101_MDMCFG2,v, 3, 3);
#endif

}


/****************************************************************
* FUNCTION NAME:Set SYNC_MODE
* FUNCTION     :Combined sync-word qualifier mode
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setSyncMode(byte v)
{
#if OEM_CODE
    Split_MDMCFG2();
    m2SYNCM = 0;

    if (v > 7)
        v = 7;

    m2SYNCM = v;
    SpiWriteReg(CC1101_MDMCFG2, m2DCOFF + m2MODFM + m2MANCH + m2SYNCM);
#else
   if (v > 7) v = 7;

   static const char *msg[] =
   {
		"No preamble/sync. ",
	   	"16 sync word bits detected. ",
	   	"16/16 sync word bits detected. ",
	   	"30/32 sync word bits detected. ",
	   	"No preamble/sync, carrier-sense above threshold. ",
	   	"15/16 + carrier-sense above threshold. ",
	   	"16/16 + carrier-sense above threshold. ",
	   	"30/32 + carrier-sense above threshold."
   };
   
   Serial.printf(FG_MAGENTA "\n%s mode = %s\n" _DONE, __FUNCTION__, msg[v]);
   regRMW(CC1101_MDMCFG2, v , 2, 0);
#endif
}


/****************************************************************
* FUNCTION NAME:Set FEC
* FUNCTION     :Enable Forward Error Correction (FEC)
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setFEC(bool v)
{
#if OEM_CODE
    Split_MDMCFG1();
    m1FEC = 0;

    if (v == 1)
        m1FEC = 128;

    SpiWriteReg(CC1101_MDMCFG1, m1FEC + m1PRE + m1CHSP);
#else
	Serial.printf(FG_MAGENTA "\n%s: %s\n" _DONE, __FUNCTION__, v ? "ON":"OFF");
	
	regRMW(CC1101_MDMCFG1, v,7,7);
#endif
}


/****************************************************************
* FUNCTION NAME:Set PRE
* FUNCTION     :Sets the minimum number of preamble bytes to be transmitted.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setNumPreambleBytes(byte v)
{
#if OEM_CODE
    Split_MDMCFG1();
    m1PRE = 0;

    if (v > 7)
        v = 7;

    m1PRE = v * 16;
    SpiWriteReg(CC1101_MDMCFG1, m1FEC + m1PRE + m1CHSP);
#else
	static const char *msg[] =
	{
		"(0) = 2",
		"(1) = 3",
		"(2) = 4",
		"(3) = 6",
		"(4) = 8",
		"(5) = 12",
		"(6) = 16",
		"(7) = 24"
	};
	
	Serial.printf(FG_MAGENTA "\n%s: %s bytes\n" _DONE, __FUNCTION__, msg[v]);
	regRMW(CC1101_MDMCFG1, v,6 ,4);
	
#endif
}


/****************************************************************
* FUNCTION NAME:Set Channel
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setLogicalChanNum(byte ch)
{
    logical_chan = ch;
    Serial.printf(FG_MAGENTA "%s: logical chan=%d\n" _DONE, __FUNCTION__, ch);
    SpiWriteReg(CC1101_CHANNR, logical_chan);
}


/****************************************************************
* FUNCTION NAME:Set Channel spacing
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setChannelSpacing(float channelSpaceF)
{
#if OEM_CODE
    Split_MDMCFG1();
    byte MDMCFG0 = 0;
    m1CHSP = 0;

    if (channelSpaceF > 405.456543)
        channelSpaceF = 405.456543;

    if (channelSpaceF < 25.390625)
        channelSpaceF = 25.390625;

    for (int i = 0; i < 5; i++)
    {
        if (channelSpaceF <= 50.682068)
        {
            channelSpaceF -= 25.390625;
            channelSpaceF /= 0.0991825;
            MDMCFG0 = channelSpaceF;
            float s1 = (channelSpaceF - MDMCFG0) * 10;

            if (s1 >= 5)
                MDMCFG0++;

            i = 5;
        }
        else
        {
            m1CHSP++;
            channelSpaceF /= 2;
        }
    }

    SpiWriteReg(19, m1CHSP + m1FEC + m1PRE);
    SpiWriteReg(20, MDMCFG0);
#else
	//pg 57
	
	int16_t exp;
	float mantissa;
	int32_t iTest;

	int16_t lockExp = -1;
	int16_t lockMantissa = -1;
	
	Serial.printf(FG_MAGENTA "%s: setting hop size = %5.2f khz\n", __FUNCTION__, channelSpaceF);
	
	channelSpaceF *= 1000.;
	float FIXED = (channelSpaceF * (float)(1<<18)) / (XTAL_Mhz * 1.e6 );
	
	for (exp = 3; exp > -1; exp--)
	{
		float expTest = (float)(1 << exp);
		float mantissa = ((FIXED - 256.0 * expTest)) /expTest;
		iTest = mantissa;
		Serial.printf("\t\texp=%d  mant=%d\n", exp, (int)mantissa);

		if (iTest < 0) continue;	// negative is bad for pll
		if (iTest > 255) continue;	// can't fit in a 8bit register

		if (lockExp < 0)
		{
			lockExp = exp;
			lockMantissa = iTest;
		}
	}
	
	Serial.printf("\tlock Mant=%d Exp=%d\n" _DONE, lockMantissa, lockExp);
	
    regRMW(CC1101_MDMCFG1, lockExp, 1, 0);
    regRMW(CC1101_MDMCFG0, lockMantissa, 7, 0);
#endif

}


/****************************************************************
* FUNCTION NAME:Set Receive bandwidth
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setRxBW(float rxBw)
{
#if OEM_CODE
    Split_MDMCFG4();
    int s1 = 3;
    int s2 = 3;

    for (int i = 0; i < 3; i++)
    {
        if (rxBw > 101.5625)
        {
            rxBw /= 2; s1--;
        }
        else
        {
            i = 3;
        }
    }

    for (int i = 0; i < 3; i++)
    {
        if (rxBw > 58.1)
        {
            rxBw /= 1.25; s2--;
        }
        else
        {
            i = 3;
        }
    }

    s1 *= 64;
    s2 *= 16;
    m4RxBw = s1 + s2;
    SpiWriteReg(16, m4RxBw + m4DaRa);
#else
	int16_t exp;
	float mantissa;
	int32_t iMant;

	int16_t lockExp = -1;
	int16_t lockMantissa = -1;

	Serial.printf(FG_MAGENTA "\n%s: setting rx bw = %5.2f khz\n" _DONE, __FUNCTION__, rxBw);

	rxBw *= 1000.;
	float FIXED = (XTAL_Mhz * 1.e6) / (rxBw * 8.);

	for (exp = 0; exp < 4; exp++)
	{
		float expTest = (float)(1 << exp);
		float mantissa = ((FIXED - 4 * expTest)) /expTest;
		iMant = mantissa;
		Serial.printf("\t\texp=%d  mant=%d\n", exp, (int)mantissa);

		if (iMant < 0) continue;	// negative is bad for pll
		if (iMant > 3) continue;	// can't fit in a 2bit register

		if (lockExp < 0)
		{
			lockExp = exp;
			lockMantissa = iMant;
		}
	}

	Serial.printf("\tlock Mant=%d Exp=%d\n", lockMantissa, lockExp);

	regRMW(CC1101_MDMCFG4, lockExp, 7, 6);
	regRMW(CC1101_MDMCFG4, lockMantissa, 5, 4);
#endif
}


/****************************************************************
* FUNCTION NAME:Set Data Rate
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setDataRateKhz(float dRate)
{
#if OEM_CODE
    Split_MDMCFG4();
    float c = dRate;
    byte MDMCFG3 = 0;

    if (c > 1621.83)
        c = 1621.83;

    if (c < 0.0247955)
        c = 0.0247955;

    m4DaRa = 0;

    for (int i = 0; i < 20; i++)
    {
        if (c <= 0.0494942)
        {
            c = c - 0.0247955;
            c = c / 0.00009685;
            MDMCFG3 = c;
            float s1 = (c - MDMCFG3) * 10;

            if (s1 >= 5)
                MDMCFG3++;

            i = 20;
        }
        else
        {
            m4DaRa++;
            c = c / 2;
        }
    }

    SpiWriteReg(16, m4RxBw + m4DaRa);
    SpiWriteReg(17, MDMCFG3);
#else
	int16_t exp;
	double mantissa;
	int32_t iTest;

	int16_t lockExp = -1;
	int16_t lockMantissa = -1;
	
	Serial.printf(FG_MAGENTA "\n%s: setting data rate = %5.2f khz\n" FG_BCYAN, __FUNCTION__, dRate);
	
	dRate *= 1000.;
	double FIXED = dRate * (double)(1 << 28)/ (double)(XTAL_Mhz * 1.e6 );
	
	for (exp = 16; exp > -1; exp--)  // exp reg is 4 bits.
	{
		double expTest = (float)(1 << exp);
		double mantissa = ((FIXED - 256.0 * expTest)) /expTest;
		iTest = mantissa;
		Serial.printf("\t\texp=%d  mant=%d\n", exp, (int)mantissa);

		if (iTest < 0) continue;	// negative is bad for pll
		if (iTest > 255) continue;	// can't fit in a 8bit register

		if (lockExp < 0)
		{
			lockExp = exp;
			lockMantissa = iTest;
		}
		
		// chip lockup if lt 54. Pin it!
		if (!lockExp && lockMantissa < 54) lockMantissa = 54;
	}

	// test lockMantissa = 34;
	// test lockExp = 12;
	
	float resultHz = ( 256. + (float)lockMantissa) * (float)(1<< lockExp) * XTAL_Hz 
					/ (float)(1 << 28);
					
	Serial.printf(FG_BGREEN "\t\tlock Mant=%d Exp=%d Result=%f\n" _DONE, 
			lockMantissa, lockExp, resultHz);
	
    regRMW(CC1101_MDMCFG4, lockExp, 3, 0);
    regRMW(CC1101_MDMCFG3, lockMantissa, 7, 0);
#endif

}



/****************************************************************
* FUNCTION NAME:Set setSymbolSpacingHz
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setSymbolSpacingHz(float HzBetweenSymbol)
{
#if OEM_CODE
    float f = 1.586914;
    float v = 0.19836425;
    int c = 0;

    if (HzBetweenSymbol > 380.859375)
        HzBetweenSymbol = 380.859375;

    if (HzBetweenSymbol < 1.586914)
        HzBetweenSymbol = 1.586914;

    for (int i = 0; i < 255; i++)
    {
        f += v;

        if (c == 7)
        {
            v *= 2; c = -1; i += 8;
        }

        if (f >= HzBetweenSymbol)
        {
            c = i; i = 255;
        }

        c++;
    }

    SpiWriteReg(21, c);
#else
	int16_t exp;
	float mantissa;
	int32_t iMant;

	int16_t lockExp = -1;
	int16_t lockMantissa = -1;

	Serial.printf(FG_MAGENTA "\n%s: spacing between symbols = %5.2f hz\n" _DONE, __FUNCTION__, HzBetweenSymbol);
	HzBetweenSymbol *= 4.0;   // four posts
	HzBetweenSymbol /= 3.;		// three panels
	
 	float FIXED = HzBetweenSymbol * (float)(1 << 17)/ (XTAL_Mhz * 1.e6 );

	for (exp = 0; exp < 8; exp++)  // exp reg is 3 bits.
	{
		float expTest = (float)(1 << exp);
		float mantissa = ((FIXED - 8 * expTest)) /expTest;
		iMant = mantissa;
		Serial.printf("\t\texp=%d  mant=%d\n", exp, (int)mantissa);

		if (iMant < 0) continue;	// negative is bad for pll
		if (iMant > 7) continue;	// can't fit in a 3 bit register

		if (lockExp < 0)
		{
			lockExp = exp;
			lockMantissa = iMant;
		}
	}
	regRMW(CC1101_DEVIATN, lockMantissa, 2, 0);
	regRMW(CC1101_DEVIATN, lockExp, 6, 4);


	//lockMantissa = 1;
	//lockExp = 1;       // 1785 pull hi or low

	for (int lockM = lockMantissa -1; lockM < lockMantissa+2; lockM++)
	{
#if 1
	float result = XTAL_Hz 
					* (8. + lockM) * (float) (1<< lockExp)
					/(float)(2<<17);
#endif
	
	Serial.printf("\tlock Mant=%d Exp=%d final= +/- %f\n", lockM, lockExp, result);
#endif
	}
}

/****************************************************************
* FUNCTION NAME:Set setDeviation_FSK2
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setDeviation_FSK2(float fdev)
{
#if OEM_CODE
    float f = 1.586914;
    float v = 0.19836425;
    int c = 0;

    if (fdev > 380.859375)
        fdev = 380.859375;

    if (fdev < 1.586914)
        fdev = 1.586914;

    for (int i = 0; i < 255; i++)
    {
        f += v;

        if (c == 7)
        {
            v *= 2; c = -1; i += 8;
        }

        if (f >= fdev)
        {
            c = i; i = 255;
        }

        c++;
    }

    SpiWriteReg(21, c);
#else
	int16_t exp;
	float mantissa;
	int32_t iMant;

	int16_t lockExp = -1;
	int16_t lockMantissa = -1;

	Serial.printf(FG_MAGENTA "\n%s: setting deviation (max pull left or right) = %5.2f khz\n" _DONE, __FUNCTION__, fdev);

	fdev *= 1000.;
	float FIXED = fdev * (float)(1 << 17)/ (XTAL_Mhz * 1.e6 );

	for (exp = 0; exp < 8; exp++)  // exp reg is 3 bits.
	{
		float expTest = (float)(1 << exp);
		float mantissa = ((FIXED - 8 * expTest)) /expTest;
		iMant = mantissa;
		Serial.printf("\t\texp=%d  mant=%d\n", exp, (int)mantissa);

		if (iMant < 0) continue;	// negative is bad for pll
		if (iMant > 7) continue;	// can't fit in a 3 bit register

		if (lockExp < 0)
		{
			lockExp = exp;
			lockMantissa = iMant;
		}
	}
	regRMW(CC1101_DEVIATN, lockMantissa, 2, 0);
	regRMW(CC1101_DEVIATN, lockExp, 6, 4);


	//lockMantissa = 1;
	//lockExp = 1;       // 1785 pull hi or low

	for (int lockM = lockMantissa -1; lockM < lockMantissa+2; lockM++)
	{
#if 1
	float result = XTAL_Hz 
					* (8. + lockM) * (float) (1<< lockExp)
					/(float)(2<<17);
#endif
	
	Serial.printf("\tlock Mant=%d Exp=%d final= +/- %f\n", lockM, lockExp, result);
#endif
	}
}

/****************************************************************
* FUNCTION NAME:RegConfigSettings
* FUNCTION     :CC1101 register config //details refer datasheet of CC1101/CC1100//
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::RegConfigSettings(void)
{
    SpiWriteReg(CC1101_FSCTRL1, 0x06);

    setCCMode(ccmode);
    setMHZ(gMHz);

    SpiWriteReg(CC1101_MDMCFG1, 0x02);
    SpiWriteReg(CC1101_MDMCFG0, 0xF8);
    SpiWriteReg(CC1101_CHANNR, logical_chan);
    SpiWriteReg(CC1101_DEVIATN, 0x47);
    SpiWriteReg(CC1101_FREND1, 0x56);
    SpiWriteReg(CC1101_MCSM0, 0x18);
    SpiWriteReg(CC1101_FOCCFG, 0x16);
    SpiWriteReg(CC1101_BSCFG, 0x1C);
    SpiWriteReg(CC1101_AGCCTRL2, 0xC7);
    SpiWriteReg(CC1101_AGCCTRL1, 0x00);
    SpiWriteReg(CC1101_AGCCTRL0, 0xB2);
    SpiWriteReg(CC1101_FSCAL3, 0xE9);
    SpiWriteReg(CC1101_FSCAL2, 0x2A);
    SpiWriteReg(CC1101_FSCAL1, 0x00);
    SpiWriteReg(CC1101_FSCAL0, 0x1F);
    SpiWriteReg(CC1101_FSTEST, 0x59);
    SpiWriteReg(CC1101_TEST2, 0x81);
    SpiWriteReg(CC1101_TEST1, 0x35);
    SpiWriteReg(CC1101_TEST0, 0x09);
    SpiWriteReg(CC1101_PKTCTRL1, 0x04);
    SpiWriteReg(CC1101_ADDR, 0x00);
    SpiWriteReg(CC1101_PKTLEN, 0x00);
}


/****************************************************************
* FUNCTION NAME:EnterTxMode
* FUNCTION     :set CC1101 send data
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::EnterTxMode(void)
{
	Serial.printf("************* Enter tx mode \n");
    SpiStrobe(CC1101_SIDLE);
    setMHZ(gMHz);
    
    SpiStrobe(CC1101_STX);      //start send
    
    Serial.printf(FG_FYELLOW "%s: TX MODE !!!! \n", __FUNCTION__);
    trxstate = MODEM_TX;

    
    getState();
}


/****************************************************************
* FUNCTION NAME:EnterRxMode
* FUNCTION     :set CC1101 to receive state
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::EnterRxMode(void)
{
	Serial.printf("************** EnterRxMode ****\n");
    SpiStrobe(CC1101_SIDLE);
    SpiStrobe(CC1101_SRX);      //start receive
    
    Serial.printf(FG_FYELLOW "%s: RX MODE !!!! \n", __FUNCTION__);
    trxstate = MODEM_RX;
    
    getState();
}

/****************************************************************
* FUNCTION NAME:EnterRxMode
* FUNCTION     :set CC1101 to receive state and change frequency
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::EnterRxMode(float mhz)
{
	Serial.printf("************* EnterRxMode + FREQ = %f ****\n", mhz);
    SpiStrobe(CC1101_SIDLE);
    setMHZ(mhz);
    SpiStrobe(CC1101_SRX);      //start receive
    
    Serial.printf(FG_FYELLOW "%s: RX MODE + freq !!!! \n", __FUNCTION__);
    trxstate = MODEM_RX;
    
    getState();
}


/****************************************************************
* FUNCTION NAME:RSSI Level
* FUNCTION     :Calculating the RSSI Level
* INPUT        :none
* OUTPUT       :none
****************************************************************/
int ELECHOUSE_CC1101::getRssi(void)
{
    int rssi;

    rssi = SpiReadStatus(CC1101_RSSI);

    if (rssi >= 128)
        rssi = (rssi - 256) / 2 - 74;
    else
        rssi = (rssi / 2) - 74;

    return rssi;
}


/****************************************************************
* FUNCTION NAME:LQI Level
* FUNCTION     :get Lqi state
* INPUT        :none
* OUTPUT       :none
****************************************************************/
byte ELECHOUSE_CC1101::getLqi(void)
{
    byte lqi;

    lqi = SpiReadStatus(CC1101_LQI);
    return lqi;
}

byte ELECHOUSE_CC1101::getState(void)
{
	byte status;
	static const char *msg[] = 
	{
		"SLEEP",	"SLEEP",	
		"IDLE",	 	"IDLE",	
		"XOFF",	 	"XOFF",	
		"VCOON",	"MANCAL",	
		"REGON",	"MANCAL",	
		"MANCAL",	"MANCAL",	
		"VCOONFS",	"_WAKEUP",	
		"REGONFS_",	"WAKEUP",	
		"STARTCAL",	"CALIBRATE",	
		"BWBOOST",	"SETTLING",	
		"FS_LOCK",	"SETTLING",	
		"IFADCON",	"SETTLING",	
		"ENDCAL",	"CALIBRATE",	
		"RX",	 	"RX",	
		"RX_END",	"RX",	
		"RX_RST",	"RX",	
		"TXRX_SWITCH",	 	"TXRX_SETTLING",	
		"RXFIFO_OVERFLOW",	"RXFIFO_OVERFLOW",	
		"FSTXON",	 		"FSTXON",	
		"TX",	 			"TX",	
		"TX_END",	 		"TX",	
		"RXTX_SWITCH",	 	"RXTX_SETTLING",	
		"TXFIFO_UNDERFLOW", "TXFIFO_UNDERFLOW	",
	};
	
    status = SpiReadStatus(CC1101_MARCSTATE);
	Serial.printf(FG_GREEN "%s:  %d = %s\n", __FUNCTION__, status, msg[ status *2 + 1]);
	
    
    return status;
}
 
/****************************************************************
* FUNCTION NAME:SetSres
* FUNCTION     :Reset CC1101
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setSres(void)
{
    Serial.println("****** chip h/w reset ***\n");
    SpiStrobe(CC1101_SRES);
    trxstate = MODEM_IDLE;
}


/****************************************************************
* FUNCTION NAME:setSidle
* FUNCTION     :set Rx / TX Off
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::EnterIdleMode(void)
{
    SpiStrobe(CC1101_SIDLE);
    trxstate = MODEM_IDLE;
    
    Serial.printf(FG_FYELLOW "%s: IDLE !!!! \n", __FUNCTION__);
    getState();
}


/****************************************************************
* FUNCTION NAME:goSleep
* FUNCTION     :set cc1101 Sleep on
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::goSleep(void)
{
    trxstate = MODEM_IDLE;
    SpiStrobe(0x36);    //Exit RX / TX, turn off frequency synthesizer and exit
    SpiStrobe(0x39);    //Enter power down mode when CSn goes high.
    
    Serial.printf(FG_FYELLOW "%s: SLEEP !!!! \n", __FUNCTION__);
}


/****************************************************************
* FUNCTION NAME:Char direct SendDataCharArray
* FUNCTION     :use CC1101 send data
* INPUT        :txBuffer: data array to send; size: number of data to send, no more than 61
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SendDataCharArray(char *txchar)
{
    SendBinaryData((byte *) txchar, strlen(txchar)+1 );  // +1 send null terminator too!
}

#include <string>
#include <cstring> 


void ELECHOUSE_CC1101::SendDataCppString(String &txchar)
{
    int len = txchar.length();
    char chartobyte[len+1];

	strcpy (chartobyte, txchar.c_str());

    // a CString has no null terminator.... just send len
    SendBinaryData((byte*)chartobyte, len); 
}
/****************************************************************
* FUNCTION NAME:SendBinaryData
* FUNCTION     :use CC1101 send data
* INPUT        :txBuffer: data array to send; size: number of data to send, no more than 61
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SendBinaryData(byte *txBuffer, byte size)
{
	if (gMHz > 866 && gMHz < 868) Serial.printf("***** DANGER TX FREQ = %f\n", gMHz);

    _SpiWriteReg("CC1101_TXFIFO", CC1101_TXFIFO, size, true);

    SpiWriteBurstReg(CC1101_TXFIFO, txBuffer, size);    //write data to send

    SpiStrobe(CC1101_SIDLE);
    SpiStrobe(CC1101_STX);      //start send

	// can't get out of here ??? you didn't power up the MBUS dumbass
    while (!digitalRead(GDO0)); // -> sync transmitted
    while ( digitalRead(GDO0)); // -> end of packet

    SpiStrobe(CC1101_SFTX);                 //flush TXfifo
    trxstate = MODEM_TX;
}


/****************************************************************
* FUNCTION NAME:SendBinaryDataWithNoGDO
* FUNCTION     :use CC1101 send data without GDO
* INPUT        :txBuffer: data array to send; size: number of data to send, no more than 61
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SendBinaryDataWithNoGDO(byte *txBuffer, byte size, int t)
{
	if (gMHz > 866 && gMHz < 868) Serial.printf("*****  DANGER TX FREQ = %f\n", gMHz);

    SpiWriteReg(CC1101_TXFIFO, size);
    SpiWriteBurstReg(CC1101_TXFIFO, txBuffer, size);    //write data to send
    SpiStrobe(CC1101_SIDLE);
    SpiStrobe(CC1101_STX);                              //start send
    delay(t);
    SpiStrobe(CC1101_SFTX);                             //flush TXfifo
    trxstate = MODEM_TX;
}


/****************************************************************
* FUNCTION NAME:Check CRC
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
bool ELECHOUSE_CC1101::CheckCRC(void)
{
    byte lqi = SpiReadStatus(CC1101_LQI);
    bool crc_ok = bitRead(lqi, 7);

    if (crc_ok == 1)
    {
        return 1;
    }
    else
    {
        SpiStrobe(CC1101_SFRX);
        SpiStrobe(CC1101_SRX);
        return 0;
    }
}


/****************************************************************
* FUNCTION NAME:CheckRxFifo
* FUNCTION     :check receive data or not
* INPUT        :none
* OUTPUT       :flag: 0 no data; 1 receive data
****************************************************************/
bool ELECHOUSE_CC1101::CheckRxFifo(int t)
{
    if (trxstate != MODEM_RX)
        EnterRxMode();

    if (SpiReadStatus(CC1101_RXBYTES) & BYTES_IN_RXFIFO)
    {
        delay(t);
        return 1;
    }
    else
    {
        return 0;
    }
}


/****************************************************************
* FUNCTION NAME:CheckReceiveFlag
* FUNCTION     :check receive data or not
* INPUT        :none
* OUTPUT       :flag: 0 no data; 1 receive data
****************************************************************/
byte ELECHOUSE_CC1101::CheckReceiveFlag(void)
{
    if (trxstate != MODEM_RX)
        EnterRxMode();

    if (digitalRead(GDO0))                      //receive data
    {
        while (digitalRead(GDO0));
        return 1;
    }
    else                                                        // no data
    {
        return 0;
    }
}


/****************************************************************
* FUNCTION NAME:ReceiveData
* FUNCTION     :read data received from RXfifo
* INPUT        :rxBuffer: buffer to store data
* OUTPUT       :size of data received
****************************************************************/
byte ELECHOUSE_CC1101::ReceiveData(byte *rxBuffer)
{
    byte size;
    byte status[2];

    if (SpiReadStatus(CC1101_RXBYTES) & BYTES_IN_RXFIFO)
    {
        size = SpiReadReg(CC1101_RXFIFO);
        SpiReadBurstReg(CC1101_RXFIFO, rxBuffer, size);
        SpiReadBurstReg(CC1101_RXFIFO, status, 2);
        SpiStrobe(CC1101_SFRX);
        SpiStrobe(CC1101_SRX);
        return size;
    }
    else
    {
        SpiStrobe(CC1101_SFRX);
        SpiStrobe(CC1101_SRX);
        return 0;
    }
}


ELECHOUSE_CC1101 ELECHOUSE_cc1101;

