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
#include "bandcal.h"

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

int8_t gModulation = -1;
byte logical_chan = 0;
int usrPwrLvlDb = 12;
byte paTableNumber;
byte SCK_PIN;
byte MISO_PIN;
byte MOSI_PIN;
byte SS_PIN;
byte GDO0;
byte GDO2;
bool spi = 0;
eGDIO_MODES ccmode = NOT_INITED;
eMODEM_STATE trxstate = MODEM_IDLE;
float gMHz = 905.0;

byte pc0PktForm;
byte pc0LenConf;


// NOTE: this is now expressed in hertz, not Smartnet vals
typedef struct CALPOINT
{
	// keep both as int for slope math
	int32_t freq;  
	int32_t cal;
};

typedef struct HI_LOW
{
	CALPOINT left;
	CALPOINT right;
};


HI_LOW Band_300_348 = { {300000000,   2000} , {  348000000,  3000} };	// made up
HI_LOW Band_378_464 = { {378000000,   3000} , {  464000000,  4000} };	// made up
HI_LOW Band_779_899 = { {792006330,  -8750} , {  900000000, -2730} };	// 792 ott beacon CAL'd
HI_LOW Band_900_928 = { {900000000,  -2730} , {  931386000, -2000} };   // CAL'd


int16_t mirror[64];


static const double XTAL_Mhz=26.0;
static const double XTAL_Hz=( 26.0 * 1e6);

/****************************************************************/
//                          -30   -20   -15   -10     0     5     7    10  +15  +30
uint8_t PA_TABLE[8]     = { 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00              };
uint8_t PA_TABLE_315[8] = { 0x12, 0x0D, 0x1C, 0x34, 0x51, 0x85, 0xCB, 0xC2              };                //300 - 348
uint8_t PA_TABLE_433[8] = { 0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0              };                //387 - 464
uint8_t PA_TABLE_868[10]= { 0x03, 0x17, 0x1D, 0x26, 0x37, 0x50, 0x86, 0xCD, 0xC5, 0xC0, };   //779 - 899.99
uint8_t PA_TABLE_915[10]= { 0x03, 0x0E, 0x1E, 0x27, 0x38, 0x8E, 0x84, 0xCC, 0xC3, 0xC0, };   //900 - 928


template <typename T> void binary( T input)
{
	T copy = input;
	Serial.print('b');
	
	for(int i = sizeof(T)*8 - 1; i > -1; i--)
	{
		Serial.printf("%d", !!(input & (1 << i)));
	}
}




template <typename T> T regMaskRead( T final, uint8_t lhs, uint8_t rhs)
{
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
	return oldField;
}

template <typename T> T regMaskWrite( T &final, T newField, uint8_t lhs, uint8_t rhs)
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

	// must have, some reg fields are signed values, dont smash other fields.
	newField &= mask; 
	
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
#define setField(name, val, lhs, rhs)	_setField(#name, name, (uint8_t)val, lhs, rhs)
#define getField(name, lhs, rhs) 		_getField(#name, name, lhs, rhs)


//---------------------------------------------------------------------

// dont read reg 0x30 onwards. those are commands, they don't have addr/data. addr triggers ops!

#define CC1101_REG_COUNT 0x2F  


static uint8_t snap1[CC1101_REG_COUNT];
static uint8_t snap2[CC1101_REG_COUNT];

void ELECHOUSE_CC1101::snapshot1(void)
{
	for (uint8_t i = 0; i < CC1101_REG_COUNT; i++)
	{
		snap1[i] = SpiReadReg((CONFIG_REG)i);
	}
}

void ELECHOUSE_CC1101::snapshot2(void)
{
	for (uint8_t i = 0; i < CC1101_REG_COUNT; i++)
	{
		snap2[i] = SpiReadReg((CONFIG_REG)i);
	}
}

void ELECHOUSE_CC1101::diffSnapshots(void)
{
	Serial.printf(FG_MAGENTA "\n\n%s  ---------------start ------\n", __FUNCTION__);
	for (uint8_t i = 0; i < CC1101_REG_COUNT; i++)
	{
		if (snap2[i] == snap1[i]) continue;
		Serial.printf("[0x%2X]\t0x%02X   ",i, snap1[i]); 
		binary(snap1[i]); 
		Serial.println();
		
		Serial.printf("\t0x%02X   ", snap2[i]); 
		binary(snap2[i]); 
		Serial.println();
		
		Serial.printf("\t       "); binary( (uint8_t) (snap1[i] ^ snap2[i]) );	Serial.println();
		Serial.printf("\t        76543210\n\n");
	}
	Serial.printf(" ------------------ done\n" FG_DONE);
}


//---------------------------------------------------------------------

uint8_t ELECHOUSE_CC1101::_getField(const char *regName, uint8_t regNum, uint8_t LHS, uint8_t RHS)
{
	uint8_t orig = SpiReadReg((CONFIG_REG) regNum);
	
	uint8_t found = regMaskRead<uint8_t> ( orig, LHS, RHS);

	Serial.printf("\n[0x%02X] %s %02d:%02d = 0x%02X\n", regNum, regName, LHS, RHS, found ); 

	return found;	
}	

//---------------------------------------------------------------------

void ELECHOUSE_CC1101::_setField(const char *regName, uint8_t regNum, uint8_t value, uint8_t LHS, uint8_t RHS)
{
	uint8_t regNow = SpiReadReg((CONFIG_REG)regNum);
	
	if ( mirror[regNum] >= 0) // has been written to before?
	{
		if (mirror[regNum] != regNow)
		{
			Serial.printf(FG_BRED "\n[0x%X] %s REG ERROR" FG_DONE,  regNum, regName);
			Serial.printf("\t expect 0x%02X ", mirror[regNum]);
			uint8_t small =  mirror[regNum];
			binary(small);
			Serial.printf(" but found 0x%02X ", regNow);
			binary(regNow);
			Serial.println();

			DumpMirror("REGISTER MISMATCH");
		}
	}

	uint8_t temp = regNow;
	uint8_t want = regMaskWrite<uint8_t> ( temp, value, LHS, RHS);

	Serial.printf("[0x%02X] %s 0x%02X\n", regNum, regName, want ); 
	
	if(regNow != want)
	{
		int x;
		for (x = 0; x < 10; x++)
		{
			_SpiWriteReg(regName, (CONFIG_REG) regNum, want, 1); //silent
			regNow = SpiReadReg((CONFIG_REG)regNum);
			if (regNow == want) break;
		}
		
		if (x == 10) 
		{
			Serial.printf(FG_RED "\n%s FAIL TO WRITE %s want 0x%X found 0x%X\n" FG_DONE, __FUNCTION__, regName, want, regNow);
			delay(5000);
		}	
	}	
}	
//---------------------------------------------------------------------

void binary (unsigned char byte) {
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
		uint8_t read = SpiReadReg((CONFIG_REG)regs);
		Serial.printf("\t0x%02X    0x%02X  ", regs, read);
		binary(read);
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
    Serial.printf(FG_FYELLOW "%s: RESET !!!! \n" FG_DONE, __FUNCTION__); 

	for (int i = 0; i < CC1101_REG_COUNT; i++) mirror[i] = -1;  // 'never written to'

	DumpMirror("SIMPLE INIT");
}

void ELECHOUSE_CC1101::DumpMirror(char *msg)
{
    Serial.printf(FG_FYELLOW "%s: %s \n" FG_DONE, __FUNCTION__, msg);
	
	for (int i = 0; i < CC1101_REG_COUNT; i++) 
	{
		uint8_t regNow = SpiReadReg((CONFIG_REG) i);
		Serial.printf("\t [0x%02X] 0x%8X   0x%2X\n", i, mirror[i], regNow);
	}
	
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
void ELECHOUSE_CC1101::_SpiWriteReg(const char*name , CONFIG_REG addr, byte value, bool bQuiet)
{
    SpiStart();

	//if ( addr == 0x15 && value != 0)
	//{
	//	assert(0);
	//}
	
    assert(addr < 64);
    
	mirror[addr] = value;
    digitalWrite(SS_PIN, LOW);
    digitalWrite(SS_PIN, LOW);

    MY_SPI.transfer(addr);
    MY_SPI.transfer(value);
    digitalWrite(SS_PIN, HIGH);
    SpiEnd();
    
    if (!bQuiet) Serial.printf(FG_WHITE "\t%s [0x%02X] %s now equals 0x%02X \n" FG_DONE, __FUNCTION__, addr, name, value);
}


/****************************************************************
* FUNCTION NAME:SpiWriteBurstReg
* FUNCTION     :CC1101 write burst data to register
* INPUT        :addr: register address; buffer:register value array; num:number to write
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SpiWriteBurstReg(CONFIG_REG addr, byte *buffer, byte num)
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

typedef struct ONE {
	uint8_t num;
	const char *msg;
};

ONE okay[] =
{
	{ 0x30 ,"SRES Reset chip."},
	{ 0x31 ,"SFSTXON Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).\n\t"
			"If in RX (with CCA) :\n\t\tGo to a wait state where only the synthesizer\n\t\t"
			"is running (for quick RX / TX turnaround)."},
	{ 0x32 ,"SXOFF Turn off crystal oscillator."},
	{ 0x33 ,"SCAL Calibrate frequency synthesizer and turn it off.\n\t"
			"SCAL can be strobed from IDLE mode without setting\n\t"
			"manual calibration mode (MCSM0.FS_AUTOCAL=0)"},
			
	{ 0x34 ,"SRX Enable RX\n\tPerform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1."},
	{ 0x35 ,"STX In IDLE state:Enable TX.\n\tPerform calibration first if MCSM0.FS_AUTOCAL=1.\n\t"
			"If in RX state and CCA is enabled:\n\tOnly go to TX if channel is clear."},
			
	{ 0x36 ,"SIDLE Exit RX / TX, turn off frequency synthesizer\n\tand exit Wake-On-Radio mode if applicable."},
	{ 0x38 ,"SWOR Start automatic RX polling sequence\n\t(Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0."},
	{ 0x39 ,"SPWD Enter power down mode when CSn goes high."},
	{ 0x3A ,"SFRX Flush the RX FIFO buffer.\n\tOnly issue SFRX in IDLE or RXFIFO_OVERFLOW states."},
	{ 0x3B ,"SFTX Flush the TX FIFO buffer.\n\tOnly issue SFTX in IDLE or TXFIFO_UNDERFLOW states."},
	{ 0x3C ,"SWOR Reset real time clock to Event1 value."},
	{ 0x3D ,"SNOP No Op used for getting STATUS"}
	
};




uint8_t ELECHOUSE_CC1101::SpiStrobe(byte commandStrobe, bool bSilent)
{
    SpiStart();
	assert(commandStrobe != 0x3B);
	
    for (int i = 0; i < sizeof(okay)/sizeof(okay[0]); i++)
    {
    	if (commandStrobe != okay[i].num) continue;
    	
		if(!bSilent) Serial.printf(FG_GREEN "\n%s 0x%0X -> %s\n" FG_DONE, __FUNCTION__, okay[i].num, okay[i].msg);
    }

	// commands are 0x30 and above. Configurations are 0x2F and below
	assert(commandStrobe > 0x2F);
	
    digitalWrite(SS_PIN, LOW);
    digitalWrite(SS_PIN, LOW);

    uint8_t ret = MY_SPI.transfer(commandStrobe); // commands only send an address w no data
    
    digitalWrite(SS_PIN, HIGH);
    digitalWrite(SS_PIN, HIGH);

	getState(bSilent);
	
    SpiEnd();
    return ret;
}


/****************************************************************
* FUNCTION NAME:SpiReadReg
* FUNCTION     :CC1101 read data from register
* INPUT        :addr: register address
* OUTPUT       :register value
****************************************************************/
byte ELECHOUSE_CC1101::SpiReadReg(CONFIG_REG addr)
{
    byte temp, value;

    SpiStart();
    temp = (byte) addr | READ_SINGLE;
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
void ELECHOUSE_CC1101::SpiReadBurstReg(CONFIG_REG addr, byte *buffer, byte num)
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
byte ELECHOUSE_CC1101::SpiReadStatus(STATUS_REG addr)
{
    byte value, temp;

    SpiStart();
    temp = (byte) addr | READ_BURST;
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
			Serial.printf(FG_DONE);
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
			Serial.printf(FG_DONE);
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
			Serial.printf(FG_DONE);
			return;
		}
		attachInterrupt(GDO0, onGDO0_IRQ, RISING);
		Serial.printf("%s RISING mode\n", __FUNCTION__);
		irqDirGDO0 = RISING;
	}
	Serial.printf(FG_DONE);
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
			Serial.printf(FG_DONE);
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
			Serial.printf(FG_DONE);
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
			Serial.printf(FG_DONE);
			return;
		}
		attachInterrupt(GDO0, onGDO0_IRQ, FALLING);
		irqDirGDO0 = FALLING;
		Serial.printf("%s no change\n", __FUNCTION__);
	}
	
	Serial.printf(FG_DONE);
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
			Serial.printf(FG_DONE);
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
			Serial.printf(FG_DONE);
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
			Serial.printf(FG_DONE);
			return;
		}
		attachInterrupt(GDO2, onGDO2_IRQ, RISING);
		Serial.printf("%s RISING mode\n", __FUNCTION__);
		irqDirGDO2 = RISING;
	}
	Serial.printf(FG_DONE);
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
			Serial.printf(FG_DONE);
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
			Serial.printf(FG_DONE);
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
			Serial.printf(FG_DONE);
			return;
		}
		attachInterrupt(GDO2, onGDO2_IRQ, FALLING);
		irqDirGDO2 = FALLING;
		Serial.printf("%s no change\n", __FUNCTION__);
	}
	
	Serial.printf(FG_DONE);
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

	assert(CONFIG_IOCFG2 == pinRegNum || CONFIG_IOCFG0 == pinRegNum);

	for (i = 0; i < end; i++)
	{
		if (pin_defs[i].opcode != value) continue;
		if (!bSilent) Serial.printf(FG_BCYAN "\n%s [0x%02X] %s\n" FG_DONE, pinRegNum == CONFIG_IOCFG2 ? "GDO2":"GDO0", value, pin_defs[i].msg);
		break;
	}
	
	if (i == end) Serial.printf(FG_BCYAN "\n%s ERROR ?? [0x%02X] %s\n" FG_DONE, pinRegNum , value, "see documentation"); 
	
	_SpiWriteReg("CC1101_IOCFGx", (CONFIG_REG) pinRegNum, value, 1);  //silent please.

	
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

    if (ccmode == GDO0_isSYNC_TXEND)
    {
    	Serial.printf(FG_RED "%s: ccmode = GDO0 used for sentSYNC and TXend ---------------\n" FG_DONE, __FUNCTION__);

		setGDO0_hostpinMode(INPUT);
		setGDO2_hostpinMode(INPUT);

        setGDOxPinConfig(CONFIG_IOCFG0, 0x06); // + sync sent .... - packet send
        setGDOxPinConfig(CONFIG_IOCFG2, 0x0B); // serial data OUT on GDO2

        //SpiWriteReg(CONFIG_PKTCTRL0, 0x05);
        setPktFormat(0);
        
        setPacketLength(CC_FIFOSIZE);
        setLengthConfig(1);

        setBaudRate(DEFAULT_BAUD);
    }
    else if (ccmode == I_DUNNO)
    {
		setGDO0_hostpinMode(INPUT);
		setGDO2_hostpinMode(INPUT);
		
    	Serial.printf(FG_RED "%s: ccmode = NO FUCKING CLUE ---------------\n" FG_DONE, __FUNCTION__);
        setGDOxPinConfig(CONFIG_IOCFG2, 0x0D); 	// serial data out
        setGDOxPinConfig(CONFIG_IOCFG0, 0x0D);	// serial data out
        
        //SpiWriteReg(CONFIG_PKTCTRL0, 0x32);
        setPktFormat(3);
        setLengthConfig(2);		// infinite

		setBaudRate(DEFAULT_BAUD);
		assert(0);
		
    }
    else if (ccmode == SYMBOL_TICK)
    {
    	Serial.printf(FG_RED "%s: ccmode = SYMBOL_TICK ---------------\n" FG_DONE, __FUNCTION__);

		setGDO0_hostpinMode(INPUT);
		setGDO2_hostpinMode(INPUT);
	
        setGDOxPinConfig(CONFIG_IOCFG2, 0x1D); // SYMBOL TICK
        setGDOxPinConfig(CONFIG_IOCFG0, 0x0D);
        
        setPktFormat(3);		//data in on GDO0 data out on GDOx
        setLengthConfig(2);  	// infinite

		setBaudRate(DEFAULT_BAUD);
		enableRisingIRQ_GDO2(callme);
	}
    else if (ccmode == NOT_INITED)
    {
    	Serial.println("ok");
    }
	else
		assert(ccmode != ccmode);
		
  

    setModulation(DEFAULT_MODULATION);
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

	// common across all selections.
	Serial.printf(FG_MAGENTA "%s: set PA lo current\n" FG_DONE, __FUNCTION__);
	setField(CONFIG_FREND0, 1, 5, 4);

	char type[20];
	
	switch (m)
	{
		case 0:
			strcpy(type, "2-FSK");
			gModulation = 0;
			break;	// 2-FSK

		case 1: 
			strcpy(type,"GFSK");
			gModulation = 1; 
			break;	// GFSK

		case 2: 
			strcpy(type, "OOK");
			gModulation = 3; 

			//Serial.printf(FG_FRED "\n%s: todo ook p/a levels?\n" FG_DONE, __FUNCTION__);
			Serial.printf(FG_MAGENTA "%s: PA power table index = %d\n" FG_DONE, __FUNCTION__, 1);
			setField(CONFIG_FREND0, 1, 2, 0);
			break;	// OOK

		case 3: 
			strcpy(type, "4-FSK");
			gModulation = 4;
			break;	// 4-FSK

		case 4: 
			strcpy(type, "MSK");
			gModulation = 7; 
			break;	// MSK
	}

	Serial.printf(FG_MAGENTA "%s: gModulation %s 0x%X\n" FG_DONE, __FUNCTION__, type, gModulation);
	setField(CONFIG_MDMCFG2, gModulation, 6, 4);


    setPA(usrPwrLvlDb);

}


/****************************************************************
* FUNCTION NAME:PA Power
* FUNCTION     :set CC1101 PA Power
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPA(int needDb)
{
    int maxPwrLvl;

    usrPwrLvlDb = needDb;

    if (gMHz >= 300 && gMHz <= 348)
    {
        if (needDb <= -30)
            maxPwrLvl = PA_TABLE_315[0];
        else if (needDb > -30 && needDb <= -20)
            maxPwrLvl = PA_TABLE_315[1];
        else if (needDb > -20 && needDb <= -15)
            maxPwrLvl = PA_TABLE_315[2];
        else if (needDb > -15 && needDb <= -10)
            maxPwrLvl = PA_TABLE_315[3];
        else if (needDb > -10 && needDb <= 0)
            maxPwrLvl = PA_TABLE_315[4];
        else if (needDb > 0 && needDb <= 5)
            maxPwrLvl = PA_TABLE_315[5];
        else if (needDb > 5 && needDb <= 7)
            maxPwrLvl = PA_TABLE_315[6];
        else if (needDb > 7)
            maxPwrLvl = PA_TABLE_315[7];

        paTableNumber = 1;
    }
    else if (gMHz >= 378 && gMHz <= 464)
    {
        if (needDb <= -30)
            maxPwrLvl = PA_TABLE_433[0];
        else if (needDb > -30 && needDb <= -20)
            maxPwrLvl = PA_TABLE_433[1];
        else if (needDb > -20 && needDb <= -15)
            maxPwrLvl = PA_TABLE_433[2];
        else if (needDb > -15 && needDb <= -10)
            maxPwrLvl = PA_TABLE_433[3];
        else if (needDb > -10 && needDb <= 0)
            maxPwrLvl = PA_TABLE_433[4];
        else if (needDb > 0 && needDb <= 5)
            maxPwrLvl = PA_TABLE_433[5];
        else if (needDb > 5 && needDb <= 7)
            maxPwrLvl = PA_TABLE_433[6];
        else if (needDb > 7)
            maxPwrLvl = PA_TABLE_433[7];

        paTableNumber = 2;
    }
    else if (gMHz >= 779 && gMHz < 900)
    {
        if (needDb <= -30)
            maxPwrLvl = PA_TABLE_868[0];
        else if (needDb > -30 && needDb <= -20)
            maxPwrLvl = PA_TABLE_868[1];
        else if (needDb > -20 && needDb <= -15)
            maxPwrLvl = PA_TABLE_868[2];
        else if (needDb > -15 && needDb <= -10)
            maxPwrLvl = PA_TABLE_868[3];
        else if (needDb > -10 && needDb <= -6)
            maxPwrLvl = PA_TABLE_868[4];
        else if (needDb > -6 && needDb <= 0)
            maxPwrLvl = PA_TABLE_868[5];
        else if (needDb > 0 && needDb <= 5)
            maxPwrLvl = PA_TABLE_868[6];
        else if (needDb > 5 && needDb <= 7)
            maxPwrLvl = PA_TABLE_868[7];
        else if (needDb > 7 && needDb <= 10)
            maxPwrLvl = PA_TABLE_868[8];
        else if (needDb > 10)
            maxPwrLvl = PA_TABLE_868[9];

        paTableNumber = 3;
    }
    else if (gMHz >= 900 && gMHz <= 932)
    {
        if (needDb <= -30)
            maxPwrLvl = PA_TABLE_915[0];
        else if (needDb > -30 && needDb <= -20)
            maxPwrLvl = PA_TABLE_915[1];
        else if (needDb > -20 && needDb <= -15)
            maxPwrLvl = PA_TABLE_915[2];
        else if (needDb > -15 && needDb <= -10)
            maxPwrLvl = PA_TABLE_915[3];
        else if (needDb > -10 && needDb <= -6)
            maxPwrLvl = PA_TABLE_915[4];
        else if (needDb > -6 && needDb <= 0)
            maxPwrLvl = PA_TABLE_915[5];
        else if (needDb > 0 && needDb <= 5)
            maxPwrLvl = PA_TABLE_915[6];
        else if (needDb > 5 && needDb <= 7)
            maxPwrLvl = PA_TABLE_915[7];
        else if (needDb > 7 && needDb <= 10)
            maxPwrLvl = PA_TABLE_915[8];
        else if (needDb > 10)
            maxPwrLvl = PA_TABLE_915[9];

        paTableNumber = 4;
    }
    else
    {
    	Serial.printf(FG_RED "***************** cannot handle this freq %f\n", gMHz);
    	assert(0);
    }

	assert(gModulation != -1);  // nobody set the moduation yet!!!

	Serial.printf(FG_BGREEN "%s: modu=%d usr pwr req = %d table pwr = %d\n", __FUNCTION__, gModulation, needDb, maxPwrLvl);
	
    if (gModulation == 2)
    {
        PA_TABLE[0] = 0;		  //ook uses index 0 for tx off power level
        PA_TABLE[1] = maxPwrLvl;  //ook uses index 1 for tx on power level
    }
    else
    {
        PA_TABLE[0] = maxPwrLvl;  // index 0 is the ON power level
        PA_TABLE[1] = 0;		  // index 1 is not used, there is no "OFF" power
    }

    SpiWriteBurstReg(CONFIG_PATABLE, PA_TABLE, 8);
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

void ELECHOUSE_CC1101::setFreqHz(uint32_t mhz, bool bSilent, bool bSkipBandCal)
{
	setMHZ((float) mhz/1e6, bSilent, bSkipBandCal);
}

void ELECHOUSE_CC1101::setMHZ(float mhz, bool bSilent, bool bSkipBandCal)
{
   	uint32_t  temp;
   	int32_t   tweak;
   	
	if (mhz == 0.0 ) mhz = gMHz;
	
	if (mhz > 866 && mhz < 868) Serial.printf(FG_FRED "***** DANGER TX FREQ = %f\n" FG_DONE, gMHz);

	gMHz= mhz;

	// addband cal sets too many 'other' registers like pa power etc
	// always call it. Take away freq cal if needed later.
   	tweak = AddBandCal(bSilent);
   	
    if (bSkipBandCal)
    {
    	//take away any band FREQUENCY aspect, zero it.
    	//range is ±202 kHz set to 0
    	SpiWriteReg(CONFIG_FSCTRL0, 0);
		Serial.printf(FG_FRED "%s  requested %9.6f passthru\n" FG_DONE, __FUNCTION__, mhz);
	}
	else
	{
		// tweek in play. Normal operation.
		Serial.printf(FG_FRED "%s  requested %9.6f becomes %9.6f\n" FG_DONE, __FUNCTION__, mhz, mhz + (float) tweak / 1000000.);
		mhz += (float) tweak / 1000000.;
	}
	

	temp = (( mhz  * (float)(1 << 16))/ XTAL_Mhz);

 	if (!bSilent) Serial.printf(FG_CYAN "%s: final h/w tgt=%7.3f\n"  FG_DONE, 
 			__FUNCTION__, mhz);
	
	SpiWriteReg(CONFIG_FREQ2, (temp >>16) & 0xFF);
	SpiWriteReg(CONFIG_FREQ1, (temp >> 8) & 0xFF);
	SpiWriteReg(CONFIG_FREQ0,  temp       & 0xFF);
	

#if 0
	// verify.
	uint32_t tweaked = SpiReadReg(CONFIG_FREQ2) << 16 | SpiReadReg(CONFIG_FREQ1)  << 8 | SpiReadReg(CONFIG_FREQ0);

               
	double retest;
	retest = (XTAL_Mhz / (double)(1<<16)) * (double) tweaked;
	Serial.printf("%s VERIFY = %f mhz \n", __FUNCTION__, (float) retest);

	double err = adjFreq - retest;

	Serial.printf("%s error = %d hz\n\n", __FUNCTION__, (int) err);
#endif
}


/****************************************************************
* FUNCTION NAME:AddBandCal
* INPUT        :none
* OUTPUT       :none
****************************************************************/
int32_t ELECHOUSE_CC1101::AddBandCal(bool bSilent)
{
	bSilent = false;
	
	//CONFIG_FSCTRL0 = add offset to any setMHZ command BY HARDWARE!
	//CONFIG_TEST0 = no clue. Too obtuse.
	int32_t retOffset;
	
	const int32_t hzPerStep = (XTAL_Mhz * 1e6)/(float) (1<<14);
	Serial.printf(FG_GREEN "\n%s hz/step = %d\n" FG_DONE, __FUNCTION__, hzPerStep); 

	int32_t freqHz = (uint32_t) (gMHz * 1e6);
	
    if (gMHz >= 300 && gMHz <= 348)
    {
    	
        retOffset = REMAP(freqHz, Band_300_348.left.freq, Band_300_348.left.cal, Band_300_348.right.freq, Band_300_348.right.cal);

		if (!bSilent) Serial.printf(FG_GREEN "%s 300->348 a %d hz internal HW retOffset to %f -> %f \n" FG_DONE, __FUNCTION__, retOffset, gMHz, gMHz+ (float) retOffset/1000000. ); 
        
        /// DO NOT USE TOO COURSE SpiWriteReg(CONFIG_FSCTRL0, retOffset / hzPerStep);

        if (gMHz < 322.88)
        {
            SpiWriteReg(CONFIG_TEST0, 0x0B);
        }
        else
        {
            SpiWriteReg(CONFIG_TEST0, 0x09);
            int s = radio.SpiReadReg(CONFIG_FSCAL2);

            if (s < 32)
                SpiWriteReg(CONFIG_FSCAL2, s + 32);

            if (paTableNumber != 1)
                setPA(usrPwrLvlDb);
        }
    }
    else if (gMHz >= 378 && gMHz <= 464)
    {
        retOffset = REMAP(freqHz, Band_378_464.left.freq, Band_378_464.left.cal, Band_378_464.right.freq, Band_378_464.right.cal);

		if (!bSilent) Serial.printf(FG_GREEN "%s 378->464 a %d hz internal HW offset to %f -> %f \n" FG_DONE, __FUNCTION__, retOffset, gMHz, gMHz+ (float) retOffset/1000000. ); 
        
        /// DO NOT USE TOO COURSE SpiWriteReg(CONFIG_FSCTRL0, offset / hzPerStep);

        if (gMHz < 430.5)
        {
            SpiWriteReg(CONFIG_TEST0, 0x0B);
        }
        else
        {
            SpiWriteReg(CONFIG_TEST0, 0x09);
            int s = radio.SpiReadReg(CONFIG_FSCAL2);

            if (s < 32)
                SpiWriteReg(CONFIG_FSCAL2, s + 32);

            if (paTableNumber != 2)
                setPA(usrPwrLvlDb);
        }
    }
    else if (gMHz >= 779 && gMHz < 900)
    {
    
		retOffset = REMAP(freqHz, Band_779_899.left.freq, Band_779_899.left.cal, Band_779_899.right.freq, Band_779_899.right.cal);
 		if (!bSilent) Serial.printf(FG_GREEN "%s 779->899 a %d hz internal HW offset to %f -> %f \n" FG_DONE, __FUNCTION__, retOffset, gMHz, gMHz+ (float) retOffset/1000000. ); 
		
		/// DO NOT USE TOO COURSE SpiWriteReg(CONFIG_FSCTRL0, offset / hzPerStep);
	
        if (gMHz < 861)
        {
            SpiWriteReg(CONFIG_TEST0, 0x0B);
        }
        else
        {
            SpiWriteReg(CONFIG_TEST0, 0x09);
            int s = radio.SpiReadReg(CONFIG_FSCAL2);

            if (s < 32)
                SpiWriteReg(CONFIG_FSCAL2, s + 32);

            if (paTableNumber != 3)
                setPA(usrPwrLvlDb);
        }
    }
    else if (gMHz >= 900 && gMHz <= 932)  //opened up a bit from 928
    {
		retOffset = REMAP(freqHz, Band_900_928.left.freq, Band_900_928.left.cal, Band_900_928.right.freq, Band_900_928.right.cal);
 		if (!bSilent) 
		{
			Serial.printf(FG_GREEN "%s 900->932 a %d hz internal HW offset to %f -> %f \n" FG_DONE, __FUNCTION__, retOffset, gMHz, gMHz+ (float) retOffset/1000000. ); 
			////Serial.printf("note: %d %d\n", offset / hzPerStep,  (uint8_t)( retOffset / hzPerStep));
		}	
		
		/// DO NOT USE TOO COURSE SpiWriteReg(CONFIG_FSCTRL0, (uint8_t)(retOffset / hzPerStep));
		
        SpiWriteReg(CONFIG_TEST0, 0x09);
        int s = radio.SpiReadReg(CONFIG_FSCAL2);

        if (s < 32)
            SpiWriteReg(CONFIG_FSCAL2, s + 32);

        if (paTableNumber != 4)
            setPA(usrPwrLvlDb);
    }
	else
	{
		Serial.printf(FG_RED "%s:%d *************** cant handle freq %f\n", __FUNCTION__, __LINE__, gMHz);
		delay(5000);
	}

	return retOffset;
}


/****************************************************************
* FUNCTION NAME:Calibration offset
* FUNCTION     :Set calibration offset
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setCalibrationOffset(byte b, int32_t left, int32_t  right)
{
    if (b == 1)
    {
        Band_300_348.left.cal = left;
        Band_300_348.right.cal = right;
    }
    else if (b == 2)
    {
        Band_378_464.left.cal = left;
        Band_378_464.right.cal = right;
    }
    else if (b == 3)
    {
        Band_779_899.left.cal = left;
        Band_779_899.right.cal = right;
    }
    else if (b == 4)
    {
        Band_900_928.left.cal = left;
        Band_900_928.right.cal = right;
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
	
    uint8_t foo = SpiReadStatus(STATUS_VERSION);
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
	Serial.printf(FG_MAGENTA "\n%s: set sync word 0x%8X\n" FG_DONE, __FUNCTION__, (sh << 8) + sl);
    SpiWriteReg(CONFIG_SYNC1, sh);
    SpiWriteReg(CONFIG_SYNC0, sl);
}


/****************************************************************
* FUNCTION NAME:Set ADDR
* FUNCTION     :Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setAddr(byte v)
{
	Serial.printf(FG_MAGENTA "\n%s: set network addr = %d\n" FG_DONE, __FUNCTION__, v);
    SpiWriteReg(CONFIG_ADDR, v);
}


/****************************************************************
* FUNCTION NAME:Set PQT
* FUNCTION     :Preamble quality estimator threshold
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPQT(byte v)
{
	Serial.printf(FG_MAGENTA "\n%s: setting preamble quality = %d\n" FG_DONE, __FUNCTION__, v);
	setField(CONFIG_PKTCTRL1,v, 7, 5);
}


/****************************************************************
* FUNCTION NAME:Set CRC_AUTOFLUSH
* FUNCTION     :Enable automatic flush of RX FIFO when CRC is not OK
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setCRC_AF(bool v)
{
	Serial.printf(FG_MAGENTA "\n%s: auto flush is %s\n" FG_DONE, __FUNCTION__, v ? "ENABLED":"DISABLED");
	setField(CONFIG_PKTCTRL1,v, 3, 3);
}


/****************************************************************
* FUNCTION NAME:Set APPEND_STATUS
* FUNCTION     :When enabled, two status bytes will be appended to the payload of the packet
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setAppendStatus(bool v)
{
	Serial.printf(FG_MAGENTA "\n%s: %s\n" FG_DONE, __FUNCTION__, v ? "ON":"OFF");
    setField(CONFIG_PKTCTRL1, v, 2, 2);
}


/****************************************************************
* FUNCTION NAME:Set ADR_CHK
* FUNCTION     :Controls address check configuration of received packages
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setAdrChk(byte v)
{
	const char *msg[] = 
	{
		"(00)No address check",
		"(01)Address check, no broadcast",
		"(10)Address check and 0 (0x00) broadcast",
		"(11)Address check and 0 (0x00) and 255 (0xFF)"
	};
	
   if (v > 3) v = 3;

   Serial.printf(FG_BMAGENTA "\n%s %s\n" FG_DONE, __FUNCTION__, msg[v]);
   
   setField(CONFIG_PKTCTRL1, v, 1, 0);

}


/****************************************************************
* FUNCTION NAME:Set WHITE_DATA
* FUNCTION     :Turn data whitening on / off.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setWhiteData(bool v)
{
	setField(CONFIG_PKTCTRL0, v, 6,6);
}


/****************************************************************
* FUNCTION NAME:Set PKT_FORMAT
* FUNCTION     :Format of RX and TX data
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPktFormat(byte v)
{

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
	Serial.print(FG_DONE);
	
	setField(CONFIG_PKTCTRL0, v , 5, 4);
	
}


/****************************************************************
* FUNCTION NAME:Set CRC
* FUNCTION     :CRC calculation in TX and CRC check in RX
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setCrc(bool v)
{
	Serial.printf(FG_MAGENTA "\n%s is %s\n" FG_DONE, __FUNCTION__, v ? "ENABLED" : "DISABLED");
	setField(CONFIG_PKTCTRL0, v , 2, 2);
}



void ELECHOUSE_CC1101::setLnaStrategy(bool bType1)
{
	bool old = getField(CONFIG_AGCCTRL1, 6, 6);
	Serial.printf(FG_MAGENTA "\n%s WAS %s drops first then %s\n" FG_DONE, __FUNCTION__, old ? "LNA-1":"LNA-2", old ?"LNA-2":"LNA-1");

	Serial.printf(FG_MAGENTA "%s NOW %s drops first then %s\n" FG_DONE, __FUNCTION__, old ? "LNA-1":"LNA-2", old ?"LNA-2":"LNA-1");

	setField(CONFIG_AGCCTRL1, bType1, 6, 6);
}


void ELECHOUSE_CC1101::setCarrierSenseAbs(int8_t vDb)
{
	int8_t oldReg = getField(CONFIG_AGCCTRL1, 3, 0);


	Serial.printf(FG_MAGENTA "\n%s WAS %d db around MAGN_TARGET\n" FG_DONE, __FUNCTION__, oldReg);
	
	if (vDb > -8 && vDb < 8)
		Serial.printf(FG_MAGENTA "%s NOW is %d db around MAGN_TARGET\n" FG_DONE, __FUNCTION__, vDb);
	else
	{	
		vDb = -8;
		Serial.printf(FG_MAGENTA "%s NOW is DISABLED (db < -7 || db > +7) \n" FG_DONE, __FUNCTION__);
	}	
	setField(CONFIG_AGCCTRL1, vDb, 3, 0);
}


void ELECHOUSE_CC1101::setCarrierSenseRel(int8_t vDb)
{
	int8_t reg, oldReg;
	int8_t oldDb, newDb;
	
	if (vDb > 13 )
	{
		reg = 3;
		newDb = 14;
	}
	else if (vDb> 9)
	{
		reg = 2;
		newDb = 10;
	}
	else if (vDb > 5)
	{
		reg = 1;
		newDb = 6;
	}
	else 
	{	
		reg = 0;
		newDb = 0;
	}


	
	oldReg = getField(CONFIG_AGCCTRL1, 5, 4);
	if (!oldReg) 
		Serial.printf(FG_MAGENTA "\n%s WAS DISABLED\n" FG_DONE, __FUNCTION__);
	else
		Serial.printf(FG_MAGENTA "%s WAS +%s db from RSSI floor\n" FG_DONE, __FUNCTION__, 
					!oldReg ? "DISABLED" : oldReg < 2 ? "6" : oldReg < 3 ? "10" : "14");

	
	Serial.printf(FG_MAGENTA "%s NOW %d+ db from RSSI floor\n" FG_DONE, __FUNCTION__, newDb);

	setField(CONFIG_AGCCTRL1, reg , 5, 4);
}

uint8_t ELECHOUSE_CC1101::setMAGNTarget(uint8_t vDb)
{
	if (vDb > 42) vDb = 42;

	uint8_t v;
	uint8_t oldv, oldDb;
	
	if (vDb < 27) 
		v = 0;
	else if (vDb < 30)
		v = 1;
	else if (vDb < 33)
		v = 2;
	else if (vDb < 36)
		v = 3;
	else if (vDb < 38)
		v = 4;
	else if (vDb < 40)
		v = 5;
	else if (vDb < 42)
		v = 6;
	else
		v = 7;

	oldv = getField(CONFIG_AGCCTRL2, 2, 0);
	if (oldv < 1)
		oldDb = 24;
	else if (oldv < 2)
		oldDb = 27;
	else if (oldv < 3)
		oldDb = 30;
	else if (oldv < 4)
		oldDb = 33;
	else if (oldv < 5)
		oldDb = 36;
	else if (oldv < 6)
		oldDb = 38;
	else if (oldv < 7)
		oldDb = 40;
	else 
		oldDb = 42;

	Serial.printf(FG_MAGENTA "\n%s is set to %d dB (old = %d db)\n" FG_DONE, __FUNCTION__, vDb, oldDb);

	
	setField(CONFIG_AGCCTRL2, v , 2, 0);
	return oldDb;
	
}



/****************************************************************
* FUNCTION NAME:Set LENGTH_CONFIG
* FUNCTION     :Configure the packet length
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setLengthConfig(byte v)
{

	Serial.println(FG_BMAGENTA);

	// only infinite packet length is allowed to have a '0' size.	
	uint8_t length = SpiReadReg(CONFIG_PKTLEN);
	Serial.printf("wwwwwwwwwwwwwwwwwwww %d \n", length);
	Serial.flush();
	
    switch(v)
    {
    	case 0:
    		Serial.printf("%s: (00)Fixed packet length mode.\n\tLength configured in PKTLEN register\n", __FUNCTION__);
			assert(length != 0);    		
    	break;

    	case 1:
    		Serial.printf("%s: (01)Variable packet length mode.\n\tPacket length configured by the first byte after sync word\n", __FUNCTION__);
			assert(length != 0);    		
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
	
	Serial.print(FG_DONE);
	
	if (v > 3) v = 3;
	setField(CONFIG_PKTCTRL0, v, 1, 0);


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
    			  "\tIf INFINITE packet length is enabled this may be 0\n" FG_DONE);

    SpiWriteReg(CONFIG_PKTLEN, v);
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
	
	Serial.printf(FG_MAGENTA "%s : fifo warn RX @ %d or TX @ %d\n" FG_DONE, __FUNCTION__, rx_lvl[i], tx_lvl[i]);

    SpiWriteReg(CONFIG_FIFOTHR, i);

    Serial.printf(FG_FRED " need to set GD0x if used\n" FG_DONE);
    
    //SpiWriteReg(CONFIG_IOCFG0, 2);  // GD00 signal on tx low
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

    SpiWriteReg(CONFIG_MDMCFG2, m2DCOFF + m2MODFM + m2MANCH + m2SYNCM);
#else
    setField(CONFIG_MDMCFG2, v, 7, 7);
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

    SpiWriteReg(CONFIG_MDMCFG2, m2DCOFF + m2MODFM + m2MANCH + m2SYNCM);
#else
    setField(CONFIG_MDMCFG2,v, 3, 3);
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
    SpiWriteReg(CONFIG_MDMCFG2, m2DCOFF + m2MODFM + m2MANCH + m2SYNCM);
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
   
   Serial.printf(FG_MAGENTA "\n%s mode = %s\n" FG_DONE, __FUNCTION__, msg[v]);
   setField(CONFIG_MDMCFG2, v , 2, 0);
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

    SpiWriteReg(CONFIG_MDMCFG1, m1FEC + m1PRE + m1CHSP);
#else
	Serial.printf(FG_MAGENTA "\n%s: %s\n" FG_DONE, __FUNCTION__, v ? "ON":"OFF");
	
	setField(CONFIG_MDMCFG1, v,7,7);
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
    SpiWriteReg(CONFIG_MDMCFG1, m1FEC + m1PRE + m1CHSP);
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
	
	Serial.printf(FG_MAGENTA "\n%s: %s bytes\n" FG_DONE, __FUNCTION__, msg[v]);
	setField(CONFIG_MDMCFG1, v,6 ,4);
	
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
    Serial.printf(FG_MAGENTA "%s: logical chan=%d\n" FG_DONE, __FUNCTION__, ch);
    SpiWriteReg(CONFIG_CHANNR, logical_chan);
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
	
	Serial.printf("\tlock Mant=%d Exp=%d\n" FG_DONE, lockMantissa, lockExp);
	
    setField(CONFIG_MDMCFG1, lockExp, 1, 0);
    setField(CONFIG_MDMCFG0, lockMantissa, 7, 0);
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

	Serial.printf(FG_MAGENTA "\n%s: setting rx bw = %5.2f khz\n" FG_DONE, __FUNCTION__, rxBw);

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

	setField(CONFIG_MDMCFG4, lockExp, 7, 6);
	setField(CONFIG_MDMCFG4, lockMantissa, 5, 4);
#endif
}


/****************************************************************
* FUNCTION NAME:Set Data Rate
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setBaudRate(uint32_t bps)
{
	int16_t exp;
	double mantissa;
	int32_t iTest;

	int16_t lockExp = -1;
	int16_t lockMantissa = -1;
	
	Serial.printf(FG_MAGENTA "\n%s: setting data rate = %d bps\n" FG_BCYAN, __FUNCTION__, bps);
	
	double dRate = bps;
	double FIXED = dRate * (double)(1 << 28)/ (double)(XTAL_Mhz * 1.e6 );
	
	for (exp = 16; exp > -1; exp--)  // exp reg is 4 bits.
	{
		double expTest = (float)(1 << exp);
		double mantissa = ((FIXED - 256.0 * expTest)) /expTest;
		iTest = mantissa;
		
		//Serial.printf("\t\texp=%d  mant=%d\n", exp, (int)mantissa);

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
					
	Serial.printf(FG_BGREEN "\t\tlock Mant=%d Exp=%d Result=%f\n" FG_DONE, 
			lockMantissa, lockExp, resultHz);
	
    setField(CONFIG_MDMCFG4, lockExp, 3, 0);
    setField(CONFIG_MDMCFG3, lockMantissa, 7, 0);

}



/****************************************************************
* FUNCTION NAME:Set setSymbolSpacingHz
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setSymbolSpacingHz(float HzBetweenSymbol)
{
	int16_t exp;
	float mantissa;
	int32_t iMant;

	int16_t lockExp = -1;
	int16_t lockMantissa = -1;

	Serial.printf(FG_MAGENTA "\n%s: spacing between symbols = %5.2f hz\n" FG_DONE, __FUNCTION__, HzBetweenSymbol);
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
	setField(CONFIG_DEVIATN, lockMantissa, 2, 0);
	setField(CONFIG_DEVIATN, lockExp, 6, 4);


	//lockMantissa = 1;
	//lockExp = 1;       // 1785 pull hi or low

	// show what the results would be by changing the mantissa by -1,0,+1
	
	for (int lockM = lockMantissa -1; lockM < lockMantissa+2; lockM++)
	{
		float result = XTAL_Hz 
						* (8. + lockM) * (float) (1<< lockExp)
						/(float)(2<<17);
		
		Serial.printf("\tlock Mant=%d Exp=%d final= +/- %f\n", lockM, lockExp, result);
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
	int16_t exp;
	float mantissa;
	int32_t iMant;

	int16_t lockExp = -1;
	int16_t lockMantissa = -1;

	Serial.printf(FG_MAGENTA "\n%s: setting deviation (max pull left or right) = %5.2f khz\n" FG_DONE, __FUNCTION__, fdev);

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
	setField(CONFIG_DEVIATN, lockMantissa, 2, 0);
	setField(CONFIG_DEVIATN, lockExp, 6, 4);


	//lockMantissa = 1;
	//lockExp = 1;       // 1785 pull hi or low

	for (int lockM = lockMantissa -1; lockM < lockMantissa+2; lockM++)
	{
		float result = XTAL_Hz 	* (8. + lockM) * (float) (1<< lockExp) /(float)(2<<17);
		Serial.printf("\tlock Mant=%d Exp=%d final= +/- %f\n", lockM, lockExp, result);
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
    SpiWriteReg(CONFIG_FSCTRL1, 0x06);

    setCCMode(ccmode);
    setMHZ(gMHz);

    SpiWriteReg(CONFIG_MDMCFG1, 0x02);
    SpiWriteReg(CONFIG_MDMCFG0, 0xF8);
    
    //SpiWriteReg(CONFIG_CHANNR, logical_chan);
    //SpiWriteReg(CONFIG_DEVIATN, 0x47);
    
    SpiWriteReg(CONFIG_FREND1, 0x56);
    SpiWriteReg(CONFIG_MCSM0, 0x18);
    SpiWriteReg(CONFIG_FOCCFG, 0x16);
    SpiWriteReg(CONFIG_BSCFG, 0x1C);
    
    SpiWriteReg(CONFIG_AGCCTRL2, 0xC7);
    SpiWriteReg(CONFIG_AGCCTRL1, 0x00);
    SpiWriteReg(CONFIG_AGCCTRL0, 0xB2);
    
    SpiWriteReg(CONFIG_FSCAL3, 0xE9);
    SpiWriteReg(CONFIG_FSCAL2, 0x2A);
    SpiWriteReg(CONFIG_FSCAL1, 0x00);
    SpiWriteReg(CONFIG_FSCAL0, 0x1F);

    SpiWriteReg(CONFIG_FSTEST, 0x59);

    SpiWriteReg(CONFIG_TEST2, 0x81);
    SpiWriteReg(CONFIG_TEST1, 0x35);
    SpiWriteReg(CONFIG_TEST0, 0x09);

    SpiWriteReg(CONFIG_PKTCTRL1, 0x04);
    SpiWriteReg(CONFIG_ADDR, 0x00);
    SpiWriteReg(CONFIG_PKTLEN, 0x00);
}


/****************************************************************
* FUNCTION NAME:EnterTxMode
* FUNCTION     :set CC1101 send data
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::EnterTxMode(void)
{
    Serial.printf(FG_FYELLOW "%s: TX MODE !!!! \n" FG_DONE, __FUNCTION__);
    SpiStrobe(CC1101_SIDLE);
    setMHZ(gMHz);
    
    SpiStrobe(CC1101_STX);      //start send
    
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
    Serial.printf(FG_FYELLOW "%s: RX MODE !!!! \n" FG_DONE, __FUNCTION__);
    
    SpiStrobe(CC1101_SIDLE);
    SpiStrobe(CC1101_SRX);      //start receive
    
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
    Serial.printf(FG_FYELLOW "%s: RX MODE @ %s !!!! \n" FG_DONE, __FUNCTION__, mhz);
    SpiStrobe(CC1101_SIDLE);
    setMHZ(mhz);
    SpiStrobe(CC1101_SRX);      //start receive
    
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

    rssi = SpiReadStatus(STATUS_RSSI);

    if (rssi >= 128)
        rssi = (rssi - 256) / 2 - 74;
    else
        rssi = (rssi / 2) - 74;

    return rssi;
}



/****************************************************************
* FUNCTION NAME:getPktStatus Level
* INPUT        :none
* OUTPUT       :none
****************************************************************/
uint8_t bCarrierSense;
uint8_t bSyncNpacket;
uint8_t bPQTpass;
uint8_t bCCA;
uint8_t bGDO2;
uint8_t bGDO0;

int ELECHOUSE_CC1101::getPktStatus(void)
{
	static int16_t last = -1;
	static uint32_t lastTime;

	uint32_t now = millis();
	uint32_t delta = now - lastTime;
	lastTime = now;
	
	uint8_t orig= SpiReadStatus(STATUS_PKTSTATUS);
	
	bCarrierSense 	= regMaskRead <uint8_t> ( orig, 6, 6);
	bPQTpass 		= regMaskRead <uint8_t> ( orig, 5, 5);
	bCCA 			= regMaskRead <uint8_t> ( orig, 4, 4);
	bSyncNpacket 	= regMaskRead <uint8_t> ( orig, 3, 3);
	bGDO2 			= regMaskRead <uint8_t> ( orig, 2, 2);
	bGDO0 			= regMaskRead <uint8_t> ( orig, 0, 0);

	if (last != orig)
	{
		last = orig;
		Serial.printf("T=%10d CarrierSense=%d PreambleQuality=%d ClearChannelAssmt=%d SyncOrPakt=%d RSSI=%3d\n", 
				delta, bCarrierSense, bPQTpass, bCCA, bSyncNpacket, getRssi());
	}

	static int lastRssi;
	int rssi = getRssi();
	if (rssi > lastRssi )
	{
		lastRssi = rssi + 10;
		Serial.printf("T=%10d CarrierSense=%d PreambleQuality=%d ClearChannelAssmt=%d SyncOrPakt=%d RSSI=%3d\n", 
				delta, bCarrierSense, bPQTpass, bCCA, bSyncNpacket, rssi);
	}

    return orig;
}


void ELECHOUSE_CC1101::setCCAmode(uint8_t type)
{
	uint8_t oldCCA = getField(CONFIG_MCSM1, 5, 4);

	static const char *lcl[] = {
		"Always",
		"If RSSI below threshold",
		"Unless currently receiving a packet",
		"If RSSI below threshold unless currently receiving a packet"
	};

	assert (type < 4);
	Serial.printf(FG_GREEN "\n%s: OLD [%d] = %s\n", __FUNCTION__, oldCCA, lcl[oldCCA]);
	Serial.printf("%s: NEW [%d] = %s\n" FG_DONE, __FUNCTION__, type, lcl[type]);

	setField(CONFIG_MCSM1,type,5, 4);
}

void ELECHOUSE_CC1101::setRxOffMode(uint8_t type)
{
	uint8_t oldRx = getField(CONFIG_MCSM1, 3, 2);

	static const char *lcl[] = {
						"IDLE",
						"FSTXON",
						"TX",
						"Stay in RX"
	};

	assert (type < 4);
	Serial.printf(FG_GREEN "\n%s: OLD [%d] = %s\n", __FUNCTION__, oldRx, lcl[oldRx]);
	Serial.printf("%s: NEW [%d] = %s\n" FG_DONE, __FUNCTION__, type, lcl[type]);

	setField(CONFIG_MCSM1,type, 3, 2);
}

void ELECHOUSE_CC1101::setTxOffMode(uint8_t type)
{
	uint8_t oldTx = getField(CONFIG_MCSM1, 1, 0);

	static const char *lcl[] = {
						"IDLE",
						"FSTXON",
						"Stay in TX + start preamble",
						"RX"
	};

	assert (type < 4);
	Serial.printf(FG_GREEN "\n%s: OLD [%d] = %s\n", __FUNCTION__, oldTx, lcl[oldTx]);
	Serial.printf("%s: NEW [%d] = %s\n" FG_DONE, __FUNCTION__, type, lcl[type]);

	setField(CONFIG_MCSM1,type, 1, 0);
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

    lqi = SpiReadStatus(STATUS_LQI);
    return lqi;
}

typedef struct PAIR
{
	bool bWait2exit;
    char *left;
    char *right;
};

byte ELECHOUSE_CC1101::getState(bool bSilent)
{
	byte status;
	static const PAIR msg[] = 
	{
		{ 1, "SLEEP",			"SLEEP"			},
		{ 0, "IDLE",	 		"IDLE"		},
		{ 1, "XOFF",	 		"XOFF"		},
		{ 1, "VCOON",			"MANCAL"		},
		{ 1, "REGON",			"MANCAL"		},
		{ 1, "MANCAL",			"MANCAL"		},
		{ 1, "VCOONFS",			"_WAKEUP"		},
		{ 1, "REGONFS_",		"WAKEUP"	},
		{ 1, "STARTCAL",		"CALIBRATE"	},
		{ 1, "BWBOOST",			"SETTLING"		},
		{ 1, "FS_LOCK",			"SETTLING"		},
		{ 1, "IFADCON",			"SETTLING"		},
		{ 1, "ENDCAL",			"CALIBRATE"		},
		{ 0, "RX",	 			"RX"			},
		{ 1, "RX_END",			"RX"			},
		{ 1, "RX_RST",			"RX"			},
		{ 1, "TXRX_SWITCH",	 	"TXRX_SETTLING"		},
		{ 1, "RXFIFO_OVERFLOW",	"RXFIFO_OVERFLOW"	},
		{ 1, "FSTXON",	 		"FSTXON"			},
		{ 0, "TX",	 			"TX"				},
		{ 1, "TX_END",	 		"TX"				},
		{ 1, "RXTX_SWITCH",	 	"RXTX_SETTLING"		},
		{ 1, "TXFIFO_UNDERFLOW", "TXFIFO_UNDERFLOW"	},
	};
    
    uint8_t elem = sizeof(msg)/ sizeof(msg[0]);

    while(true)
    {
   		status = SpiReadStatus(STATE_MARCSTATE);
	    if ( status < elem)
	    {
			if(!bSilent) Serial.printf(FG_GREEN "%s:  %d = %s\n", __FUNCTION__, status, msg[ status].right);
		}
		else
		{
			if(!bSilent) Serial.printf(FG_GREEN "%s:  unknown %d\n", __FUNCTION__, status);
			break;
		}
		if (!msg[status].bWait2exit) break;
	}    
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
    Serial.println("****** chip h/w reset ***\n"); delay(5000);
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
    
    Serial.printf(FG_FYELLOW "%s: IDLE !!!! \n" FG_DONE, __FUNCTION__);
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
    
    Serial.printf(FG_FYELLOW "%s: SLEEP !!!! \n" FG_DONE, __FUNCTION__);
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

/*
	static bool bDebugWTF = false;
	if (bDebugWTF)
	{
		radio.snapshot2();
    	radio.diffSnapshots();
	}
	else
	{
		radio.snapshot1();
	}
	bDebugWTF = true;
*/

	// first byte in to tx is the size!
    _SpiWriteReg("CONFIG_FIFO", CONFIG_FIFO, size, true);

	// all following bytes are sent off.
    SpiWriteBurstReg(CONFIG_FIFO, txBuffer, size);    //write data to send

    SpiStrobe(CC1101_SIDLE);
    SpiStrobe(CC1101_STX);      //start send

	uint8_t test = SpiReadReg(CONFIG_IOCFG0); 	// is GDO0 in the correct mode?
	assert (test == 6);


	// can't get out of here ??? you didn't power up the MBUS dumbass
    while (!digitalRead(GDO0)); // -> sync transmitted
    while ( digitalRead(GDO0)); // -> end of packet

    ///// SpiStrobe(CC1101_SFTX);                 //flush TXfifo
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

    SpiWriteReg(CONFIG_FIFO, size);
    SpiWriteBurstReg(CONFIG_FIFO, txBuffer, size);    //write data to send
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
    byte lqi = SpiReadStatus(STATUS_LQI);
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

    if (SpiReadStatus(STATUS_RXBYTES) & BYTES_IN_RXFIFO)
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

    if (SpiReadStatus(STATUS_RXBYTES) & BYTES_IN_RXFIFO)
    {
        size = SpiReadReg(CONFIG_FIFO);
        SpiReadBurstReg(CONFIG_FIFO, rxBuffer, size);
        SpiReadBurstReg(CONFIG_FIFO, status, 2);
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

ELECHOUSE_CC1101 radio;

