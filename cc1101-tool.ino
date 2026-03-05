//
// CC1101 interactive terminal tool
// allows for sending / receiving data over serial port
// on selected radio channel, modulation, ..
//
// (C) Adam Loboda '2023 , adam.loboda@wp.pl
//
// based on great SmartRC library by Little_S@tan
// Please download ZIP from
// https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
// and attach it as ZIP library for Arduino
//
// Also uses Arduino Command Line interpreter by Edgar Bonet
// from https://gist.github.com/edgar-bonet/607b387260388be77e96
//
// This code will ONLY work with ESP32 board
//


// fix for m5core gpio not defined
//#include "soc/gpio_struct.h"
//#include "hal/gpio_ll.h"
//----


#include <M5Unified.h>
#include <Wire.h>
#include "built_on.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include "esp_intr_types.h"
#include "keyboard.h"
#include "bandcal.h"
#include "layer1.h"

const char *ssid = MY_SSID;
const char *password = MY_SSID_PASSWORD;



#define LINE Serial.printf("%s:%d %s\n", __FILE__, __LINE__, __FUNCTION__)

#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <EEPROM.h>
#include <SPI.h>

#define RECORDINGBUFFERSIZE 4096    // Buffer for recording the frames
#define EPROMSIZE 512               // Size of EEPROM in your Arduino chip. For ESP32 it is Flash simulated so very slow
#define BUF_LENGTH 128              // Buffer for the incoming command.

#define DEFAULT_TxFREQ  928  //867.010  //905. //866.9375     //866.8875 	//866.9625   // 905

#define DEFAULT_RxFREQ 867.38751

#define DEFAULT_STEP   12500

#if defined (ARDUINO_M5STACK_CORE2)

byte PIN_MOSI = 23;
byte PIN_MISO = 38;
byte PIN_SCK = 18;

byte PIN_CS = 27;

int PIN_GDO2 = 19;
int PIN_GDO0 = 33;

#elif defined (ARDUINO_M5STACK_CORES3)
byte PIN_MOSI = 37;
byte PIN_MISO = 35;
byte PIN_SCK = 36;

byte PIN_CS = 5;

int PIN_GDO2 = 10;
int PIN_GDO0 = 7;

#else
#error unknown processor
#endif


// position in big recording buffer
int bigrecordingbufferpos = 0;

// number of frames in big recording buffer
int framesinbigrecordingbuffer = 0;

// check if CLI receiving mode enabled
int receivingmode = 0;

// check if CLI jamming mode enabled
int jammingmode = 0;

// check if CLI recording mode enabled
int recordingmode = 0;

// check if CLI chat mode enabled
int chatmode = 0;

static bool do_echo = true;

// buffer for receiving  CC1101
byte ccreceivingbuffer[CC_FIFOSIZE] = { 0 };

// buffer for sending  CC1101
byte ccsendingbuffer[CC_FIFOSIZE * 2] = { 0 };
//char ccsendingbuffer[CCBUFFERSIZE] = {0};

// buffer for recording and replaying of many frames
byte bigrecordingbuffer[RECORDINGBUFFERSIZE] = { 0 };

// buffer for hex to ascii conversions
char textBuffer[RECORDINGBUFFERSIZE * 2 + 1];

#define RANDO_LENGTH 55

//char hexBuffer[BUF_LENGTH];
// convert bytes in table to string with hex numbers

uint8_t * makeRandomTxBuffer(uint8_t len)
{
	static uint8_t packCtr ;
	static uint8_t TX_BUFFER[CC_FIFOSIZE];
	
	len = min(len, (uint8_t) CC_FIFOSIZE);
	
	for (int i= 0; i < len; i++) TX_BUFFER[i] = random(255);
	
	TX_BUFFER[0]= len;
	sprintf( (char *) &TX_BUFFER[1],"%03d:", packCtr++);

	return TX_BUFFER;
}


void binToAscii(byte *asciiIn, char *hexOut, int len)
{

	int i;
	for (i = 0; i < len; i++)
	{
		sprintf(&hexOut[i * 2], "%02X", asciiIn[i]);
	}
	hexOut[i] = 0;  // overrun if not overallocated.
}


// convert string with hex numbers to array of bytes
int  hextoascii(char *pAsciiOut, byte *pHexIn, int len)
{
    byte i, j;

    for (i = 0; i < (len / 2); i++)
    {
        j = pHexIn[i * 2];

        if ((j > 47) && (j < 58))
            pAsciiOut[i] = (j - 48) * 16;

        if ((j > 64) && (j < 71))
            pAsciiOut[i] = (j - 55) * 16;

        if ((j > 96) && (j < 103))
            pAsciiOut[i] = (j - 87) * 16;

        j = pHexIn[i * 2 + 1];

        if ((j > 47) && (j < 58))
            pAsciiOut[i] = pAsciiOut[i] + (j - 48);

        if ((j > 64) && (j < 71))
            pAsciiOut[i] = pAsciiOut[i] + (j - 55);

        if ((j > 96) && (j < 103))
            pAsciiOut[i] = pAsciiOut[i] + (j - 87);
    }

    
    pAsciiOut[i++] = '\0';
    return i;
}

void doesNothing(void)
{
	return;
}

// Initialize CC1101 board with default settings, you may change your preferences here
static void cc1101initialize(void)
{

    // initializing library with custom pins selected
    radio.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    radio.setGDOx(PIN_GDO0, PIN_GDO2);


    // Main part to tune CC1101 with proper frequency, modulation and encoding
    radio.Init();                // must be set to initialize the cc1101!
    radio.defineGDO0_pinNum(PIN_GDO0);         // set lib internal gdo pin (gdo0). Gdo2 not use for this example.
    radio.setCCMode(GDO0_isSYNC_TXEND);          // set config for internal transmission mode. value 0 is for RAW recording/replaying

    radio.setModulation(DEFAULT_MODULATION);      	// set modulation mode. 
    											//	0 = 2-FSK, 
    											//	1 = GFSK, 
    											//	2 = ASK/OOK, 
    											//	3 = 4-FSK, 
    											//	4 = MSK.
    											
    radio.setMHZ(DEFAULT_TxFREQ);  	// Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
    radio.setSymbolSpacingHz(1200);  // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
    
    radio.setLogicalChanNum(0);         	// Set the Channelnumber from 0 to 255. Default is cahnnel 0.

    radio.setChannelSpacing(25.39);    // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
    radio.setRxBW(58.3);       	// Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
    
    radio.setBaudRate(DEFAULT_BAUD);   // Set the Data Rate in Baud. 
    										// Value from 200 to 1,621,830. 
    										//Default is 99970 Baud!
    
    radio.setPA(0);             // Set TxPower. 
    										// The following settings are possible depending on the frequency band.
    										// (-30  -20  -15  -10  -6    0    5    7    10   11   12) 
    										// Default is max!

	// RSSI locks if sync is found
    radio.setSyncMode(0);        // Combined sync-word qualifier mode. 
											// 0 = No preamble/sync. 
											// 1 = 16 sync word bits detected. 
											// 2 = 16/16 sync word bits detected. 
											// 3 = 30/32 sync word bits detected. 
											// 4 = No preamble/sync, carrier-sense above threshold. 
											// 5 = 15/16 + carrier-sense above threshold. 
											// 6 = 16/16 + carrier-sense above threshold. 
											// 7 = 30/32 + carrier-sense above threshold.
    												
    radio.setSyncWord(0x55, 0x75); 	// Set sync word. Must be the same for the transmitter and receiver. 
    												//Default is 211,145 (Syncword high, Syncword low)
    												
    radio.setAdrChk(0);          	// Controls address check configuration of received packages. 
    												// 0 = No address check. 
    												// 1 = Address check, no broadcast. 
    												// 2 = Address check and 0 (0x00) broadcast. 
    												// 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
    												
    radio.setAddr(0);            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).

    radio.setWhiteData(0);       // Turn data whitening on / off. 
    											// 0 = Whitening off. 
    											// 1 = Whitening on.
    											
    radio.setPktFormat(0);   // Format of RX and TX data. 
										// 0 = Normal mode, use FIFOs for RX and TX. 
										// 1 = Synchronous serial mode, 
										//			Data in on GDO0 and data out on either of the GDOx pins. 
										// 2 = Random TX mode; sends random data using PN9 generator. 
										//		Used for test. Works as normal mode, setting 0 (00), 
										//		in RX. 3 = Asynchronous serial mode, 
										//		Data in on GDO0 and data out on either of the GDOx pins.
    											
	radio.setPacketLength(CC_FIFOSIZE);
												// If FIXED packet length mode is enabled. 
												// If VARIABLE packet this value indicates the maximum packet length allowed.
												// If INFINITE packet format may allow this to be zero
	
	
    radio.setLengthConfig(1);    //  0 = Fixed packet length mode. 
    										//	1 = Variable packet length mode. 
    										//	2 = Infinite packet length mode. 
    										//  3 = Reserved
    										
    radio.setCrc(0);             // 1 = CRC calculation in TX and CRC check in RX enabled. 
    										// 0 = CRC disabled for TX and RX.
    										
    radio.setCRC_AF(0);          // Enable automatic flush of RX FIFO when CRC is not OK. 
    										// 		This requires that only one packet is in the RXIFIFO 
    										//		and that packet length is limited to the RX FIFO size.
    										
    radio.setDcFilterOff(0);     // Disable digital DC blocking filter before demodulator. 
    										// 	Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled.
    										//	1 = Disable (current optimized). 
    										//	0 = Enable (better sensitivity).
    										
    radio.setManchester(0);      // Enables Manchester encoding/decoding. 
    										//	0 = Disable. 
    										//  1 = Enable.

    radio.setFEC(0);             // Enable Forward Error Correction (FEC) with interleaving
    										// 		for packet payload (Only supported for fixed packet length mode. 
    										//		0 = Disable. 
    										//		1 = Enable.
    										
    radio.setNumPreambleBytes(7);             // Sets the minimum number of preamble bytes to be transmitted. 
    										//		Values: 0 : 2, 
    										//				1 : 3, 
    										//				2 : 4,
    										//				3 : 6, 
    										//				4 : 8, 
    										//				5 : 12, 
    										//				6 : 16, 
    										//				7 : 24
    										
    radio.setPQT(1);             // Preamble quality estimator threshold. 
    										// 		The preamble quality estimator increases an internal counter
    										//			by one each time a bit is received that is different from the previous bit, 
    										//			and decreases the counter by 8 each time a bit 
    										//			is received that is the same as the last bit. 
    										// A threshold of PQT for this counter is used to gate sync word detection. 
    										// When PQT=0 a sync word is always accepted.
    										
    radio.setAppendStatus(1);    // When enabled, two status bytes will be appended to the payload of the packet. 
    										// 	The status bytes contain RSSI and LQI values,
    										//	as well as CRC OK.

    										
											
	radio.setLnaStrategy(1);
	radio.setCarrierSenseAbs(99);  // disabled (>7)
	radio.setCarrierSenseRel(9);
	radio.setMAGNTarget(33);
	
	radio.setCCAmode(0);
	radio.setRxOffMode(3);
	radio.setTxOffMode(0);
	
	Serial.println("==================================================");
	
}

//-----------------------------------------------------------------------

void txSendByFifos(void)
{
	byte *rando;  

	radio.EnterIdleMode();
#if 0
	radio.setModulation(DEFAULT_MODULATION); //4fsk
	radio.setBaudRate(4.8);
	radio.setDeviation(1.8);
	radio.setNumPreambleBytes (7);  // long preamble
#else
	radio.setMHZ(0); 				// refresh tx freq
	radio.setModulation(DEFAULT_MODULATION); 			//4fsk
	radio.setBaudRate(DEFAULT_BAUD);
	radio.setSymbolSpacingHz(1200);
	radio.setNumPreambleBytes (7);  	// long preamble
#endif

	radio.setCCMode(GDO0_isSYNC_TXEND);	//gdO = SYNC+Sent

	delay(1000);

	uint32_t pctr = 0;
	float freq = radio.getMHZ();

	
    while(!Serial.available())
    {
    	ArduinoOTA.handle();
    	pctr++;
		if (freq < 800 || pctr > 1) break;    /////////////////////////
    	
    	int j;
        Serial.printf("\r\nTransmitting RF packet %d\r\n", pctr);

		rando = makeRandomTxBuffer(RANDO_LENGTH);
		
    	// send these data to radio over CC1101
    	radio.SendBinaryData(rando, RANDO_LENGTH);


    	char abuf[RANDO_LENGTH * 2 + 1];
        Serial.print(F("Sent frame: "));
        for (j = 0; j < RANDO_LENGTH; j++)
        {
        	sprintf(&abuf[j*2], "%02X", rando[j]);
        }
        abuf[j] = 0;
        
        Serial.printf("%f %s\n", freq, abuf);

		int waitSec = 12;
    	while(waitSec--)
    	{
			if(Serial.available()) goto bye; 
    		delay(1000);
        }
		// for DEBUG only
	}
bye:
	Serial.read();

	radio.EnterIdleMode();
	radio.setBaudRate(DEFAULT_BAUD);
}
//-----------------------------------------------------------------------

// Execute a complete CC1101 command.
static void exec(char *input)
{
    char *cmd = strsep(&input, " ");
    Serial.printf("cmd = %s\n", cmd);
	char * cmd_args;
	
    char tcmd_args[20];
    
    char *tst_args = strsep(&input, " ");
    if (tst_args)
    	strcpy(tcmd_args, tst_args);
    else
    	tcmd_args[0] = '\0';

    cmd_args = &tcmd_args[0];
    
    Serial.printf("cmd_args = %s\n", cmd_args);
    
    int setting, setting2, len;
    uint16_t brute, poweroftwo;
    byte j, k;
    float nextParam;
    float endFreq;
    // variables for frequency scanner
    float freq;
    long compare_freq;
    float mark_freq;
    int rssi;
    int mark_rssi = -100;

    // identification of the command & actions

    if (strcmp_P(cmd, PSTR("help")) == 0)
    {
        Serial.println(F(
           "setmodulation <mode> : Set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.\r\n"
           "setmhz <frequency>   : Here you can set your basic frequency. default = 433.92).\n\tThe cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ.\r\n"
           "setdeviation <deviation> : Set the Frequency deviation in kHz. Value from 1.58 to 380.85.\r\n"
		    "\r\n"
           "setchannel <channel> : Set the Channelnumber from 0 to 255. Default is cahnnel 0.\r\n"
           "setchsp <spacing>  :  The channel spacing is multiplied by the channel number CHAN \n\tand added to the base frenquency in kHz.\n\tValue from 25.39 to 405.45. \r\n"
           "setrxbw <Receive bndwth> : Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. \r\n"
			"\r\n"
           "setdrate <datarate> : Set the Data Rate in kBaud. Value from 0.02 to 1621.83.\r\n"
           "setpa <power value> : Set RF transmission power.\n\tThe following settings are possible depending on the frequency band.\n\t(-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!\r\n"
           "setsyncmode  <sync mode> : Combined sync-word qualifier mode. 0 = No preamble/sync. \n\t1 = 16 sync word bits detected. \n\t2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.\r\n"
           ));
        Serial.println(F(
           "setsyncword <decimal LOW, decimal HIGH> : \n\tSet sync word. Must be the same for the transmitter and receiver. \n\t(Syncword high, Syncword low) Default is 211,145\r\n"
           "setadrchk <address chk> : Controls address check configuration of received packages. \n\t0 = No address check. \n\t1 = Address check, no broadcast. \n\t2 = Address check and 0 (0x00) broadcast. \n\t3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.\r\n"
           "setaddr <address> : Address used for packet filtration. \n\tOptional broadcast addresses are 0 (0x00) and 255 (0xFF).\r\n"
   		    "\r\n"
           "setwhitedata <whitening> : Turn data whitening on / off. \n\t0 = Whitening off. \n\t1 = Whitening on.\r\n"
           "setpktformat <pktformat> : Format of RX and TX data. \n\t0 = Normal mode, use FIFOs for RX and TX. \n\t1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. \n\t2 = Random TX mode; sends random data using PN9 generator.  \n\t3 = Asynchronous serial mode\r\n"
           "setlengthconfig <mode> : Set packet Length mode : \n\t0 = Fixed packet length mode. \n\t1 = Variable packet length mode. \n\t2 = Infinite packet length mode. \n\t3 = Reserved \r\n"
   		    "\r\n"
           "setpacketlength <mode> : Indicates the packet length when fixed packet length mode is enabled. \n\tIf variable packet length mode is used, this value indicates the maximum packet length allowed.\r\n"
           "setcrc <mode> : Switches on/of CRC calculation and check. \n\t1 = CRC calculation in TX and CRC check in RX enabled. \n\t0 = CRC disabled for TX and RX.\r\n"
           "setcrcaf <mode> : Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.\r\n"
           ));
        Serial.println(F(
           "setdcfilteroff <mode> : Disable digital DC blocking filter before demodulator. \n\tOnly for data rates ≤ 250 kBaud \n\tThe recommended IF frequency changes when the DC blocking is disabled. \n\t1 = Disable (current optimized). \n\t0 = Enable (better sensitivity).\r\n"
           "setmanchester <mode> : Enables Manchester encoding/decoding. \n\t0 = Disable. \n\t1 = Enable.\r\n"
           "setfec <mode> : Enable Forward Error Correction (FEC) \n\twith interleaving for packet payload (Only supported for fixed packet length mode. \n\t0 = Disable. \n\t1 = Enable.\r\n"
   		    "\r\n"
           "setpre <mode> : Sets the minimum number of preamble bytes to be transmitted. \n\tValues: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24\r\n"
           "setpqt <mode> : Preamble quality estimator threshold. \r\n"
           "setappendstatus <mode> : When enabled, two status bytes will be appended to the payload of the packet. \n\tThe status bytes contain RSSI and LQI values, \n\tas well as CRC OK.\r\n"
   		    "\r\n"
           "getrssi : Display quality information about last received frames over RF\r\n"
           "scan <start> <stop> : Scan frequency range for the highest signal.\r\n"
           "chat :  Enable chat mode between many devices. \n\tNo exit available, disconnect device to quit\r\n"
   		    "\r\n"
           ));
        Serial.println(F(
           "rx : Sniffer. Enable or disable printing of received RF packets on serial terminal.\r\n"
           "tx <hex-vals> : Send packet of max 60 bytes <hex values> over RF\r\n"
           "cal : send 8 ook beacons\r\n"
           "flush : Clear the recording buffer\r\n"
           "play <N> : Replay 0 = all frames or N-th recorded frame previously stored in the buffer.\r\n"
   		    "\r\n"
   		   "rxnew : fsk with SYMBOL clock\r\n"
           "rxraw <microseconds> : Sniffs radio by sampling with <microsecond> interval \n\tand prints received bytes in hex.\r\n"
           "recraw <microseconds> : Recording RAW RF data with <microsecond> sampling interval.\r\n"
           ));
        Serial.println(F(
           "addraw <hex-vals> : Manually add chunks (max 60 hex values) to the buffer \n\tso they can be further replayed.\r\n"
   		    "\r\n"
           "showraw : Showing content of recording buffer in RAW format.\r\n"
           "playraw <microseconds> : Replaying previously recorded RAW RF data with <microsecond> sampling interval.\r\n"
           "showbit : Showing content of recording buffer in RAW format as a stream of bits.\r\n"
   		    "\r\n"
           "init : Restarts CC1101 board with default parameters\r\n"
           ));

        // Handling SETMODULATION command
    }
    else if (strcmp_P(cmd, PSTR("setmodulation")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setModulation(setting);
        Serial.print(F("\r\nModulation: "));

        if (setting == 0)
            Serial.print(F("2-FSK"));
        else if (setting == 1)
            Serial.print(F("GFSK"));
        else if (setting == 2)
            Serial.print(F("ASK/OOK"));
        else if (setting == 3)
            Serial.print(F("4-FSK"));
        else if (setting == 4)
            Serial.print(F("MSK"));

        ;
        Serial.print(F(" \r\n"));

        // Handling SETMHZ command
    }
    else if (strcmp_P(cmd, PSTR("setmhz")) == 0)
    {
        nextParam = atof(cmd_args);
        radio.setMHZ(nextParam);
        Serial.print(F("\r\nFrequency: "));
        Serial.print(nextParam);
        Serial.print(F(" gMHz\r\n"));

        // Handling SETDEVIATION command
    }
    else if (strcmp_P(cmd, PSTR("setdeviation")) == 0)
    {
        nextParam = atof(cmd_args);
        radio.setDeviation(nextParam);
        Serial.print(F("\r\nDeviation: "));
        Serial.print(nextParam);
        Serial.print(F(" KHz\r\n"));

        // Handling SETCHANNEL command
    }
    else if (strcmp_P(cmd, PSTR("setchannel")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setLogicalChanNum(setting);
        Serial.print(F("\r\nChannel:"));
        Serial.print(setting);
        Serial.print(F("\r\n"));

        // Handling SETCHSP command
    }
    else if (strcmp_P(cmd, PSTR("setchsp")) == 0)
    {
        nextParam = atof(cmd_args);
        radio.setChannelSpacing(nextParam);
        Serial.print(F("\r\nChann spacing: "));
        Serial.print(nextParam);
        Serial.print(F(" kHz\r\n"));

        // Handling SETRXBW command
    }
    else if (strcmp_P(cmd, PSTR("setrxbw")) == 0)
    {
        nextParam = atof(cmd_args);
        radio.setRxBW(nextParam);
        Serial.print(F("\r\nRX bandwidth: "));
        Serial.print(nextParam);
        Serial.print(F(" kHz \r\n"));

        // Handling SETDRATE command
    }
    else if (strcmp_P(cmd, PSTR("setdrate")) == 0)
    {
        nextParam = atof(cmd_args);
        radio.setBaudRate(nextParam);
        Serial.print(F("\r\nDatarate: "));
        Serial.print(nextParam);
        Serial.print(F(" kbaud\r\n"));

        // Handling SETPA command
    }
    else if (strcmp_P(cmd, PSTR("setpa")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setPA(setting);
        Serial.print(F("\r\nTX PWR: "));
        Serial.print(setting);
        Serial.print(F(" dBm\r\n"));

        // Handling SETSYNCMODE command
    }
    else if (strcmp_P(cmd, PSTR("setsyncmode")) == 0)
    {
        int setting = atoi(cmd_args);
        radio.setSyncMode(setting);
        Serial.print(F("\r\nSynchronization: "));

        if (setting == 0)
            Serial.print(F("No preamble"));
        else if (setting == 1)
            Serial.print(F("16 sync bits"));
        else if (setting == 2)
            Serial.print(F("16/16 sync bits"));
        else if (setting == 3)
            Serial.print(F("30/32 sync bits"));
        else if (setting == 4)
            Serial.print(F("No preamble/sync, carrier-sense"));
        else if (setting == 5)
            Serial.print(F("15/16 + carrier-sense"));
        else if (setting == 6)
            Serial.print(F("16/16 + carrier-sense"));
        else if (setting == 7)
            Serial.print(F("30/32 + carrier-sense"));

        ;
        Serial.print(F("\r\n"));

        // Handling SETSYNCWORD command
    }
    else if (strcmp_P(cmd, PSTR("setsyncword")) == 0)
    {
        setting = atoi(strsep(&cmd_args, " "));
        setting2 = atoi(cmd_args);
        radio.setSyncWord(setting2, setting);
        Serial.print(F("\r\nSynchronization:\r\n"));
        Serial.print(F("high = "));
        Serial.print(setting);
        Serial.print(F("\r\nlow = "));
        Serial.print(setting2);
        Serial.print(F("\r\n"));


        // Handling SETADRCHK command
    }
    else if (strcmp_P(cmd, PSTR("setadrchk")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setAdrChk(setting);
        Serial.print(F("\r\nAddress checking:"));

        if (setting == 0)
            Serial.print(F("No adr chk"));
        else if (setting == 1)
            Serial.print(F("Adr chk, no bcast"));
        else if (setting == 2)
            Serial.print(F("Adr chk and 0 bcast"));
        else if (setting == 3)
            Serial.print(F("Adr chk and 0 and FF bcast"));

        ;
        Serial.print(F("\r\n"));

        // Handling SETADDR command
    }
    else if (strcmp_P(cmd, PSTR("setaddr")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setAddr(setting);
        Serial.print(F("\r\nAddress: "));
        Serial.print(setting);
        Serial.print(F("\r\n"));

        // Handling SETWHITEDATA command
    }
    else if (strcmp_P(cmd, PSTR("setwhitedata")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setWhiteData(setting);
        Serial.print(F("\r\nWhitening "));

        if (setting == 0)
            Serial.print(F("OFF"));
        else if (setting == 1)
            Serial.print(F("ON"));

        Serial.print(F("\r\n"));

        // Handling SETPKTFORMAT command
    }
    else if (strcmp_P(cmd, PSTR("setpktformat")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setPktFormat(setting);
        Serial.print(F("\r\nPacket format: "));

        if (setting == 0)
            Serial.print(F("Normal mode"));
        else if (setting == 1)
            Serial.print(F("Synchronous serial mode"));
        else if (setting == 2)
            Serial.print(F("Random TX mode"));
        else if (setting == 3)
            Serial.print(F("Asynchronous serial mode"));

        ;
        Serial.print(F("\r\n"));

        // Handling SETLENGTHCONFIG command
    }
    else if (strcmp_P(cmd, PSTR("setlengthconfig")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setLengthConfig(setting);
        Serial.print(F("\r\nPkt length mode: "));

        if (setting == 0)
            Serial.print(F("Fixed"));
        else if (setting == 1)
            Serial.print(F("Variable"));
        else if (setting == 2)
            Serial.print(F("Infinite"));
        else if (setting == 3)
            Serial.print(F("Reserved"));

        ;
        Serial.print(F("\r\n"));

        // Handling SETPACKETLENGTH command
    }
    else if (strcmp_P(cmd, PSTR("setpacketlength")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setPacketLength(setting);
        Serial.print(F("\r\nPkt length: "));
        Serial.print(setting);
        Serial.print(F(" bytes\r\n"));

        // Handling SETCRC command
    }
    else if (strcmp_P(cmd, PSTR("setcrc")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setCrc(setting);
        Serial.print(F("\r\nCRC checking: "));

        if (setting == 0)
            Serial.print(F("Disabled"));
        else if (setting == 1)
            Serial.print(F("Enabled"));

        ;
        Serial.print(F("\r\n"));

        // Handling SETCRCAF command
    }
    else if (strcmp_P(cmd, PSTR("setcrcaf")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setCRC_AF(setting);
        Serial.print(F("\r\nCRC Autoflush: "));

        if (setting == 0)
            Serial.print(F("Disabled"));
        else if (setting == 1)
            Serial.print(F("Enabled"));

        ;
        Serial.print(F("\r\n"));

        // Handling SETDCFILTEROFF command
    }
    else if (strcmp_P(cmd, PSTR("setdcfilteroff")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setDcFilterOff(setting);
        Serial.print(F("\r\nDC filter: "));

        if (setting == 0)
            Serial.print(F("Enabled"));
        else if (setting == 1)
            Serial.print(F("Disabled"));

        ;
        Serial.print(F("\r\n"));

        // Handling SETMANCHESTER command
    }
    else if (strcmp_P(cmd, PSTR("setmanchester")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setManchester(setting);
        Serial.print(F("\r\nManchester coding: "));

        if (setting == 0)
            Serial.print(F("Disabled"));
        else if (setting == 1)
            Serial.print(F("Enabled"));

        ;
        Serial.print(F("\r\n"));

        // Handling SETFEC command
    }
    else if (strcmp_P(cmd, PSTR("setfec")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setFEC(setting);
        Serial.print(F("\r\nForward Error Correction: "));

        if (setting == 0)
            Serial.print(F("Disabled"));
        else if (setting == 1)
            Serial.print(F("Enabled"));

        ;
        Serial.print(F("\r\n"));

        // Handling SETPRE command
    }
    else if (strcmp_P(cmd, PSTR("setpre")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setNumPreambleBytes(setting);
        Serial.print(F("\r\nMinimum preamble bytes:"));
        Serial.print(setting);
        Serial.print(F(" means 0 = 2 bytes, 1 = 3b, 2 = 4b, 3 = 6b, 4 = 8b, 5 = 12b, 6 = 16b, 7 = 24 bytes\r\n"));
        Serial.print(F("\r\n"));


        // Handling SETPQT command
    }
    else if (strcmp_P(cmd, PSTR("setpqt")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setPQT(setting);
        Serial.print(F("\r\nPQT: "));
        Serial.print(setting);
        Serial.print(F("\r\n"));

        // Handling SETAPPENDSTATUS command
    }
    else if (strcmp_P(cmd, PSTR("setappendstatus")) == 0)
    {
        setting = atoi(cmd_args);
        radio.setAppendStatus(setting);
        Serial.print(F("\r\nStatus bytes appending: "));

        if (setting == 0)
            Serial.print(F("Enabled"));
        else if (setting == 1)
            Serial.print(F("Disabled"));

        ;
        Serial.print(F("\r\n"));

        // Handling GETRSSI command
    }
    else if (strcmp_P(cmd, PSTR("getrssi")) == 0)
    {
        //Rssi Level in dBm
        Serial.print(F("Rssi: "));
        Serial.println(radio.getRssi());
        //Link Quality Indicator
        Serial.print(F(" LQI: "));
        Serial.println(radio.getLqi());
        Serial.print(F("\r\n"));


        // Handling SCAN command - frequency scanner by Little S@tan !
    }
    else if (strcmp_P(cmd, PSTR("scan")) == 0)
    {
    	// round down to nearest step size
        nextParam = atof(strsep(&cmd_args, " "));
        uint32_t start = ((nextParam * 1000000.)/ DEFAULT_STEP) * DEFAULT_STEP;
        nextParam = start/1000000;

        // round up to nearest step size
        endFreq = atof(cmd_args);
		uint32_t end = (((endFreq * 1000000.) + DEFAULT_STEP/2) /DEFAULT_STEP ) * DEFAULT_STEP;
		endFreq = end/1000000;
		
        Serial.print(F("\r\nScanning frequency range from : "));
        Serial.print(nextParam);
        Serial.print(F(" gMHz to "));
        Serial.print(endFreq);
        Serial.print(F(" MHz, press any key for stop or wait...\r\n"));

		delay(6000);
		
        // initialize parameters for scanning
        radio.Init();
        radio.setRxBW(58);
        radio.EnterRxMode();

        // Do scanning until some key pressed
        freq = nextParam;  // start frequency for scanning
        mark_rssi = -100;

        while (!Serial.available())
        {
        	Serial.printf("scan freq = %f ", freq);
            radio.setMHZ(freq);
            delay(50);
            rssi = radio.getRssi();
        	Serial.printf(" rssi = %d\n", rssi);

            if (rssi > -75)
            {
                if (rssi > mark_rssi)
                {
                    mark_rssi = rssi;
                    mark_freq = freq;
                }
            }
            freq += (float)DEFAULT_STEP/1000000.0; // 0.01;

            if (freq > endFreq)
            {
                freq = nextParam;

                if (mark_rssi > -75)
                {
                    long fr = mark_freq * 100;

                    if (fr == compare_freq)
                    {
                        Serial.print(F("\r\nSignal found at  "));
                        Serial.print(F("Freq: "));
                        Serial.print(mark_freq);
                        Serial.print(F(" Rssi: "));
                        Serial.println(mark_rssi);
                        mark_rssi = -100;
                        compare_freq = 0;
                        mark_freq = 0;
                    }
                    else
                    {
                        compare_freq = mark_freq * 100;
                        //freq = mark_freq - 0.10;
                        mark_freq = 0;
                        mark_rssi = -100;
                    }
               }
            }
        }
		Serial.read();
		
        // handling SAVE command
    }

    else if (strcmp_P(cmd, PSTR("rx")) == 0)
    {
        Serial.print(F("\r\nReceiving and printing RF packet changed to "));

        if (receivingmode == 1)
        {
            receivingmode = 0;
            Serial.print(F("Disabled"));
        }
        else if (receivingmode == 0)
        {
            radio.EnterRxMode();
            Serial.print(F("Enabled"));
            receivingmode = 1;
            jammingmode = 0;
            recordingmode = 0;
        }

        ;
        Serial.print(F("\r\n"));


        // Handling CHAT command
    }

    else if (strcmp_P(cmd, PSTR("tx")) == 0)
    {
    	txSendByFifos();
    }
    else if (strcmp_P(cmd, PSTR("cal-CW")) == 0)
    {
    	char *temp = strsep(&cmd_args, " ");
    	if (temp)
    	{
	    	uint32_t startFreq = atoi(temp);
    		bandCalKnob(startFreq);
    	}
    	else
    	{
    		Serial.printf("cal-CW requires a start freq\n");
    	}
    }
	else if (strcmp_P(cmd, PSTR("cal-pkt")) == 0)
	{
    	byte binaryArray[50];

        // convert hex array to set of bytes
        int iCnt = sizeof(binaryArray);
        
		/////radio.setCCMode(I_DUNNO);

		radio.EnterIdleMode();
		
		radio.setCCMode(GDO0_isSYNC_TXEND);  //gdO = SYNC+Sent
		radio.setModulation(2); //ook
		radio.setBaudRate(300);
		radio.setMHZ();
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

        	// send these data to radio over CC1101
			//radio.SendBinaryDataWithNoGDO(binaryArray, iCnt, 1000);

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

	// handling RECRAW command
    else if (strcmp_P(cmd, PSTR("recraw")) == 0)
    {
        // take interval period for samplink
        //setting = atoi(param2);
        
		setting =  (1.e6/9600);
		
        if (setting > 0)
        {
            // setup async mode on CC1101 with GDO0 pin processing
            radio.setCCMode(I_DUNNO);
            radio.setPktFormat(3);
            radio.EnterRxMode();


            //start recording to the buffer with bitbanging of GDO0 pin state
            Serial.print(F("\r\nWaiting for radio signal to start RAW recording...\r\n"));
            radio.setGDO0_hostpinMode( INPUT);

            // this is only for ESP32 boards because they are getting some noise on the beginning
            setting2 = digitalRead(PIN_GDO0);
            delayMicroseconds(1000);

            // waiting for some data first or serial port signal
            while (digitalRead(PIN_GDO0) == LOW);

            //start recording to the buffer with bitbanging of GDO0 pin state
            Serial.print(F("\r\nStarting RAW recording to the buffer...\r\n"));

			radio.enableRisingIRQ_GDO0(true);

            for (int i = 0; i < RECORDINGBUFFERSIZE ; i++)
            {
                byte receivedbyte = 0;

                for (int j = 7; j > -1; j--)                        // 8 bits in a byte
                {
                    bitWrite(receivedbyte, j, digitalRead(PIN_GDO0));   // Capture GDO0 state into the byte
                    delayMicroseconds(setting);                     // delay for selected sampling interval
                }

                ;
                // store the output into recording buffer
                bigrecordingbuffer[i] = receivedbyte;
            }

            Serial.printf("\nRecording RAW data complete. up=%d dn=%d\n", irqUpCtrGDO0, irqDnCtrGDO0);

			radio.enableRisingIRQ_GDO0(false);

            
            // setting normal pkt format again
            radio.setCCMode(GDO0_isSYNC_TXEND);
            radio.setPktFormat(0);
            radio.EnterRxMode();
        }
        else
        {
            Serial.print(F("Wrong parameters.\r\n"));
        }

        ;

        // handling RXRAW command - sniffer
    }
    else if (strcmp_P(cmd, PSTR("rxraw")) == 0)
    {
        // take interval period for samplink
        //setting = atoi(param2);
		setting =  (1.e6/9600);

        if (setting > 0)
        {
            // setup async mode on CC1101 with GDO0 pin processing
            radio.setCCMode(I_DUNNO);
            radio.setPktFormat(3);
            radio.setModulation(DEFAULT_MODULATION); //fsk-4
            radio.EnterRxMode();
            
            //start recording to the buffer with bitbanging of GDO0 pin state
            Serial.print(F("\r\nSniffer enabled...\r\n"));
            radio.setGDO0_hostpinMode(INPUT);

            // Any received char over Serial port stops printing  RF received bytes
            while (!Serial.available())
            {

                // we have to use the buffer not to introduce delays
                for (int i = 0; i < RECORDINGBUFFERSIZE ; i++)
                {
                    byte receivedbyte = 0;

                    for (int j = 7; j > -1; j--)                        // 8 bits in a byte
                    {
                        bitWrite(receivedbyte, j, digitalRead(PIN_GDO0));   // Capture GDO0 state into the byte
                        delayMicroseconds(setting);                     // delay for selected sampling interval
                    }

                    ;
                    // store the output into recording buffer
                    bigrecordingbuffer[i] = receivedbyte;
                }

                // when buffer full print the ouptput to serial port
                for (int i = 0; i < RECORDINGBUFFERSIZE ; i = i + 32)
                {
                    binToAscii(&bigrecordingbuffer[i], textBuffer, 32);
                    Serial.print((char *)textBuffer);
                }


            }; // end of While loop
			Serial.read();
			
            Serial.printf("\nStopping the sniffer. up=%d dn=%d \n", irqUpCtrGDO0, irqDnCtrGDO0);
            
            // setting normal pkt format again
            radio.setCCMode(GDO0_isSYNC_TXEND);
            radio.setPktFormat(0);
            radio.EnterRxMode();
        }
        else
        {
            Serial.print(F("Wrong parameters.\r\n"));
        }

    } 
    else if (strcmp_P(cmd, PSTR("rxnew")) == 0)
    {
        // take interval period for samplink
        //setting = atoi(param2);
		setting =  (1.e6/9600);
		
        if (setting > 0)
        {
            // setup async mode on CC1101 with GDO0 pin processing
            radio.setCCMode(SYMBOL_TICK);
            radio.setModulation(DEFAULT_MODULATION); //fsk-4
            radio.EnterRxMode();
            
            //start recording to the buffer with bitbanging of GDO0 pin state
            Serial.print(F("\r\n New Sniffer enabled...\r\n"));
            radio.setGDO2_hostpinMode(INPUT);

			radio.enableRisingIRQ_GDO2(true);

            // Any received char over Serial port stops printing  RF received bytes

            uint32_t start = micros();
            while (!Serial.available())
            {
#if 1
			
                // we have to use the buffer not to introduce delays
                for (int i = 0; i < RECORDINGBUFFERSIZE ; i++)
                {
                
                    byte receivedbyte = 0;

					// di-bit count, move by 2 bits per symbol.
                    for (int j = 7; j > 0 ; j -=2)                        // 8 bits in a byte
                    {
						bool ret = radio.wait4RisingIRQ_GDO2();
						if (ret == true)
						{
							// GDO0 points to one part of the di-bit.
							radio.setGDOxPinConfig(CONFIG_IOCFG0, 0x16, true);
							bitWrite(receivedbyte, j, digitalRead(PIN_GDO0));	// Capture GDO0 state into the byte

							// GDO0 points to the OTHER part of the di-bit.
							radio.setGDOxPinConfig(CONFIG_IOCFG0, 0x17, true);
							bitWrite(receivedbyte, j-1, digitalRead(PIN_GDO0));	// Capture GDO0 state into the byte
						}
						else
							Serial.print('x'); //should never happen.
						
                     }

                    ;
                    // store the output into recording buffer
                    bigrecordingbuffer[i] = receivedbyte;
                }


                // when buffer full print the ouptput to serial port
                for (int i = 0; i < RECORDINGBUFFERSIZE ; i = i + 32)
                {
                    binToAscii(&bigrecordingbuffer[i], textBuffer, 32);
                    Serial.print((char *)textBuffer);
                }

#endif
            }; // end of While loop
			Serial.read();
			
            uint32_t deltaT = micros() - start;

            Serial.printf("\nStopping the new sniffer. up=%d dn=%d \n", irqUpCtrGDO2, irqDnCtrGDO2);
            Serial.printf("\nbitrate = %f\n", (float) irqUpCtrGDO2 / (float) deltaT);
            
			radio.enableRisingIRQ_GDO2(false);


            // setting normal pkt format again
            radio.setCCMode(GDO0_isSYNC_TXEND);
            radio.setPktFormat(0);
            radio.EnterRxMode();
        }
        else
        {
            Serial.print(F("Wrong parameters.\r\n"));
        }
    }
    else if (strcmp_P(cmd, PSTR("x25")) == 0)
    {
    	layer1();
    }
	else if (strcmp_P(cmd, PSTR("p25")) == 0)
	{
    	#define SAVE_SIZE 70
    	uint8_t p25buf[SAVE_SIZE];
    	uint8_t p25Cnt = 0;
    	
        radio.EnterIdleMode();

        radio.setFreqHz(866887500);
        radio.setPktFormat(0);
        
        radio.setGDOxPinConfig(CONFIG_IOCFG2, INPUT);

        radio.setSyncWord(0x75, 0x5F);
        radio.setSyncMode(1); //1// of 16 bits ok
        // sigh radio.setPQT(3);
        
        radio.setPacketLength(CC_FIFOSIZE);

        //fixed packet length
        //packet size NOT inside packet
        //packet size set above in setPacketLength(blah);
        radio.setLengthConfig(0);
        

        radio.setBaudRate(9600);
        radio.setDeviation(3.6); // (1.8k L + 1.8k R) = 3.6k

        radio.setModulation(3); //fsk-4
        
        radio.setGDO2_hostpinMode(INPUT);

        radio.setGDOxPinConfig(CONFIG_IOCFG2, 0x6); // flag sync-eop
		radio.enableChangingIRQ_GDO2(true);

        radio.EnterRxMode();

        while (!Serial.available())
        {
        	
        	yield();
   			bool toilet;
   			uint8_t rxCount;
			bool ret = radio.wait4ChangingIRQ_GDO2();
			
			if (ret == true)
			{
				bool bUP = radio.digitalReadGDO2();
				//Serial.printf("%c",  bUP? '+':'-');

				if (bUP)
				{
					p25Cnt = 0;
					memset(p25buf, 0x55, SAVE_SIZE);
					
					while(radio.digitalReadGDO2())
					{
						rxCount = radio.GetRxFifoCount(toilet);
						if (!rxCount) continue;

						float fdev = radio.getCarrierDev();

						//if (rxCount == 1) delayMicroseconds(1000000. * 8./9600);
 						
						// read as fast as possible
						uint8_t rxData = radio.SpiReadReg(CONFIG_FIFO);

						if (p25Cnt == SAVE_SIZE) break;
						
						p25buf[p25Cnt++] = rxData;
						//Serial.printf("[%3d] %02X %c fdev=%7.2f\n", rxCount, rxData, rxData, fdev);
						
						assert(p25Cnt < SAVE_SIZE);
						
						//if (toilet) break;
			
					}
				}
				else
				{
					while(true)
					{
						// drain it!!! 
						rxCount = radio.GetRxFifoCount(toilet);
						if (!rxCount) break;
						if (rxCount == 1) delayMicroseconds(1000000. * 8./9600);
						
						uint8_t rxData = radio.SpiReadReg(CONFIG_FIFO);
						
						if (p25Cnt == SAVE_SIZE) break;
						p25buf[p25Cnt++] = rxData;
						
						//Serial.printf("\t[%3d] %02X %c\n", rxCount, rxData, rxData);
						assert(p25Cnt < SAVE_SIZE);
						
						
					}
					
					if (toilet) radio.SpiStrobe(CC1101_SFRX);
					dumpBinary(p25buf,p25Cnt);
				
				}
				
			}
			else
			{
				// timeout
				rxCount =radio.GetRxFifoCount(toilet);
				Serial.printf("_%d_", rxCount);
			}
		}
		Serial.read();

		radio.EnterIdleMode();
		radio.enableChangingIRQ_GDO2(false);

        // setting normal pkt format again
        radio.setCCMode(GDO0_isSYNC_TXEND);
        radio.setPktFormat(0);
        radio.EnterRxMode();

    }
    else if (strcmp_P(cmd, PSTR("showraw")) == 0)
    {
        // show the content of recorded RAW signal as hex numbers
        Serial.print(F("\r\nRecorded RAW data:\r\n"));

        for (int i = 0; i < RECORDINGBUFFERSIZE ; i = i + 32)
        {
            binToAscii(&bigrecordingbuffer[i], textBuffer, 32);
            Serial.print((char *)textBuffer);
        }

        Serial.print(F("\r\n"));


        // handling SHOWBIT command
    }
    else if (strcmp_P(cmd, PSTR("rec")) == 0)
    {
        Serial.print(F("\r\nRecording mode set to "));

        if (recordingmode == 1)
        {
            Serial.print(F("Disabled"));
            bigrecordingbufferpos = 0;
            recordingmode = 0;
        }
        else if (recordingmode == 0)
        {
            radio.EnterRxMode();
            Serial.print(F("Enabled"));
            bigrecordingbufferpos = 0;

            // flush buffer for recording
            for (int i = 0; i < RECORDINGBUFFERSIZE; i++)
                bigrecordingbuffer[i] = 0;

            ;
            recordingmode = 1;
            jammingmode = 0;
            receivingmode = 0;
            // start counting frames in the buffer
            framesinbigrecordingbuffer = 0;
        }

        ;
        Serial.print(F("\r\n"));


        // Handling PLAY command
    }
    else if (strcmp_P(cmd, PSTR("show")) == 0)
    {
        if (framesinbigrecordingbuffer > 0)
        {
            Serial.print(F("\r\nFrames stored in the recording buffer:\r\n "));
            // rewind recording buffer position to the beginning
            bigrecordingbufferpos = 0;

            // start reading and sending frames from the buffer : FIFO
            for (setting = 1; setting <= framesinbigrecordingbuffer; setting++)
            {
                // read length of the recorded frame first from the buffer
                len = bigrecordingbuffer[bigrecordingbufferpos];

                if ((len <= 60)and(len > 0))
                {
                    // take next frame from the buffer  for replay
                    // flush hexBuffer
                    for (setting2 = 0; setting2 < BUF_LENGTH; setting2++)
                        textBuffer[setting2] = 0;

                    
                    binToAscii(&bigrecordingbuffer[bigrecordingbufferpos + 1], textBuffer, len);
                    Serial.print(F("\r\nFrame "));
                    Serial.print(setting);
                    Serial.print(F(" : "));
                    Serial.print((char *)textBuffer);
                    Serial.print(F("\r\n"));
                }

                ;
                // increase position to the buffer and check exception
                bigrecordingbufferpos = bigrecordingbufferpos + 1 + len;

                if (bigrecordingbufferpos > RECORDINGBUFFERSIZE)
                    break;

                //
            }

            ;
            // rewind buffer position
            // bigrecordingbufferpos = 0;
            Serial.print(F("\r\n"));
        }
        else
        {
            Serial.print(F("Wrong parameters.\r\n"));
        }

        ;


        // Handling FLUSH command
    }
    else if (strcmp_P(cmd, PSTR("flush")) == 0)
    {
        // flushing bigrecordingbuffer with zeros and rewinding all the pointers
        for (setting = 0; setting < RECORDINGBUFFERSIZE; setting++)
            bigrecordingbuffer[setting] = 0;

        // and rewinding all the pointers to the recording buffer
        bigrecordingbufferpos = 0;
        framesinbigrecordingbuffer = 0;
        Serial.print(F("\r\nRecording buffer cleared.\r\n"));


        // Handling ECHO command
    }
	else if (strcmp_P(cmd, PSTR("x")) == 0)
	{
	    receivingmode = 0;
	    jammingmode = 0;
	    recordingmode = 0;
	    Serial.print(F("\r\n"));

	    // Handling INIT command
	    // command 'init' initializes board with default settings
	}
	else if (strcmp_P(cmd, PSTR("init")) == 0)
	{
	    // init cc1101
	    cc1101initialize();
	    // give feedback
	    Serial.print(F("CC1101 initialized\r\n"));

	}
	else
	{
	    Serial.print(F("Error: Unknown cmd: "));
	    Serial.println(cmd);
	    //  debug only
	    // asciitohex(command, (byte *)hexBuffer,  strlen(command));
	    // Serial.print(F("\r\n"));
	    // Serial.print((char *)hexBuffer);
	    // Serial.print(F("\r\n"));
	}
}


void setup()
{
	// POWER UP THE BUS !!!!!! spi always has power, the BUS does NOT
	// POWER UP THE BUS !!!!!! spi always has power, the BUS does NOT
	// POWER UP THE BUS !!!!!! spi always has power, the BUS does NOT

	M5.begin(); // POWER UP THE BUS !!!!!!

	ArduinoOTA
	   .onStart([]() {
		 String type;
		 if (ArduinoOTA.getCommand() == U_FLASH) {
		   type = "sketch";
		 } else {  // U_SPIFFS
		   type = "filesystem";
		 }
	
		 // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		 Serial.println("Start updating " + type);
	   })
	   .onEnd([]() {
		 Serial.println("\nEnd");
	   })
	   .onProgress([](unsigned int progress, unsigned int total) {
		 Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	   })
	   .onError([](ota_error_t error) {
		 Serial.printf("Error[%u]: ", error);
		 if (error == OTA_AUTH_ERROR) {
		   Serial.println("Auth Failed");
		 } else if (error == OTA_BEGIN_ERROR) {
		   Serial.println("Begin Failed");
		 } else if (error == OTA_CONNECT_ERROR) {
		   Serial.println("Connect Failed");
		 } else if (error == OTA_RECEIVE_ERROR) {
		   Serial.println("Receive Failed");
		 } else if (error == OTA_END_ERROR) {
		   Serial.println("End Failed");
		 }
	   });

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	while (WiFi.waitForConnectResult() != WL_CONNECTED) {
		Serial.println("Connection Failed! Rebooting...");
		delay(5000);
		ESP.restart();
	}

	// Port defaults to 3232
	// ArduinoOTA.setPort(3232);

	// Hostname defaults to esp3232-[MAC]
	ArduinoOTA.setHostname(REMOTE_HOSTNAME);

	// No authentication by default
	// ArduinoOTA.setPassword("admin");

	// Password can be set with it's md5 value as well
	// MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

	ArduinoOTA
	  .onStart([]() {
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH) {
		  type = "sketch";
		} else {  // U_SPIFFS
		  type = "filesystem";
		}
	
		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		Serial.println("Start updating " + type);
	  })
	  .onEnd([]() {
		Serial.println("\nEnd");
	  })
	  .onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	  })
	  .onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) {
		  Serial.println("Auth Failed");
		} else if (error == OTA_BEGIN_ERROR) {
		  Serial.println("Begin Failed");
		} else if (error == OTA_CONNECT_ERROR) {
		  Serial.println("Connect Failed");
		} else if (error == OTA_RECEIVE_ERROR) {
		  Serial.println("Receive Failed");
		} else if (error == OTA_END_ERROR) {
		  Serial.println("End Failed");
		}
	  });

	ArduinoOTA.begin();
	
	 Serial.println("Ready");
	 Serial.print("IP address: ");
	 Serial.println(WiFi.localIP());

	
	// POWER UP THE BUS !!!!!! spi always has power, the BUS does NOT
	// POWER UP THE BUS !!!!!! spi always has power, the BUS does NOT
	// POWER UP THE BUS !!!!!! spi always has power, the BUS does NOT
	
//#if defined (ARDUINO_M5STACK_CORES3)
//    pinMode(19, INPUT);     // S3 bug. Jtag messes up Usb serial
//#endif

    // initialize USB Serial Port CDC
    Serial.begin(115200);
    delay(3000);

//	_setup_M5();
//	_lclear();
//	_cprintf(_GREEN, 0,	"%s", built_on);									   
	

    Serial.println(F("CC1101 terminal tool connected, use 'help' for list of commands..."));
    Serial.println(F("(C) Adam Loboda 2023  "));

    //Init EEPROM - for ESP32 based boards only
    EEPROM.begin(EPROMSIZE);


    Serial.println();   // print CRLF


    // initialize CC1101 module with preffered parameters
    cc1101initialize();

    if (radio.getCC1101())      // Check the CC1101 Spi connection.
        Serial.printf("\ncc1101 initialized. Connection OK\n\n");
    else
        Serial.printf("\ncc1101 connection error! check the wiring.\n\n");


//	_cprintf(_GREEN, 2,	"freq = %f", radio.getMHZ());

    // setup variables
    bigrecordingbufferpos = 0;

    esp_intr_dump(stdout);
}


void loop()
{
	ArduinoOTA.handle();
	radio.getPktStatus();

	static bool bFirstTime = true;
	if (bFirstTime)
	{
		// flush noise (from jtag?)
		while (Serial.available()) 
		{
			Serial.read();
			Serial.print('.');
		}
		Serial.println("ready");
		bFirstTime = false;

#ifndef DNS_YELLOW
		txSendByFifos();
#endif
	

	}
	
    // index for serial port characters
    int i = 0;

    /* Process incoming commands. */
    while (Serial.available())
    {
        static char buffer[BUF_LENGTH];
        static int length = 0;

        // handling CHAT MODE
        if (chatmode == 1)
        {

            // clear serial port buffer index
            i = 0;

            // something was received over serial port put it into radio sending buffer
            while (Serial.available() and(i < (CC_FIFOSIZE - 1)))
            {
                // read single character from Serial port
                ccsendingbuffer[i] = Serial.read();

                // also put it as ECHO back to serial port
                Serial.write(ccsendingbuffer[i]);

                // if CR was received add also LF character and display it on Serial port
                if (ccsendingbuffer[i] == 0x0d)
                {
                    Serial.write(0x0a);
                    i++;
                    ccsendingbuffer[i] = 0x0a;
                }

                //

                // increase CC1101 TX buffer position
                i++;
            }

            ;

            // put NULL at the end of CC transmission buffer
            ccsendingbuffer[i] = '\0';

            // send these data to radio over CC1101
            radio.SendDataCharArray((char *)ccsendingbuffer);


        }
        // handling CLI commands processing
        else
        {
            int data = Serial.read();

            if (data == '\b' || data == '\177') // BS and DEL
            {
                if (length)
                {
                    length--;

                    if (do_echo)
                        Serial.write("\b \b");
                }
            }
            else if (data == '\r' || data == '\n')
            {
                if (do_echo)
                    Serial.write("\r\n");         // output CRLF

                buffer[length] = '\0';

                if (length)
                {
                	radio.EnterIdleMode();
                    exec(buffer);
                }

                length = 0;
            }
            else if (length < BUF_LENGTH - 1)
            {
                buffer[length++] = data;

                if (do_echo)
                    Serial.write(data);
            }
        }

         // end of handling CLI processing

    }
  
    /* Process RF received packets */

    //Checks whether something has been received.
    if (radio.CheckReceiveFlag() && (receivingmode == 1 || recordingmode == 1 || chatmode == 1))
    {

        //CRC Check. If "setCrc(false)" crc returns always OK!
        if (radio.CheckCRC())
        {
            //Get received Data and calculate length
            int len = radio.ReceiveData(ccreceivingbuffer);

            // Actions for CHAT MODE
            if ((chatmode == 1) && (len < CC_FIFOSIZE))
            {
                // put NULL at the end of char buffer
                ccreceivingbuffer[len] = '\0';
                //Print received in char format.
                Serial.print((char *)ccreceivingbuffer);
            }

            ;      // end of handling Chat mode

            // Actions for RECEIVNG MODE
            if (((receivingmode == 1) && (recordingmode == 0)) && (len < CC_FIFOSIZE))
            {
                // put NULL at the end of char buffer
                ccreceivingbuffer[len] = '\0';

                // flush hexBuffer
                for (int i = 0; i < BUF_LENGTH; i++)
                    textBuffer[i] = 0;

                ;

                //Print received packet as set of hex values directly
                // not to loose any data in buffer
                // asciitohex((byte *)ccreceivingbuffer, (byte *)hexBuffer,  len);
                binToAscii(ccreceivingbuffer, textBuffer, len);
                Serial.print((char *)textBuffer);
                // set RX  mode again
                radio.EnterRxMode();
            }

            ;        // end of handling receiving mode

            // Actions for RECORDING MODE
            if (((recordingmode == 1) && (receivingmode == 0)) && (len < CC_FIFOSIZE))
            {
                // copy the frame from receiving buffer for replay - only if it fits
                if ((bigrecordingbufferpos + len + 1) < RECORDINGBUFFERSIZE)
                {      // put info about number of bytes
                    bigrecordingbuffer[bigrecordingbufferpos] = len;
                    bigrecordingbufferpos++;
                    // next - copy current frame and increase
                    memcpy(&bigrecordingbuffer[bigrecordingbufferpos], ccreceivingbuffer, len);
                    // increase position in big recording buffer for next frame
                    bigrecordingbufferpos = bigrecordingbufferpos + len;
                    // increase counter of frames stored
                    framesinbigrecordingbuffer++;
                    // set RX  mode again
                    radio.EnterRxMode();
                }
                else
                {
                    Serial.print(F("Recording buffer full! Stopping..\r\nFrames stored: "));
                    Serial.print(framesinbigrecordingbuffer);
                    Serial.print(F("\r\n"));
                    bigrecordingbufferpos = 0;
                    recordingmode = 0;
                }

                ;

            }

            ;       // end of handling frame recording mode

        }

        ;      // end of CRC check IF


    }

    ;      // end of Check receive flag if

    // if jamming mode activate continously send something over RF...
    if (jammingmode == 1)
    {
        // populate cc1101 sending buffer with random values
        randomSeed(analogRead(0));

        for (i = 0; i < 60; i++)
            ccsendingbuffer[i] = (byte)random(255);

        ;
        // send these data to radio over CC1101
        radio.SendBinaryData(ccsendingbuffer, 60);
    }

    ;

}  // end of main LOOP
