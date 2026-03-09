#include <SPI.h>
#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <Arduino.h>
#include "pretty.h"



typedef struct ONE_ENTRY
{
	char *msg;
	uint8_t regnum;
	uint8_t value;
};

ONE_ENTRY MANY[] =
{
    { "IOCFG1"	, 0x0001, 0x2E }, 		// GDO1 Output Pin Configuration
    { "IOCFG0"	, 0x0002, 0x06 }, 		// GDO0 Output Pin Configuration
    { "FIFOTHR"	, 0x0003, 0x47 }, 		// RX FIFO and TX FIFO Thresholds
    { "SYNC1"	, 0x0004, 0x7A }, 		// Sync Word, High Byte
    { "SYNC0"	, 0x0005, 0x0E }, 		// Sync Word, Low Byte
    { "PKTLEN"	, 0x0006, 0x14 }, 		// Packet Length
    { "PKTCTRL1", 0x0007, 0x04 }, 		// Packet Automation Control
    { "PKTCTRL0", 0x0008, 0x05 }, 		// Packet Automation Control
    { "ADDR"	, 0x0009, 0x00 }, 		// Device Address
    { "CHANNR"	, 0x000A, 0x00 }, 		// Channel Number
    { "FSCTRL1"	, 0x000B, 0x06 }, 		// Frequency Synthesizer Control
    { "FSCTRL0"	, 0x000C, 0x00 }, 		// Frequency Synthesizer Control
    { "FREQ2"	, 0x000D, 0x21 }, 		// Frequency Control Word, High Byte
    { "FREQ1"	, 0x000E, 0x4E }, 		// Frequency Control Word, Middle Byte
    { "FREQ0"	, 0x000F, 0xC4 }, 		// Frequency Control Word, Low Byte
    { "MDMCFG4"	, 0x0010, 0xF7 }, 		// Modem Configuration
    { "MDMCFG3"	, 0x0011, 0x83 }, 		// Modem Configuration
    { "MDMCFG2"	, 0x0012, 0x46 }, 		// Modem Configuration
    { "MDMCFG1"	, 0x0013, 0x22 }, 		// Modem Configuration
    { "MDMCFG0"	, 0x0014, 0xF8 }, 		// Modem Configuration
    { "DEVIATN"	, 0x0015, 0x01 }, 		// Modem Deviation Setting
    { "MCSM2"	, 0x0016, 0x07 }, 		// Main Radio Control State Machine Configuration
    { "MCSM1"	, 0x0017, 0x30 }, 		// Main Radio Control State Machine Configuration
    { "MCSM0"	, 0x0018, 0x18 }, 		// Main Radio Control State Machine Configuration
    { "FOCCFG"	, 0x0019, 0x16 }, 		// Frequency Offset Compensation Configuration
    { "BSCFG"	, 0x001A, 0x6C }, 		// Bit Synchronization Configuration
    { "AGCCTRL2"	, 0x001B, 0x43 }, 	// AGC Control
    { "AGCCTRL1"	, 0x001C, 0x49 }, 	// AGC Control
    { "AGCCTRL0"	, 0x001D, 0x91 }, 	// AGC Control
    { "WOREVT1"	, 0x001E, 0x87 }, 		// High Byte Event0 Timeout
    { "WOREVT0"	, 0x001F, 0x6B }, 		// Low Byte Event0 Timeout
    { "WORCTRL"	, 0x0020, 0xFB }, 		// Wake On Radio Control
    { "FREND1"	, 0x0021, 0x56 }, 		// Front End RX Configuration
    { "FREND0"	, 0x0022, 0x10 }, 		// Front End TX Configuration
    { "FSCAL3"	, 0x0023, 0xE9 }, 		// Frequency Synthesizer Calibration
    { "FSCAL2"	, 0x0024, 0x2A }, 		// Frequency Synthesizer Calibration
    { "FSCAL1"	, 0x0025, 0x00 }, 		// Frequency Synthesizer Calibration
    { "FSCAL0"	, 0x0026, 0x1F }, 		// Frequency Synthesizer Calibration
    { "RCCTRL1"	, 0x0027, 0x41 }, 		// RC Oscillator Configuration
    { "RCCTRL0"	, 0x0028, 0x00 }, 		// RC Oscillator Configuration
    { "FSTEST"	, 0x0029, 0x59 }, 		// Frequency Synthesizer Calibration Control
    { "PTEST"	, 0x002A, 0x7F }, 		// Production Test
    { "AGCTEST"	, 0x002B, 0x3F }, 		// AGC Test
    { "TEST2"	, 0x002C, 0x81 }, 		// Various Test Settings
    { "TEST1"	, 0x002D, 0x35 }, 		// Various Test Settings
    { "TEST0"	, 0x002E, 0x09 } 		// Various Test Settings
};

void setupTi(void)
{
	for (int i = 0; i < sizeof(MANY)/sizeof(ONE_ENTRY); i++)
	{
		radio._SpiWriteReg(MANY[i].msg, (CONFIG_REG)MANY[i].regnum, MANY[i].value, false);
	}
}
