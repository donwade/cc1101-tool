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
#include <stdint.h>
#ifndef ELECHOUSE_CC1101_SRC_DRV_h
#define ELECHOUSE_CC1101_SRC_DRV_h

#include <Arduino.h>

typedef enum 
{
    NOT_INITED,
    I_DUNNO,
    GDO0_isSYNC_TXEND,
    GDO0_isSYNC_RX,
    SYMBOL_TICK
}eGDIO_MODES;

typedef enum {
    TRIG_NONE = 0,
    TRIG_RISING = 1,
    TRIG_FALLING = 2,                                                             
    TRIG_BOTH = 3,
    TRIG_LOW = 4,
    TRIG_HIGH = 5 
} eIRQ_TRIGGER;
typedef enum {
    MODEM_IDLE,
    MODEM_TX,
    MODEM_RX
}eMODEM_STATE;

//***************************************CC1101 define**************************************************//
// CC1101 CONFIG REGSITER
typedef enum CONFIG_REG {
    CC1101_IOCFG2       , // 0x00  GDO2 output pin configuration
    CC1101_IOCFG1       , // 0x01  GDO1 output pin configuration
    CC1101_IOCFG0       , // 0x02  GDO0 output pin configuration
    CC1101_FIFOTHR      , // 0x03  RX FIFO and TX FIFO thresholds
    CC1101_SYNC1        , // 0x04  Sync word, high INT8U
    CC1101_SYNC0        , // 0x05  Sync word, low INT8U
    CC1101_PKTLEN       , // 0x06  Packet length
    CC1101_PKTCTRL1     , // 0x07  Packet automation control
    CC1101_PKTCTRL0     , // 0x08  Packet automation control
    CC1101_ADDR         , // 0x09  Device address
    CC1101_CHANNR       , // 0x0A  Channel number
    CC1101_FSCTRL1      , // 0x0B  Frequency synthesizer control
    CC1101_FSCTRL0      , // 0x0C  Frequency synthesizer control
    CC1101_FREQ2        , // 0x0D  Frequency control word, high INT8U
    CC1101_FREQ1        , // 0x0E  Frequency control word, middle INT8U
    CC1101_FREQ0        , // 0x0F  Frequency control word, low INT8U
    CC1101_MDMCFG4      , // 0x10  Modem configuration
    CC1101_MDMCFG3      , // 0x11  Modem configuration
    CC1101_MDMCFG2      , // 0x12  Modem configuration
    CC1101_MDMCFG1      , // 0x13  Modem configuration
    CC1101_MDMCFG0      , // 0x14  Modem configuration
    CC1101_DEVIATN      , // 0x15  Modem deviation setting
    CC1101_MCSM2        , // 0x16  Main Radio Control State Machine configuration
    CC1101_MCSM1        , // 0x17  Main Radio Control State Machine configuration
    CC1101_MCSM0        , // 0x18  Main Radio Control State Machine configuration
    CC1101_FOCCFG       , // 0x19  Frequency Offset Compensation configuration
    CC1101_BSCFG        , // 0x1A  Bit Synchronization configuration
    CC1101_AGCCTRL2     , // 0x1B  AGC control
    CC1101_AGCCTRL1     , // 0x1C  AGC control
    CC1101_AGCCTRL0     , // 0x1D  AGC control
    CC1101_WOREVT1      , // 0x1E  High INT8U Event 0 timeout
    CC1101_WOREVT0      , // 0x1F  Low INT8U Event 0 timeout
    CC1101_WORCTRL      , // 0x20  Wake On Radio control
    CC1101_FREND1       , // 0x21  Front end RX configuration
    CC1101_FREND0       , // 0x22  Front end TX configuration
    CC1101_FSCAL3       , // 0x23  Frequency synthesizer calibration
    CC1101_FSCAL2       , // 0x24  Frequency synthesizer calibration
    CC1101_FSCAL1       , // 0x25  Frequency synthesizer calibration
    CC1101_FSCAL0       , // 0x26  Frequency synthesizer calibration
    CC1101_RCCTRL1      , // 0x27  RC oscillator configuration
    CC1101_RCCTRL0      , // 0x28  RC oscillator configuration
    CC1101_FSTEST       , // 0x29  Frequency synthesizer calibration control
    CC1101_PTEST        , // 0x2A  Production test
    CC1101_AGCTEST      , // 0x2B  AGC test
    CC1101_TEST2        , // 0x2C  Various test settings
    CC1101_TEST1        , // 0x2D  Various test settings
    CC1101_TEST0        , // 0x2E  Various test settings

    // these are read/writeable but use block mode.
    CC1101_PATABLE =0x3E, // 0x3E
    CC1101_RXTXFIFO     
};



//CC1101 Strobe commands

typedef enum STROBES {
    STROBE_SRES =0x30   ,   // Reset chip. 
    STROBE_SFSTXON      ,   // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                            //  RX/TX: Go to a wait state where only the synthesizer is
                            // running (for quick RX / TX turnaround).

    STROBE_SXOFF        ,   // Turn off crystal oscillator.
    STROBE_SCAL         ,   // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).

    STROBE_SRX          ,   // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.

    STROBE_STX          ,   // 0x35 In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        // Only go to TX if channel is clear.
    STROBE_SIDLE        ,   // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.

    STROBE_SAFC         ,   // Perform AFC adjustment of the frequency synthesizer
    STROBE_SWOR         ,   // Start automatic RX polling sequence (Wake-on-Radio)
    STROBE_SPWD         ,   // Enter power down mode when CSn goes high.
    STROBE_SFRX         ,   // Flush the RX FIFO buffer.
    STROBE_SFTX         ,   // Flush the TX FIFO buffer.
    STROBE_SWORRST      ,   // Reset real time clock.
    STROBE_SNOP             // No operation. May be used to pad strobe commands to two
                            // INT8Us for simpler software.
};

//CC1101 STATUS REGSITER (read only)
typedef enum STATUS_REG
{
    STATUS_PARTNUM =0x30,// 30
    STATUS_VERSION      ,// 31
    STATUS_FREQEST      ,// 32
    STATUS_LQI          ,// 33
    STATUS_RSSI         ,// 34
    STATUS_MARCSTATE    ,// 35
    STATUS_WORTIME1     ,// 36
    STATUS_WORTIME0     ,// 37
    STATUS_PKTSTATUS    ,// 38
    STATUS_VCO_VC_DAC   ,// 39
    STATUS_TXBYTES      ,// 3A
    STATUS_RXBYTES      ,// 3B
    STATUS_RCCTRL1      ,// 3C
    STATUS_RCCTRL2      , // 3D
    STATUS_LASTVALUE
};


//************************************* class **************************************************//
class ELECHOUSE_CC1101
{
private:
void    _setField(const char *name, uint8_t register, uint8_t val, uint8_t LHS, uint8_t RHS);
uint8_t _getField(const char *regName, uint8_t regNum, uint8_t LHS, uint8_t RHS);

void SpiStart(void);
void SpiEnd(void);

void GDOx_SetPinMode(void);
void Reset(void);

void setSpi(void);

void RegConfigSettings(void);
void Calibrate(bool bSilent = true);

public:
void DumpRegs(void);
void DumpMirror(char *msg);

void snapshot1(void);
void snapshot2(void);
void diffSnapshots(void);

void Init(void);
int getPktStatus(bool bOnlyOnChange = false);

byte SpiReadStatus(STATUS_REG addr);
void setSpiPin(byte sck, byte miso, byte mosi, byte ss);
void setGDOx(byte gdo0, byte gdo2);
void defineGDO0_pinNum(byte gdo0);
void setTxFifoThreshold(uint8_t v);
void setCCMode(eGDIO_MODES s);
void setModulation(byte m);
void setPA(int p);

void setLnaStrategy(bool bType1);
void setCarrierSenseAbs(int8_t vDb);
void setCarrierSenseRel(int8_t vDb);
uint8_t setMAGNTarget(uint8_t vDb);
void setCCAmode(uint8_t type);
void setRxOffMode(uint8_t type);
void setTxOffMode(uint8_t type);


void setMHZ(float mhz = 0.0, bool bSilent = true);
void setFreqHz(uint32_t hz = 0, bool bSilent = true);

float getMHZ(void);
void setGDOxPinConfig(CONFIG_REG reg, uint8_t value, bool bSilent=false);

void enableRisingIRQ_GDO0(bool bEnable);
void enableFallingIRQ_GDO0(bool bEnable);

void enableRisingIRQ_GDO2(bool bEnable);
void enableFallingIRQ_GDO2(bool bEnable);

bool wait4RisingIRQ_GDO0(void);
bool wait4FallingIRQ_GDO0(void);

bool wait4RisingIRQ_GDO2(void);
bool wait4FallingIRQ_GDO2(void);



void setLogicalChanNum(byte chnl);
void setChannelSpacing(float f);
void setRxBW(float f);
void setBaudRate(uint32_t d);
void setDeviation_FSK2(float d);
void setSymbolSpacingHz(float d);
void StartTransmitter(void);
void EnterRxMode(bool bSilent=true);
void EnterRxMode(float mhz, bool bSilent=true);
int getRssi(void);
byte getLqi(void);
byte getMSMState(bool bSilent=true);
byte getState(bool bSilent = true);
int getErrorCarrier(void);


void setSres(void);
void EnterIdleMode(bool bSilent = true);
void goSleep(void);
void SendBinaryData(byte *txBuffer, byte size);
void SendDataCharArray(char *txchar);
void SendBinaryDataWithNoGDO(byte *txBuffer, byte size, int t);
void SendBinaryData(char *txchar, int t);
void SendDataCppString(String &txchar);

byte CheckReceiveFlag(void);
byte ReceiveData(byte *rxBuffer);
bool CheckCRC(void);
uint8_t SpiStrobe(STROBES strobe, bool bSilent = true);
void _SpiWriteReg(const char*name, byte addr, byte value, bool bQuiet=false);
void SpiWriteBurstReg(CONFIG_REG addr, byte *buffer, byte num);
byte SpiReadReg(CONFIG_REG addr);
void SpiReadBurstReg(CONFIG_REG addr, byte *buffer, byte num);
void setCalibrationOffset(byte b, int32_t s, int32_t e);
bool getCC1101(void);
eMODEM_STATE getMode(void);
void setSyncWord(byte sh, byte sl);
void setAddr(byte v);
void setWhiteData(bool v);
void setPktFormat(byte v);
void setCrc(bool v);
void setLengthConfig(byte v);
void setPacketLength(byte v);
void setDcFilterOff(bool v);
void setManchester(bool v);
void setSyncMode(byte v);
void setFEC(bool v);
void setNumPreambleBytes(byte v);
void setPQT(byte v);
void setCRC_AF(bool v);
void setAppendStatus(bool v);
void setAdrChk(byte v);
bool CheckRxFifo(int t);
void setGDO0_hostpinMode(int8_t direction = INPUT);
void setGDO2_hostpinMode(int8_t direction = INPUT);
bool getGDO0(void);
bool getGDO2(void);

private:

    int irqDirGDO0;
    int irqDirGDO2;
    
 };

extern uint32_t irqUpCtrGDO0;
extern uint32_t irqDnCtrGDO0;
extern uint32_t irqUpCtrGDO2;
extern uint32_t irqDnCtrGDO2;
extern uint32_t irqDeltaTimeGDO0;
extern uint32_t irqDeltaTimeGDO2;

extern SemaphoreHandle_t sem_GDO0_UP ;
extern SemaphoreHandle_t sem_GDO0_DN ;
extern SemaphoreHandle_t sem_GDO2_UP ;
extern SemaphoreHandle_t sem_GDO2_DN ;


extern ELECHOUSE_CC1101 ELECHOUSE_cc1101;
#define DEFAULT_BAUD 300
#define DEFAULT_SPACING 4800  // was 1200
#define CC_FIFOSIZE 64
#define DEFAULT_MODULATION  3 //fsk-4
#define MASK_GETBYTES_FIFO   0x7F            //byte number in RXfifo

void runP25(void);
// buffer for recording and replaying of many frames

#define RECORDINGBUFFERSIZE 4096/2    // Buffer for recording the frames

#define LINE Serial.printf(">>>  %s:%d \n", __FUNCTION__,__LINE__);

extern uint32_t ulastTimeRisingGO0;
extern uint32_t ulastTimeFallingGO0;
extern uint32_t ulastTimeRisingGO2;
extern uint32_t ulastTimeFallingGO2;


#if defined (ARDUINO_M5STACK_CORE2)

#define PIN_MOSI 23
#define PIN_MISO 38
#define PIN_SCK 18

#define PIN_CS 27

#define PIN_GDO2 19
#define PIN_GDO0 33

#elif defined (ARDUINO_M5STACK_CORES3)
#define PIN_MOSI 37
#define PIN_MISO 35
#define PIN_SCK 36

#define PIN_CS 5

#define PIN_GDO2 10
#define PIN_GDO0 7

#else
#error unknown processor
#endif

 
void binToAscii(byte *asciiIn, char *hexOut, int len);

#endif
