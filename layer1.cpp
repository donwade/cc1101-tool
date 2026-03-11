
// fix for m5core gpio not defined
//#include "soc/gpio_struct.h"
//#include "hal/gpio_ll.h"
//----


#include <M5Unified.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include "esp_intr_types.h"
#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include "pretty.h"

#include "keyboard.h"
#include "bandcal.h"

/*
time uS	    90 	    97	   104	   111	   118	   125	   132	  
baud	 11111	 10309	  9615	  9009	  8474	  8000	  7575
count		 4		17		10		11		 0		 0		 0 
*/

uint32_t irqGDO0ctr;
uint32_t irqGDO2ctr;

//-----------------------------------------------
#define HISTORY_WIDE 9
uint32_t  history[HISTORY_WIDE];
uint32_t farLeft, farRight;

#define BIG_SHIFT 2

uint64_t bigShifter[BIG_SHIFT], keptMatch[BIG_SHIFT];
uint32_t preambleCtr = 0;

const uint32_t BAUD_4800uS = 1000000/4800;
const uint32_t BAUD_9600uS = 1000000/9600;

#if 0	// 1=go by baud rate 0=go by direct uS

#define BAUD_LEFT 12000
#define BAUD_RIGHT 3500

const int timeLeftBin  = 1000000./BAUD_LEFT;
const int timeRightBin = 1000000./BAUD_RIGHT;
#else
#define US_MIDDLE BAUD_4800uS
#define US_WIDE  20
#define US_LEFT  (US_MIDDLE - US_WIDE)
#define US_RIGHT (US_MIDDLE + US_WIDE)

const int timeLeftBin  = US_LEFT;
const int timeRightBin = US_RIGHT;
#endif

const int BIN_STEP_SIZE = (timeRightBin - timeLeftBin)/ HISTORY_WIDE;


void statsCapture(uint32_t tick)
{
	int index;
	
	if (tick < timeLeftBin ) 
	{
		farLeft++;
    }
	else if (tick > timeRightBin)
	{
		farRight++;
	}
	else
	{
	 	tick -= timeLeftBin;
		index = tick / BIN_STEP_SIZE;
		history[index]++;
	}
}

void show(void)
{
	int i;
	Serial.println();

	Serial.printf("%10s ", "time uS");
 	for (i = 0; i < HISTORY_WIDE; i++)
	{
		Serial.printf("%7d ", timeLeftBin + i * BIN_STEP_SIZE);
	}
	
	Serial.println();

	Serial.printf("%10s ", "baud");
 	for (i = 0; i < HISTORY_WIDE; i++)
	{
		Serial.printf("%7d ", 1000000/(timeLeftBin + i * BIN_STEP_SIZE));
	}
	
	Serial.println();
	
	Serial.printf("%10s ", "count");
	for (i = 0; i < HISTORY_WIDE; i++)
	{
		Serial.printf("%7d ", history [i]);
	}

	Serial.printf("\n\nfaster than %6d baud = %d \n"
				      "slower than %6d baud = %d"
				     "\n----------------\n", 
			1000000/(timeLeftBin), 
			farLeft,
			1000000/(timeLeftBin + (HISTORY_WIDE) * BIN_STEP_SIZE),
			farRight);
			
			Serial.printf(FG_YELLOW "preambleCtr: %d\n" FG_DONE, preambleCtr);
}

//------------------------------------------
// 01 01 11 11 x 125 repetitions by bits.
// 0x5F repetitions over 15 bytes (125/8)

#define M_WIDE 48 
//#define M_MATCH (0xF5FF7FFF00000000 ) //<< (64 - M_WIDE))
#define M_MATCH (0x5555555500000000 ) //<< (64 - M_WIDE))
#define M_MASK  (0xFFFFFFFF00000000 ) // << (64 - M_WIDE))

void findPreamble(void)
{
	uint64_t lshifter = bigShifter[BIG_SHIFT-1];
	
	if ((lshifter & M_MASK) == M_MATCH)
	{	
		preambleCtr++;
		memcpy(keptMatch, bigShifter, sizeof(keptMatch));
	}
}

//------------------------------------------
void shift640(bool carryIn)
{
	for (int j = 0; j < BIG_SHIFT; j++)
	{
		bool carryOut;
		carryOut = !!(bigShifter[j] & 0x8000000000000000);
		bigShifter[j] <<=1;
		bigShifter[j] |= carryIn;
		carryIn = carryOut;
	}
}
//------------------------------------------


#define TSLICE (BAUD_4800uS -20)

static volatile uint32_t sharedLastTime;

uint32_t isrGDO0Jitter;
uint32_t isrGDO2Jitter;
 
ICACHE_RAM_ATTR void goGDO0_IRQ(void)
{
	uint32_t delta;
	
	uint32_t now = micros();
	
	irqGDO0ctr++;
	
	delta = now - sharedLastTime;

	statsCapture(delta);	// if needed

	if (delta < TSLICE) return;

	sharedLastTime = now; // stops other side from tripping.

	uint8_t pin0 = digitalRead(GDO0);
	uint8_t pin2 = digitalRead(GDO2);
	uint32_t end = delta/TSLICE;
	assert(end);

	for(int j=0; j < end; j++)
	{
		shift640(pin0); // what is the order for this?
		shift640(pin2); // what is the order for this?
	}
	
	isrGDO0Jitter = delta;
	findPreamble();

	
	
}
//-----------------------------------------------

ICACHE_RAM_ATTR void goGDO2_IRQ(void)
{
	uint32_t delta;
	
	uint32_t now = micros();
	
	irqGDO2ctr++;
	
	delta = now - sharedLastTime;

	if (delta < TSLICE) return;

	sharedLastTime = now; // stops other side from tripping.

	uint8_t pin0 = digitalRead(GDO0);
	uint8_t pin2 = digitalRead(GDO2);
	
	uint32_t end = delta/TSLICE;
	assert(end);

	for(int j=0; j < end; j++)
	{
		shift640(pin0); // what is the order for this?
		shift640(pin2); // what is the order for this?
	}
	
	isrGDO2Jitter = delta;
	findPreamble();


}	

//-----------------------------------------------
#if 0
hw_timer_t *hwTimer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrTimerCnt = 0;
volatile uint32_t isrTimerJitter = 0;
uint64_t isrHwTimerValue;

void ARDUINO_ISR_ATTR onTimer() 
{
  static uint32_t lastTime;
  uint32_t now;
  
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);

  isrTimerCnt = isrTimerCnt + 1;
  now = micros();
  isrTimerJitter = now - lastTime;
  lastTime = now;

  portEXIT_CRITICAL_ISR(&timerMux);

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);

  // It is safe to use digitalRead/Write here if you want to toggle an output
  
}

void setupTimers(void)
{
	// Create semaphore to inform us when the hwTimer has fired
	timerSemaphore = xSemaphoreCreateBinary();

	// Set hwTimer frequency to 1Mhz
	hwTimer = timerBegin(1000000);

	// Attach onTimer function to our hwTimer.
	timerAttachInterrupt(hwTimer, &onTimer);

	// Set alarm to call onTimer function every second (value in microseconds).
	// Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).

	timerAlarm(hwTimer,
			   104/2,     // time in uS
			   true,	// auto reload
			   0		// forever.
			   );	
	Serial.printf("sssssssssssssssssss %d\n", timerGetFrequency(hwTimer));

}
#endif

//--------------------------------

void layer1(void)
{
	
    radio.EnterIdleMode();

    //radio.setFreqHz(866887500);
    radio.setFreqHz(866988000);		// nice looking p25?

	radio.setSyncMode(0);	// no sync RAW
	radio.setPQT(0);			 
    
    //fixed packet length
    //packet size NOT inside packet
    //packet size set above in setPacketLength(blah);
    
    radio.setPktFormat(3);       // async, raw read pins - fifo disabled
    radio.setLengthConfig(0xFF);

    radio.setBaudRate(4800);
    radio.setDeviation(1.8); // 1.8k

    radio.setModulation(3); //fsk-4
	radio.setFEC(0);
	radio.setCrc(0);
	
	radio.setRxBW(58.3);
	
	radio.setRxOffMode(3); // stay in RX mode
	radio.setRxIF(152344.);
	

	radio.setGDOxPinConfig(CONFIG_IOCFG0, 23 ); //dibit lo
	radio.setGDOxPinConfig(CONFIG_IOCFG2, 22 ); //dibit hi 

    radio.setAGCHysteresis(2); 	// medium
    radio.setAGCWaitTime(16);	// small wait before doing something
    radio.setAGCFreezeAlgo(3);	// HOLD ! //normal agc. adjust as need
    radio.setAGCLength(16);		// avg 16 samples of amplitude.
	//assert (radio.SpiReadReg(CONFIG_AGCCTRL0) == 0x91);

    radio.setCarrierSenseAbs(0);
    radio.setCarrierSenseRel(0);
    radio.setLnaStrategy(1);
	assert (radio.SpiReadReg(CONFIG_AGCCTRL1) == 0x40);
    
    radio.setMAGNTarget(35);
    radio.setMaxLnaGain(0); 	//max no limits
    radio.setMaxDvgaGain(1);	//first highest gain cannot be used.
	assert (radio.SpiReadReg(CONFIG_AGCCTRL2) == 0x43);

	radio.setFOClimit(1);	//  rxbw/8  max change
	
	radio.enableChangingIRQ_GDO0(true, goGDO0_IRQ);
	radio.enableChangingIRQ_GDO2(true, goGDO2_IRQ);
    
    radio.EnterRxMode();

    while (!Serial.available())
    {
    	Serial.printf("GDO0= %d GDO2=%d deltaF %f \n", 
    				   irqGDO0ctr, irqGDO2ctr, radio.getCarrierDev());
    				   
		radio.getState(false);
		show();
    	delay(1000);

		//Serial.printf(FG_YELLOW "Unsigned: %" PRIu64 "\n" FG_DONE, bigShifter[0]);
		Serial.printf("bigShifter[%d]: 0x%" PRIX64 "\n",BIG_SHIFT-1, bigShifter[BIG_SHIFT-1]);
		Serial.printf("keptMatch : 0x%" PRIX64 "\n", keptMatch[BIG_SHIFT-1]);

		uint64_t x;
		x  = M_MATCH;
		Serial.printf("match     : 0x%" PRIX64 "\n", x);
		
		x = M_MASK;
		Serial.printf("mask      : 0x%" PRIX64 "\n", x);

		//Serial.printf("Signed: %" PRId64 "\n", read64);
	}
	Serial.read();

	radio.EnterIdleMode();
	
	radio.enableChangingIRQ_GDO0(false, goGDO0_IRQ);
	radio.enableChangingIRQ_GDO2(false, goGDO2_IRQ);

    // setting normal pkt format again
    radio.setCCMode(GDO0_isSYNC_TXEND);
    radio.setPktFormat(0);


}

/*
 Repeat hwTimer example

 This example shows how to use hardware hwTimer in ESP32. The hwTimer calls onTimer
 function every second. The hwTimer can be stopped with button attached to PIN 0
 (IO0).

 This example code is in the public domain.

// Stop button is attached to PIN 0 (IO0)
#define BTN_STOP_ALARM 0

hw_timer_t *hwTimer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrTimerCnt = 0;
volatile uint32_t lastIsrAt = 0;

void ARDUINO_ISR_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrTimerCnt = isrTimerCnt + 1;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void setup() {
  Serial.begin(115200);

  // Set BTN_STOP_ALARM to input mode
  pinMode(BTN_STOP_ALARM, INPUT_PULLUP);

  // Create semaphore to inform us when the hwTimer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Set hwTimer frequency to 1Mhz
  hwTimer = timerBegin(1000000);

  // Attach onTimer function to our hwTimer.
  timerAttachInterrupt(hwTimer, &onTimer);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(hwTimer, 1000000, true, 0);
}

void loop() {
  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrTimerCnt;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);
    // Print it
    Serial.print("onTimer no. ");
    Serial.print(isrCount);
    Serial.print(" at ");
    Serial.print(isrTime);
    Serial.println(" ms");
  }
  // If button is pressed
  if (digitalRead(BTN_STOP_ALARM) == LOW) {
    // If hwTimer is still running
    if (hwTimer) {
      // Stop and free hwTimer
      timerEnd(hwTimer);
      hwTimer = NULL;
    }
  }
}

void setup() {
  Serial.begin(115200); // Initialize serial communication

}

void loop() {
  // Check if the semaphore has been given by the ISR
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    // This code runs in the loop context, not the ISR context
    Serial.println("Timer interrupt triggered!");
    // You can safely use Serial functions here
  }
  // Your main loop code continues here
}

*/




/* PARKING LOT 
	radio.setGDO2_hostpinMode(INPUT);

	radio.setGDOxPinConfig(CONFIG_IOCFG2, INPUT);

	radio.setSyncWord(0x75, 0x5F);
	radio.setSyncMode(1); //1// of 16 bits ok
	// sigh radio.setPQT(3);

	radio.setPacketLength(CC_FIFOSIZE);

	radio.setGDOxPinConfig(CONFIG_IOCFG2, 0x6); // flag sync-eop
	radio.enableChangingIRQ_GDO2(true);

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

*/

