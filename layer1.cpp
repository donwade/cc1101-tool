
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

#include "keyboard.h"
#include "bandcal.h"

uint32_t irqCnt0;
uint32_t irqCnt2;

//-----------------------------------------------
#define HISTORY_WIDE 9
uint32_t  history[HISTORY_WIDE];
uint32_t farLeft, farRight;

#define BAUD_LEFT 12000
#define BAUD_RIGHT 3500

const int timeLeftBin  = 1000000./BAUD_LEFT;
const int timeRightBin = 1000000./BAUD_RIGHT;
const int BIN_STEP_SIZE = (timeRightBin - timeLeftBin)/ HISTORY_WIDE;


void capture(uint32_t tick)
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
 	
}


//-----------------------------------------------
ICACHE_RAM_ATTR void goGDO0_IRQ(void)
{
	static uint32_t lastime;
	uint32_t diff;
	uint32_t now = micros();
	diff = now - lastime;
	lastime = now;
	
	capture(diff);
	
	irqCnt0++;
	
}
//-----------------------------------------------

ICACHE_RAM_ATTR void goGDO2_IRQ(void)
{
	irqCnt2++;
}
//-----------------------------------------------
bool bTimedOut;

hw_timer_t * timer = NULL; // Pointer to the hardware timer
volatile SemaphoreHandle_t timerSemaphore; // Semaphore to signal the loop from the ISR

// ISR (Interrupt Service Routine) callback function
void IRAM_ATTR onTimer() 
{
  bTimedOut = true;
  // Use a semaphore to safely communicate with the loop function
  xSemaphoreGiveFromISR(timerSemaphore, NULL); 
}
//-----------------------------------------------
 
void layer1(void)
{
    radio.EnterIdleMode();

    radio.setFreqHz(866887500);

	radio.setSyncMode(0);	// no sync RAW
	radio.setPQT(0);			 
    
    //fixed packet length
    //packet size NOT inside packet
    //packet size set above in setPacketLength(blah);
    
    radio.setPktFormat(3);       // async, raw read pins - fifo disabled
    radio.setLengthConfig(0xFF);

    radio.setBaudRate(9600);
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
    radio.setAGCFreezeAlgo(0);	// normal agc. adjust as need
    radio.setAGCLength(16);		// avg 16 samples of amplitude.
	assert (radio.SpiReadReg(CONFIG_AGCCTRL0) == 0x91);

    radio.setCarrierSenseAbs(0);
    radio.setCarrierSenseRel(0);
    radio.setLnaStrategy(1);
	assert (radio.SpiReadReg(CONFIG_AGCCTRL1) == 0x40);
    
    radio.setMAGNTarget(35);
    radio.setMaxLnaGain(0); 	//max no limits
    radio.setMaxDvgaGain(1);	//first highest gain cannot be used.
	assert (radio.SpiReadReg(CONFIG_AGCCTRL2) == 0x43);

	
	radio.enableChangingIRQ_GDO0(true, goGDO0_IRQ);
	radio.enableChangingIRQ_GDO2(true, goGDO2_IRQ);
    
    radio.EnterRxMode();

    while (!Serial.available())
    {
    	Serial.printf("tmr=%d i0= %d i2=%d\n", bTimedOut, irqCnt0, irqCnt2);
		radio.getState(false);
		show();
    	delay(1000);
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
 Repeat timer example

 This example shows how to use hardware timer in ESP32. The timer calls onTimer
 function every second. The timer can be stopped with button attached to PIN 0
 (IO0).

 This example code is in the public domain.

// Stop button is attached to PIN 0 (IO0)
#define BTN_STOP_ALARM 0

hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

void ARDUINO_ISR_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter = isrCounter + 1;
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

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Set timer frequency to 1Mhz
  timer = timerBegin(1000000);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer, 1000000, true, 0);
}

void loop() {
  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
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
    // If timer is still running
    if (timer) {
      // Stop and free timer
      timerEnd(timer);
      timer = NULL;
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

