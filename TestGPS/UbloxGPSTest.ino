
#include "Arduino.h"
#include <Wire.h>
#include "ublox.h"
#include <stdio.h>
#include <stdarg.h>
#include "MyTime.h"

#define loraSerial Serial1
#define debugSerial SerialUSB

enum LedColor {
  RED = 1,
  GREEN,
  BLUE
};


uint8_t buffer[128];
uint32_t epoch   = 0;
uint32_t uptime  = 0;
uint32_t pulseat = 0;

UBlox uBlox;
Time myTime(60,"CET",2,0,Mar,Last,Sun,"CEST",3,0,Oct,Last,Sun);

void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt) {

  uBlox.db_printf("%4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d.%d valid=%2.2x lat=%d lon=%d sats=%d fixType=%2.2x\n",
                  NavPvt->year, NavPvt->month, NavPvt->day,
                  NavPvt->hour, NavPvt->minute, NavPvt->seconds, NavPvt->nano, NavPvt->valid,
                  NavPvt->lat, NavPvt->lon, NavPvt->numSV, NavPvt->fixType);

  if ((NavPvt->valid & 3) == 3 && (epoch == uptime || (NavPvt->minute == 0 && NavPvt->seconds == 0)) ) {     // Valid date & time
      epoch = myTime.mktime(NavPvt->year,NavPvt->month,NavPvt->day,NavPvt->hour,NavPvt->minute,NavPvt->seconds);
      uBlox.db_printf("set epoch=%d\n", epoch);
      myTime.dstwindow(NavPvt->year);
  }
}

void scani2c() {

	int nDevices = 0;
	for (int address = 1; address < 127; address++) {
		Wire.beginTransmission(address);
		int error = Wire.endTransmission();
		if (error == 0) {
			SerialUSB.print("I2C device found at address 0x");
			if (address < 16)
				SerialUSB.print("0");
			SerialUSB.print(address, HEX);
			SerialUSB.println("  !");
			nDevices++;
		}
		else if (error == 4) {
			SerialUSB.print("Unknow error at address 0x");
			if (address < 16)
				SerialUSB.print("0");
			SerialUSB.println(address, HEX);
		}
	}
	if (nDevices == 0)
		SerialUSB.println("No I2C devices found\n");
	else
		SerialUSB.println("done\n");

}

// Fast way to manipulate RGB leds
void Led(uint8_t color) {
	// Switch Off
	REG_PORT_OUTSET0 = (uint32_t) 0x8000;     // RED
	REG_PORT_OUTSET1 = (uint32_t) 0x0c00;     // GREEN & BLUE
	// Switch Requested color on
	if (color == RED)
		REG_PORT_OUTCLR0 = (uint32_t) 0x8000;   // RED
	else if (color == GREEN)
		REG_PORT_OUTCLR1 = (uint32_t) 0x0400;   // GREEN
	else if (color == BLUE)
		REG_PORT_OUTCLR1 = (uint32_t) 0x0800;   // BLUE

}

void ISR () {
  epoch++;
  uptime++;
  pulseat = millis();
}

void setup() {
	while (!SerialUSB && millis() < 7500);
	delay(500);
	//
	pinMode(LED_RED, OUTPUT);
	pinMode(LED_GREEN, OUTPUT);
	pinMode(LED_BLUE, OUTPUT);
	Led(0);
	//
	uBlox.enable();
	delay(100);												//Give it some time ...
	attachInterrupt(GPS_TIMEPULSE, ISR, RISING);
	//
	Wire.begin();
	// Wire.setClock(400000);
	scani2c();
	delay(100);
	//
	uBlox.flush();
	//
	TimePulseParameters xnew;
	if (uBlox.getTimePulseParameters(0, &xnew)) {
		xnew.freqPeriod = 1000000;
		xnew.pulseLenRatio = 100000;
		uBlox.db_printf(
				">>freqPeriod=%d freqPeriodLock=%d pulseLenRatio=%d pulseLenRatioLock=%d flags=%4.4x\n",
				xnew.freqPeriod, xnew.freqPeriodLock, xnew.pulseLenRatio,
				xnew.pulseLenRatioLock, xnew.flags);
		uBlox.setTimePulseParameters(&xnew);
		//
		memset(&xnew, 0, sizeof(TimePulseParameters));      // Wipe structure
		if (uBlox.getTimePulseParameters(0, &xnew)) 		// Reread TimePulseParameters
			uBlox.db_printf(
					"<<freqPeriod=%d freqPeriodLock=%d pulseLenRatio=%d pulseLenRatioLock=%d flags=%4.4x\n",
					xnew.freqPeriod, xnew.freqPeriodLock, xnew.pulseLenRatio,
					xnew.pulseLenRatioLock, xnew.flags);
	}

	PortConfigurationDDC pcd;
	if (uBlox.getPortConfigurationDDC(&pcd)) {
		uBlox.db_printf(
				"portID=%x txReady=%x mode=%x inProtoMask=%4.4x outProtoMask=%4.4x flags=%4.4x reserved5=%x\n",
				pcd.portID, pcd.txReady, pcd.mode, pcd.inProtoMask,
				pcd.outProtoMask, pcd.flags, pcd.reserved5);
		pcd.outProtoMask = 1;           					// Disable NMEA
		uBlox.setPortConfigurationDDC(&pcd);
	}
	else
		uBlox.db_printf("uBlox.getPortConfigurationDDC(&pcd) == false\n");

	uBlox.CfgMsg(UBX_NAV_PVT, 1);   						// Navigation Position Velocity TimeSolution
	uBlox.funcNavPvt = delegateNavPvt;
	//
	//
	struct tmx tm1,tm2;

	myTime.dstwindow(2016);                                  // Set Initial year
	myTime.localtime (myTime.dstfirst(),&tm1);
	myTime.localtime (myTime.dstlast(),&tm2);

	uBlox.db_printf("%4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d - %4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d (%d %d)\n",
	        tm1.tm_year+1900,tm1.tm_mon+1,tm1.tm_mday,tm1.tm_hour,tm1.tm_min,tm1.tm_sec,
	        tm2.tm_year+1900,tm2.tm_mon+1,tm2.tm_mday,tm2.tm_hour,tm2.tm_min,tm2.tm_sec,
	        myTime.dstfirst(),myTime.dstlast());
	//
	//
    SerialUSB.println("Hit any key to enter loop or wait 5000 ms");
    uint32_t s = millis();
	while (SerialUSB.read() == -1 && millis() - s < 5000);
	// Waiting for assist data
	char c = SerialUSB.read();
	if (c == 0xaa) {
	    while (SerialUSB.read() == -1 && millis() - s < 100);   // Wait 100 ms
	    c = SerialUSB.read();
	    if (c == 0x55) {
	        SerialUSB.println("ready ...");
	        uint32_t s = millis();
	        while (millis() - s < 1000) {
	            if (SerialUSB.available()) {
	                char c = SerialUSB.read();
	                int id = uBlox.process(c);
	                if (id == 0xaa55) {
	                    // send packet to ublox ....
	                    uBlox.sendraw();
	                    s = millis();
	                }
	            }
	        }
	    }
	}
	//
	SerialUSB.println("Entering loop() now");
}

void loop() {
  static uint32_t _uptime = 0;
  if (SerialUSB.available()) {
    char c = SerialUSB.read();
    if (c == 'x') {
    	epoch = uptime;
    	SerialUSB.println("resynch");
    }
    else if (c == 's') {
      SerialUSB.println("Holding");
      while (SerialUSB.read() == -1);
    }
    else if (c == 'u') {
      PortConfigurationDDC pcd;
      if (uBlox.getPortConfigurationDDC(&pcd)) {
        uBlox.db_printf("portID=%x txReady=%x mode=%x inProtoMask=%4.4x outProtoMask=%4.4x flags=%4.4x reserved5=%x\n", pcd.portID, pcd.txReady, pcd.mode, pcd.inProtoMask, pcd.outProtoMask, pcd.flags, pcd.reserved5);
        pcd.outProtoMask ^= 2;   // NMEA toggle
        uBlox.setPortConfigurationDDC(&pcd);
      }
      else
        uBlox.db_printf("uBlox.getPortConfigurationDDC(&pcd) == false\n");
    }
  }
  //
  if (uptime != _uptime) {
	struct tmx t;
	uint32_t now = millis();
	myTime.localtime(epoch,&t);
    uBlox.db_printf("%4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d - Up since %d for %d seconds (%d)\n",t.tm_year+1900,t.tm_mon+1,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec,epoch - uptime, uptime,millis()-now);
    _uptime = uptime;
  }
  //
  uint32_t now = millis();
  if (now - pulseat > 300 && now - pulseat < 1000) {
	  uint16_t bytes = uBlox.available();
	  if (bytes) {
		  // uBlox.db_printf("Found bytes %d\n",bytes);
		  uBlox.GetPeriodic(bytes);
	  }
  }
  //
  Led((now / 250) & 3);
}
