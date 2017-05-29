#include "RTCZero.h"
#include "Sodaq_RN2483.h"

#define loraSerial Serial1
#define debugSerial SerialUSB

#define DEBUG_BAUD 57600

RTCZero rtc;
volatile bool rtc_flag = false;

void setup()
{
  // Enable battery powered mode
  pinMode(ENABLE_PIN_IO, OUTPUT);
  digitalWrite(ENABLE_PIN_IO, HIGH);

  // Startup delay
  delay(5000);
  
  // Setup debug serial
  debugSerial.begin(DEBUG_BAUD);

  // Setup LoRa and put in sleep mode
  loraSerial.begin(LoRaBee.getDefaultBaudRate());
  LoRaBee.init(loraSerial, LORA_RESET);
  LoRaBee.sleep();
  loraSerial.flush();
  
  // Power down the GPS
  pinMode(GPS_ENABLE, OUTPUT);
  digitalWrite(GPS_ENABLE, LOW);

  // Setup blink LED 
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);

  // Setup RTC, every minute alarm
  rtc.begin(true);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(alarmMatch);

  //Set sleep mode
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

void loop()
{
  if(rtc_flag)
  {
    rtc_flag = false;
    
    LoRaBee.wakeUp();

    // Blink LED
    digitalWrite(LED_RED, LOW);
    delay(2000);
    digitalWrite(LED_RED, HIGH);

    LoRaBee.sleep();
    loraSerial.flush();
  }

  //Enter sleep mode
  __WFI();
}

void alarmMatch()
{
  rtc_flag = true;
}


