// Digital Input Test

#include <SPI.h>

#define NUM_PINS 12
#define CONSOLE_SERIAL SerialUSB

void setup() 
{
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    pinMode(i, INPUT);
  }

  pinMode(BUTTON, INPUT);
  pinMode(SWITCH_SENSE, INPUT);

  digitalWrite(GPS_ENABLE, HIGH);
  pinMode(GPS_ENABLE, OUTPUT);
  pinMode(GPS_TIMEPULSE, INPUT);

  // Test to see if these operations interfere
  SPI.begin();
  Serial.begin(9600);
}

void loop() 
{
  CONSOLE_SERIAL.println();
  
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    CONSOLE_SERIAL.println(String("Pin ") + i + ": " + digitalRead(i));
  }

  CONSOLE_SERIAL.println(String("Pin BUTTON: ") + digitalRead(BUTTON));
  CONSOLE_SERIAL.println(String("Pin SWITCH_SENSE: ") + digitalRead(SWITCH_SENSE));
  CONSOLE_SERIAL.println(String("Pin GPS_TIMEPULSE: ") + digitalRead(GPS_TIMEPULSE));
  
  delay(1000);
}
