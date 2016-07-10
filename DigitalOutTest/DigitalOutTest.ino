// Digital Output Test

#include <SPI.h>

#define NUM_PINS 12
#define NUM_TOGGLES 5
#define CONSOLE_SERIAL SerialUSB

void setup() 
{
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    pinMode(i, OUTPUT);
  }

  // Test to see if these operations interfere
  SPI.begin();
  Serial.begin(9600);
}

void loop() 
{
  CONSOLE_SERIAL.println();
  
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    for (uint8_t j = 0; j < NUM_TOGGLES; j++) {
      CONSOLE_SERIAL.println(String("Pin ") + i + ": HIGH");
      digitalWrite(i, HIGH);      
      delay(2000);
      CONSOLE_SERIAL.println(String("Pin ") + i + ": LOW");
      digitalWrite(i, LOW);
      delay(2000);
    }
  }
}
