// Analog Test

#include <SPI.h>

#define NUM_PINS 14
#define DAC_PIN A0
#define CONSOLE_SERIAL SerialUSB

void setup() 
{
  // Analog out at 50% of 3.3V
  analogWrite(DAC_PIN, 127);

  // Test to see if these operations interfere
  SPI.begin();
  Serial.begin(9600);
}

void loop() 
{
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    if (i != (DAC_PIN - A0)) {
      CONSOLE_SERIAL.println(String("AIN Pin A") + i + ": " + analogRead(A0 + i));     
    }
    else {
      CONSOLE_SERIAL.println(String("Skipping DAC pin A") + i);
    }
  }
  delay(5000);
  CONSOLE_SERIAL.println();
}
