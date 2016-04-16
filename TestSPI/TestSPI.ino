// External SPI Test

#include <SPI.h>

#define CONSOLE_SERIAL SerialUSB

void setup() 
{
  delay(5000);

  CONSOLE_SERIAL.println("Testing MPU9250 on SPI");
  
  SPI.begin();
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);

  // This a test with the MPU9250
  // Requires SPI to use 1Mhz MSBFIRST MODE0
  SPISettings settings(1000000, MSBFIRST, SPI_MODE0);
  
  // Register 0x75 should read 0x71 each time (Read flag 0x80)
  for (int i = 0; i < 100; i++) {
    SPI.beginTransaction(settings);
    digitalWrite(SS, LOW);
    
    SPI.transfer(0x75 | 0x80);
    CONSOLE_SERIAL.println(String(SPI.transfer(0x00), HEX));

    digitalWrite(SS, HIGH);
    SPI.endTransaction();
    
    delay(500);
  }
}

void loop() 
{
}
