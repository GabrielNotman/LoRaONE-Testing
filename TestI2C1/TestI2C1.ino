// External I2C Test (I2C1)
// The included BMP085 & SHT2x libraries 
// have been modfied to use the Wire1 instance

#include <Wire.h>
#include "Sodaq_BMP085.h"
#include "Sodaq_SHT2x.h"

#define CONSOLE_SERIAL SerialUSB

// TPH BMP sensor
Sodaq_BMP085 bmp;

void setup() 
{
  delay(5000);

  CONSOLE_SERIAL.println("Testing TPH on I2C1");

  // Initialise the Wire1 bus
  Wire1.begin();

  // Initialise the TPH BMP sensor
  bmp.begin();
}

void loop() 
{
  CONSOLE_SERIAL.print("TPH Readings from I2C1: ");
  CONSOLE_SERIAL.println(getTPHData());
  delay(2000);
}

String getTPHData()
{
  String data;
  data = String(SHT2x.GetTemperature())  + ", ";
  data += String(bmp.readTemperature()) + ", ";
  data += String(bmp.readPressure() / 100)  + ", ";
  data += String(SHT2x.GetHumidity());
  
  return data;
}
