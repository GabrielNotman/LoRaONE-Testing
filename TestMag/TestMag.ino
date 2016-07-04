// Accel Test

#include <Wire.h>

#define CONSOLE_SERIAL SerialUSB

#define ACCEL_ADR 0b0011110
#define TEST_REG 0x0F

#define TEST_INT1
#define TEST_INT2

volatile bool int1_flag = false;
volatile bool int2_flag = false;

void setup() 
{
  delay(5000);
  Wire.begin();

  CONSOLE_SERIAL.println("Testing Magnetometer");

  // Reboot 
  // CTRL0
  writeReg(0x1F, 0b10000000);

  // Enable Temperature, High Resolution, 50hz, no interrupts
  // CTRL5
  writeReg(0x24, 0b10010000);

  // Magnetic sensor in Continous-conversion mode
  // CTRL7
  writeReg(0x26, 0b00000000);
}

void loop()
{
  int16_t x_val = (readReg(0x09) << 8) | readReg(0x08);
  int16_t y_val = (readReg(0x0B) << 8) | readReg(0x0A);
  int16_t z_val = (readReg(0x0D) << 8) | readReg(0x0C);
  CONSOLE_SERIAL.println(String("Magnetometer Readings: ") + x_val + ", " + y_val + ", " + z_val);

  int16_t t_val = (readReg(0x06) << 8) | readReg(0x05);
  CONSOLE_SERIAL.println(String("Temperature Reading: ") + t_val);
 
  for (int i=0; i<1000; i++) {
    delayMicroseconds(1000);
  }
}

uint8_t readReg(uint8_t reg)
{
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ACCEL_ADR, 0x01);
  
  uint8_t val = Wire.read();
  Wire.endTransmission();

  return val;
}

uint8_t writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delayMicroseconds(10000);
}

