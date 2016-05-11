/*************************************************** 
  This is a library for the Adafruit BMP085/BMP180 Barometric Pressure + Temp sensor

  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Sodaq_BMP085.h"

#define BMP085_DEBUG 0
#define BMP085_ENABLE_DIAG 0

#define DEBUG_SERIAL Serial

#define BMP085_I2CADDR 0x77

#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD       0x2E
#define BMP085_READPRESSURECMD   0x34

Sodaq_BMP085::Sodaq_BMP085()
{
  oversampling = BMP085_ULTRAHIGHRES;
}

void Sodaq_BMP085::begin(uint8_t mode)
{
  if (mode > BMP085_ULTRAHIGHRES)
    mode = BMP085_ULTRAHIGHRES;
  oversampling = mode;

  Wire1.begin();

  if (read8(0xD0) != 0x55)
    return;

  /* read calibration data */
  ac1 = read16(BMP085_CAL_AC1);
  ac2 = read16(BMP085_CAL_AC2);
  ac3 = read16(BMP085_CAL_AC3);
  ac4 = read16(BMP085_CAL_AC4);
  ac5 = read16(BMP085_CAL_AC5);
  ac6 = read16(BMP085_CAL_AC6);

  b1 = read16(BMP085_CAL_B1);
  b2 = read16(BMP085_CAL_B2);

  mb = read16(BMP085_CAL_MB);
  mc = read16(BMP085_CAL_MC);
  md = read16(BMP085_CAL_MD);
#if BMP085_ENABLE_DIAG
  DEBUG_SERIAL.print("ac1 = "); DEBUG_SERIAL.println(ac1, DEC);
  DEBUG_SERIAL.print("ac2 = "); DEBUG_SERIAL.println(ac2, DEC);
  DEBUG_SERIAL.print("ac3 = "); DEBUG_SERIAL.println(ac3, DEC);
  DEBUG_SERIAL.print("ac4 = "); DEBUG_SERIAL.println(ac4, DEC);
  DEBUG_SERIAL.print("ac5 = "); DEBUG_SERIAL.println(ac5, DEC);
  DEBUG_SERIAL.print("ac6 = "); DEBUG_SERIAL.println(ac6, DEC);

  DEBUG_SERIAL.print("b1 = "); DEBUG_SERIAL.println(b1, DEC);
  DEBUG_SERIAL.print("b2 = "); DEBUG_SERIAL.println(b2, DEC);

  DEBUG_SERIAL.print("mb = "); DEBUG_SERIAL.println(mb, DEC);
  DEBUG_SERIAL.print("mc = "); DEBUG_SERIAL.println(mc, DEC);
  DEBUG_SERIAL.print("md = "); DEBUG_SERIAL.println(md, DEC);
#endif
}

uint16_t Sodaq_BMP085::readRawTemperature(void)
{
  if ((ac1 == 0 || ac1 == -1) && (mb == 0 || mb == -1)) {
    // The device parameters were not read properly, or begin() was never executed
    begin(oversampling);
  }
  write8(BMP085_CONTROL, BMP085_READTEMPCMD);
  delayMicroseconds(5e3);
#if BMP085_ENABLE_DIAG
  DEBUG_SERIAL.print("Raw temp: "); DEBUG_SERIAL.println(read16(BMP085_TEMPDATA));
#endif
  return read16(BMP085_TEMPDATA);
}

uint32_t Sodaq_BMP085::readRawPressure(void)
{
  uint32_t raw;

  write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

  if (oversampling == BMP085_ULTRALOWPOWER)
    delayMicroseconds(5e3);
  else if (oversampling == BMP085_STANDARD)
    delayMicroseconds(8e3);
  else if (oversampling == BMP085_HIGHRES)
    delayMicroseconds(14e3);
  else
    delayMicroseconds(26e3);

  raw = read16(BMP085_PRESSUREDATA);

  raw <<= 8;
  raw |= read8(BMP085_PRESSUREDATA + 2);
  raw >>= (8 - oversampling);

  /* this pull broke stuff, look at it later? */
#if 0
  if (oversampling==0) {
    raw <<= 8;
    raw |= read8(BMP085_PRESSUREDATA+2);
    raw >>= (8 - oversampling);
  }
#endif

#if BMP085_ENABLE_DIAG
  DEBUG_SERIAL.print("Raw pressure: "); DEBUG_SERIAL.println(raw);
#endif
  return raw;
}

/*
 * Do some precalculation for temperature (B5)
 *
 * The manual (BMP085 Data Sheet Rev 1.2) says:
 *  X1 = (UT - AC6) * AC5 / 2^15
 *  X2 = MC * 2^11 / (X1 + MD)
 *  B5 = X1 + X2
 */
int32_t Sodaq_BMP085::computeB5(int32_t UT)
{
  int32_t X1;
  int32_t X2;
  int32_t B5;
  X1 = ((UT - ac6) * ac5) >> 15;
  X2 = ((int32_t) mc << 11) / (X1 + md);
  B5 = X1 + X2;

#if BMP085_ENABLE_DIAG
  DEBUG_SERIAL.print("X1 = "); DEBUG_SERIAL.println(X1);
  DEBUG_SERIAL.print("X2 = "); DEBUG_SERIAL.println(X2);
  DEBUG_SERIAL.print("B5 = "); DEBUG_SERIAL.println(B5);
#endif
  return B5;
}

int32_t Sodaq_BMP085::readPressure(void)
{
  int32_t UT;
  int32_t UP;
  int32_t B3;
  int32_t B5;
  int32_t B6;
  int32_t X1;
  int32_t X2;
  int32_t X3;
  int32_t p;
  uint32_t B4;
  uint32_t B7;

  UT = readRawTemperature();
  UP = readRawPressure();

#if BMP085_DEBUG == 1
  // use datasheet numbers!
  UT = 27898;
  UP = 23843;
  ac6 = 23153;
  ac5 = 32757;
  mc = -8711;
  md = 2868;
  b1 = 6190;
  b2 = 4;
  ac3 = -14383;
  ac2 = -72;
  ac1 = 408;
  ac4 = 32741;
  oversampling = 0;
#endif

  // do temperature calculations
  B5 = computeB5(UT);

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t) b2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t) ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t) ac1 * 4 + X3) << oversampling) + 2) / 4;

#if BMP085_ENABLE_DIAG
  DEBUG_SERIAL.print("B6 = "); DEBUG_SERIAL.println(B6);
  DEBUG_SERIAL.print("X1 = "); DEBUG_SERIAL.println(X1);
  DEBUG_SERIAL.print("X2 = "); DEBUG_SERIAL.println(X2);
  DEBUG_SERIAL.print("B3 = "); DEBUG_SERIAL.println(B3);
#endif

  X1 = ((int32_t) ac3 * B6) >> 13;
  X2 = ((int32_t) b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t) ac4 * (uint32_t) (X3 + 32768)) >> 15;
  B7 = ((uint32_t) UP - B3) * (uint32_t) (50000UL >> oversampling);

#if BMP085_ENABLE_DIAG
  DEBUG_SERIAL.print("X1 = "); DEBUG_SERIAL.println(X1);
  DEBUG_SERIAL.print("X2 = "); DEBUG_SERIAL.println(X2);
  DEBUG_SERIAL.print("B4 = "); DEBUG_SERIAL.println(B4);
  DEBUG_SERIAL.print("B7 = "); DEBUG_SERIAL.println(B7);
#endif

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

#if BMP085_ENABLE_DIAG
  DEBUG_SERIAL.print("p = "); DEBUG_SERIAL.println(p);
  DEBUG_SERIAL.print("X1 = "); DEBUG_SERIAL.println(X1);
  DEBUG_SERIAL.print("X2 = "); DEBUG_SERIAL.println(X2);
#endif

  p = p + ((X1 + X2 + (int32_t) 3791) >> 4);
#if BMP085_ENABLE_DIAG
  DEBUG_SERIAL.print("p = "); DEBUG_SERIAL.println(p);
#endif
  return p;
}

/*
 * Read and calculate true temperature
 *
 * The manual (BMP085 Data Sheet Rev 1.2) says:
 *  X1 = (UT - AC6) * AC5 / 2^15
 *  X2 = MC * 2^11 / (X1 + MD)
 *  B5 = X1 + X2
 *  T = (B5 + 8) / 2^4
 * and this is in units of 0.1 degrees Celcius
 */
float Sodaq_BMP085::readTemperature(void)
{
  int32_t UT, B5;
  float temp;

  UT = readRawTemperature();

#if BMP085_DEBUG == 1
  // use datasheet numbers!
  UT = 27898;
  ac6 = 23153;
  ac5 = 32757;
  mc = -8711;
  md = 2868;
#endif

  // step 1
  B5 = computeB5(UT);
  temp = (B5 + 8) >> 4;
  temp /= 10;

  return temp;
}

float Sodaq_BMP085::readAltitude(float sealevelPressure)
{
  float altitude;

  float pressure = readPressure();

  altitude = 44330 * (1.0 - pow(pressure / sealevelPressure, 0.1903));

  return altitude;
}


/*********************************************************************/

/*
 * Read a 1 byte register
 */
uint8_t Sodaq_BMP085::read8(uint8_t a)
{
  uint8_t ret;

  Wire1.beginTransmission(BMP085_I2CADDR);       // start transmission to device
  Wire1.write(a);                                // sends register address to read from
  Wire1.endTransmission();                       // end transmission

  Wire1.beginTransmission(BMP085_I2CADDR);       // start transmission to device
  Wire1.requestFrom(BMP085_I2CADDR, 1);          // send data n-bytes read
  ret = Wire1.read();
  Wire1.endTransmission();                       // end transmission

  return ret;
}

/*
 * Read a 2 byte register
 */
uint16_t Sodaq_BMP085::read16(uint8_t a)
{
  uint16_t ret;

  Wire1.beginTransmission(BMP085_I2CADDR);
  Wire1.write(a);
  Wire1.endTransmission();

  Wire1.beginTransmission(BMP085_I2CADDR);
  Wire1.requestFrom(BMP085_I2CADDR, 2);
  ret = Wire1.read();
  ret <<= 8;
  ret |= Wire1.read();
  Wire1.endTransmission();

  return ret;
}

/*
 * Write a 1 byte register
 */
void Sodaq_BMP085::write8(uint8_t a, uint8_t d)
{
  Wire1.beginTransmission(BMP085_I2CADDR);
  Wire1.write(a);
  Wire1.write(d);
  Wire1.endTransmission();
}
