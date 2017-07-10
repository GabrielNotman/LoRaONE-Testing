// Accel Test

#include <Wire.h>
#include <Sodaq_LIS3DE.h>

#define CONSOLE_SERIAL SerialUSB

volatile bool int1_flag = false;
volatile bool int2_flag = false;

Sodaq_LIS3DE accelerometer;

void setup() 
{
  // Enable battery powered mode
  pinMode(ENABLE_PIN_IO, OUTPUT);
  digitalWrite(ENABLE_PIN_IO, HIGH);
  
  delay(5000);
  Wire.begin();

  CONSOLE_SERIAL.println("Startup");

  // LED pins
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  // Accelerometer INT pins
  pinMode(ACCEL_INT1, INPUT);
  pinMode(ACCEL_INT2, INPUT);
  attachInterrupt(ACCEL_INT1, ISR1, HIGH);
  attachInterrupt(ACCEL_INT2, ISR2, HIGH);

  accelerometer.enable(true, Sodaq_LIS3DE::NormalLowPower10Hz, Sodaq_LIS3DE::XYZ, Sodaq_LIS3DE::Scale4g, true);
  delay(10);

  // Set to 2% of range (2 * 8g / 100%)
  accelerometer.enableInterrupt1(Sodaq_LIS3DE::XHigh | Sodaq_LIS3DE::XLow | Sodaq_LIS3DE::YHigh | Sodaq_LIS3DE::YLow | Sodaq_LIS3DE::ZHigh | Sodaq_LIS3DE::ZLow, 2 * 8.0 / 100.0, 0, Sodaq_LIS3DE::MovementRecognition);
  accelerometer.enableInterrupt2(Sodaq_LIS3DE::XHigh | Sodaq_LIS3DE::XLow | Sodaq_LIS3DE::YHigh | Sodaq_LIS3DE::YLow | Sodaq_LIS3DE::ZHigh | Sodaq_LIS3DE::ZLow, 2 * 8.0 / 100.0, 0, Sodaq_LIS3DE::MovementRecognition);
}

void loop()
{
  digitalWrite(LED_RED, LOW);
  
  if (int1_flag) {
    int1_flag = false;
    CONSOLE_SERIAL.println("INT1 Interrupt");
    
    digitalWrite(LED_GREEN, LOW);
    delay(1000);
    digitalWrite(LED_GREEN, HIGH);
  }
  if (int2_flag) {
    int2_flag = false;
    CONSOLE_SERIAL.println("INT2 Interrupt");

    // Blink LED
    digitalWrite(LED_BLUE, LOW);
    delay(1000);
    digitalWrite(LED_BLUE, HIGH);
  }

  delay(5000);
  digitalWrite(LED_RED, HIGH);
}

void ISR1()
{
  int1_flag = true;
}

void ISR2()
{
  int2_flag = true;
}

