// Interrupt Test

#include <SPI.h>

// Options for TEST_PIN:
// D0 - D13 (or A0-A13)
// BUTTON, GPS_TIMEPULSE, ACCEL_INT1, 
// ACCEL_INT2, SWITCH_SENSE

#define TEST_PIN 13
#define DEBOUNCE_MS 200
#define CONSOLE_SERIAL SerialUSB

volatile uint32_t lastInt = 0;
volatile bool int_flag = false;

void setup() 
{
  delay(5000);
  
  pinMode(TEST_PIN, INPUT);
  attachInterrupt(TEST_PIN, ISR, RISING);

  CONSOLE_SERIAL.println(String("Testing the interrupt on pin: ") + TEST_PIN);
  
  // Test to see if these operations interfere
  SPI.begin();
  //Serial.begin(9600);
}

void loop() 
{
  if (int_flag) {
    CONSOLE_SERIAL.println(String("Interrupt on: ") + TEST_PIN);
    int_flag = false;
  }
}

void ISR()
{
  if ((millis() - DEBOUNCE_MS) > lastInt) {
    lastInt = millis();
    int_flag = true;
  }
}

