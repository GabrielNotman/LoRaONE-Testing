// PWM Test

#include <SPI.h>

// Available on pins 2,3,4,5,6,7,8,9,10,11
// Also available inverted on LED_RED, LED_GREEN, LED_BLUE
// The following pairs share the same channel 8:4, 9:5
#define TEST_PIN  LED_BLUE
#define TEST_PIN2 5
#define CONSOLE_SERIAL SerialUSB

uint8_t intensity = 0;

void setup() 
{
  // To test ENABLE_PIN_IO
  pinMode(ENABLE_PIN_IO, OUTPUT);
  digitalWrite(ENABLE_PIN_IO, HIGH);

  delay(5000);
  
  pinMode(TEST_PIN, OUTPUT);

  CONSOLE_SERIAL.println(String("Testing PWM on pins: ") + TEST_PIN + " & " + TEST_PIN2);
  
  // Test to see if these operations interfere
  SPI.begin();
  Serial.begin(9600);
}

void loop() 
{
  analogWrite(TEST_PIN, intensity);
  analogWrite(TEST_PIN2, intensity - 128);
  intensity += 1;
  delay(10);
}
