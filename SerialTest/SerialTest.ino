// Serial Echo Test

void setup() 
{
  // Start the UART
  Serial.begin(9600);
}

void loop() 
{
  // Simply echo the data
  while(Serial.available()) {
    Serial.write(Serial.read());
  }
}
