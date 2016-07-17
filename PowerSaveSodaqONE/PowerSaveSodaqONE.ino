void setup() 
{
  pinMode(ENABLE_PIN_IO, OUTPUT);
  digitalWrite(ENABLE_PIN_IO, HIGH);

  delay(5000);

  Serial1.begin(57600);
  Serial1.println("sys sleep 30000");
  delay(100);
  
  pinMode(GPS_ENABLE, OUTPUT);
  digitalWrite(GPS_ENABLE, LOW);

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __WFI();
}

void loop() 
{
}

