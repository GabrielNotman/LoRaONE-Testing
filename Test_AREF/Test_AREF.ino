#define A0_OUT 127
#define AREF1 3.3
#define AREF2 (3.3/255.0 * (float)A0_OUT)


void setup() 
{
  analogWrite(A0, A0_OUT);
}

void loop() 
{
  // On the SAMD platform you have to
  // preface the analogReference mode
  // with AR, e.g. AR_DEFAULT for DEFAULT
    
  analogReference(AR_DEFAULT);
  uint16_t a2 = analogRead(A2);
 
  analogReference(AR_EXTERNAL);
  uint16_t a3 = analogRead(A2);
  
  SerialUSB.println(String("Reading on A2, ") + AREF1 + "V Ref: " + ((float)a2) / 1023.0f * AREF1);
  SerialUSB.println(String("Reading on A3, ") + AREF2 + "V Ref: " + ((float)a3) / 1023.0f * AREF2);

  delay(2000);  
}


