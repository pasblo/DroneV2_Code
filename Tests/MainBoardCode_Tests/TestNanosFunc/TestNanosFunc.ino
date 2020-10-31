long beginMillis;

void setup() {
  Serial.begin(115200);
  delay(5000);
  beginMillis = millis();
}

void loop() {
  if(millis() - beginMillis < 100){
    Serial.print(micros());
    Serial.print(", ");
    print64(nanos());
    Serial.println();
  }
}

void print64(uint64_t value)
{
    if ( value >= 10 )
    {
        print64(value / 10);
    }
   
    Serial.print((byte) value % 10);
}
