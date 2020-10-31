unsigned long counter;
unsigned int lastMillis;
void setup() {
  Serial.begin(115200);
  lastMillis = 0;

}

void loop() {
  counter++;
  if(millis() - lastMillis >= 1000){
    Serial.println(counter);
    lastMillis = millis();
    counter = 0;
  }
}
