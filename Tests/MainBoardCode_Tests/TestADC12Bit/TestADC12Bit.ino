void setup() {
  Serial.begin(115200);
  analogReadRes(12);
  //analogReadAveraging(0);

}

void loop() {
  Serial.println(analogRead(17));
  delay(100);
}
