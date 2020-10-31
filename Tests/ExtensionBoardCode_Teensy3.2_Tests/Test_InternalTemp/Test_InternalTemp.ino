int sensorValue = 0;  // variable to store the value coming from the sensor
float average = 0;

void setup() {
  analogReference(INTERNAL);
  analogReadResolution(12);
  Serial.begin(115200);
}

void loop() {
   for (byte counter =0;counter<255;counter++){    
  average = analogRead(38)+average;  
  }
  average= average/255;
  float C = 25.0 + 0.17083 * (2454.19 - average);
  Serial.print(average);
  Serial.print(' ');
  Serial.println(C);
  average =0;
  delay(255);
}
