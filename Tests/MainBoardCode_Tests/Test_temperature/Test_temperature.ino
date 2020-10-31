float temp;

void setup() {
  Serial.begin(57600);
};

void loop () {
  temp = analogRead(17)*3.3/1024.0;
  temp = temp - 0.5;
  temp = temp / 0.01;
  Serial.println(temp);
  delay(500);
};
