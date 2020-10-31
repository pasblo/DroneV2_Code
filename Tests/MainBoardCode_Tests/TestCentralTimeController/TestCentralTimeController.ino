#include <C:\\Users\\pabri\\Desktop\\Electronica\\DronProyect\\V2\\DroneV2_Code\\MainBoardCode\\InternalTime.h>

long firstMicros;
void setup(){
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  startCentralTimer();
  firstMicros = micros();
  digitalWrite(13, HIGH);
  
}

void loop(){
  Serial.print("First: ");
  Serial.println((long) getMicroseconds());
  Serial.print("Second: ");
  Serial.println(micros() - firstMicros);
  delay(1000);
}

