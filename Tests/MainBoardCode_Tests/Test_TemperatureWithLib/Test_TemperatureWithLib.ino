#define MAIN_PROCESSOR

#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/Temperature.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/Temperature.cpp"

void setup() {
  Serial.begin(115200);
  delay(5000);
  DTemperature.begin();
}

void loop() {
  DTemperature.updateTemperatures();
  Serial.print("PCB temp: ");
  Serial.print(DTemperature.getPCBTemperature());
  Serial.print(", Processor temp: ");
  Serial.print(DTemperature.getProcessorTemperature());
  Serial.print(", calculated in: ");
  Serial.print(DTemperature.getLastExecutionTime());
  Serial.println(" us");
  delay(100);
}
