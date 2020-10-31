#define MAIN_PROCESSOR

#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Processor.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Processor.cpp"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/SerialHelper.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/SerialHelper.cpp"

void setup() {
  Serial.begin(115200);
  delay(5000);
  DProcessor.begin();
}

void loop() {
  DProcessor.updateProcessor();
  DSerialHelper.printDouble(DProcessor.getVrefVoltage(), 10000);
  Serial.print(", in ");
  Serial.print(DSerialHelper.getLastExecutionTime());
  Serial.println("us");
  delay(100);
}
