#define MAIN_PROCESSOR

#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/Current.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/Current.cpp"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Processor.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Processor.cpp"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/Temperature.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/Temperature.cpp"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/SerialHelper.h"

void setup() {
  DProcessor.setADCMode(HIGH_RESOLUTION_MODE);
  DProcessor.begin();
}

void loop() {
  DProcessor.updateProcessor();
  DCurrent.updateCurrentSensors();
  DTemperature.updateTemperatureSensors();
  Serial.print("Current used: ");
  DSerialHelper.printDouble(DCurrent.getPCBCurrent(), 6);
  Serial.print("mA, processor voltage: ");
  Serial.print(DProcessor.getVrefVoltage());
  Serial.print("V, pcb temperature: ");
  Serial.print(DTemperature.getPCBTemperature());
  Serial.println("ºC");
  delay(10);
}
