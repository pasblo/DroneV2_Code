#include "C:\\Users\\pabri\\Desktop\\Electronica\\DronProyect\\V2\\DroneV2_Code\\MainBoardCode\\DroneTime.h"
#include "C:\\Users\\pabri\\Desktop\\Electronica\\DronProyect\\V2\\DroneV2_Code\\MainBoardCode\\DroneTime.cpp"
#include <TimeController.h>

// Creating the instances of the time controller object and the fps controller variable type
TimeController timeController = TimeController();

fpsController_t controller;

void setup() {
  Serial.begin(115200);
  timeController.initializeFPSController(&controller);
  timeController.addTimedFunction(&printTimeOnScreen, 10);
}

void printTimeOnScreen(){
  droneDateTime_t dtFormat;
  uint64_t testTime = micros();
  DroneTime::obtainDroneTimestamp(&dtFormat, testTime*1000000);
  Serial.print(dtFormat.date.year);
  Serial.print("/");
  Serial.print(dtFormat.date.month);
  Serial.print("/");
  Serial.print(dtFormat.date.dayOfTheMonth);
  Serial.print("  ");
  Serial.print(dtFormat.time.hour);
  Serial.print(":");
  Serial.print(dtFormat.time.minute);
  Serial.print(":");
  Serial.print(dtFormat.time.second);
  Serial.print(":");
  Serial.print(dtFormat.time.millisecond);
  Serial.print(":");
  Serial.println(dtFormat.time.microsecond);
}

void loop() {
  timeController.controlFPS(&controller);

}
