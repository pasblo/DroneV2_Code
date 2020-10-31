#define MAIN_PROCESSOR

#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Storage/Flash.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Storage/Flash.cpp"

uint8_t flashData[8] = {2, 3, 0, 5, 2, 0, 0, 1};

void setup() {
  Serial.begin(115200);
  delay(5000);
  if(!DFlash.begin()){
    Serial.println("Error at conecting to the Flash");
    delay(100000);
  }
  DFlash.writeByteArray(flashData, 8);
}

void loop() {
  if(DFlash.writeNext()){
    Serial.print("Written in: ");
    Serial.print(DFlash.getRuntime());
    Serial.println("us");
  }
}
