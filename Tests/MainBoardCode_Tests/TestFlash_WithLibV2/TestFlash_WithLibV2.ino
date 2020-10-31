#define MAIN_PROCESSOR

#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Storage/Flash.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Storage/Flash.cpp"

struct Data{
  uint16_t test1;
};

void setup() {
  Serial.begin(115200);
  delay(5000);
  if(!DFlash.begin()){
    Serial.println("Error at conecting to the Flash");
    delay(100000);
  }
  Data testData;
  testData.test1 = 4989;
  DFlash.writeStruct(testData);
}

void loop() {
  if(!DFlash.writeNext()){
    Serial.println("Error in the writing to the flash");
    /*Serial.print("Written in: ");
    Serial.print(DFlash.getRuntime());
    Serial.println("us");*/
  }
}
