#define MAIN_PROCESSOR

#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/BMP388/BMP388.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/BMP388/BMP388.cpp"

void setup() {
  Serial.begin(115200);
  DBMP388.begin();
  DBMP388.calibrateSeaLevel(610.0);
}

void loop() {
  DBMP388.readSensor();
  Serial.print(DBMP388.getRawAltitude()*1000.0);
  Serial.print(",");
  Serial.println(DBMP388.getFilteredAltitude()*1000.0);
}
