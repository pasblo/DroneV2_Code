#define MAIN_PROCESSOR

#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Comunication/DSPIV4.h>
#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Comunication/DSPIV4.cpp>

void setup() {
  Serial.begin(115200);
  DSPI.begin(1000000, MSBFIRST, SPI_MODE0);

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
}

void loop() {
  DSPI.transferInformation(10, 1);
  delay(500);
}
