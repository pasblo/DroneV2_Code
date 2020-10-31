#define SPI_MASTER
#define USE_SPI0

#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Comunication/DSPI.h>
#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Comunication/DSPI.cpp>

void setup() {
  DSPI.begin(1000000, SPI);
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
}

void loop() {
  DSPI.transfer16Word(3, 10);
  delay(500);
}
