
/*
 * ALgo esta muy mal aqui
 */

#define MAIN_PROCESSOR

#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Comunication/DSPIV3.h>
#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Comunication/DSPIV3.cpp>

#define EXTENSION_CS 10

uint8_t dataSent[1] = {2};
uint8_t dataRecived[1];

//DroneSPISettings settings();

void setup() {
  pinMode(EXTENSION_CS, OUTPUT);
  digitalWrite(EXTENSION_CS, HIGH);

  Serial.begin(115200);
  DSPI.begin();
  DSPI.setUseChecksum(false);
  DSPI.configureSPIsettings(&DroneSPISettings(1000000, MSBFIRST, SPI_MODE0));
}

void loop() {
  DSPI.beginTransaction();
  digitalWrite(EXTENSION_CS, LOW);
  DSPI.transfer(dataSent, 1, dataRecived, 1);
  digitalWrite(EXTENSION_CS, HIGH);
  DSPI.endTransaction();
  Serial.println("Sent");
  delay(500);
}

