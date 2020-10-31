#define EXTERNAL_PROCESSOR

#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Comunication/DSPIV3.h>
#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Comunication/DSPIV3.cpp>

uint8_t dataSent[1] = {1};
uint8_t dataRecived[1];

void setup() {
  Serial.begin(115200);
  DSPI.begin();
  DSPI.setUseChecksum(false);
  DSPI.configureSPIsettings(&DroneSPISettings(MSBFIRST, SPI_MODE0));
}

void loop() {
  Serial.println(SPI0_PUSHR, BIN);
  DSPI.restockData(dataSent);
  Serial.println(SPI0_PUSHR, BIN);
  delay(1000);
  if(DSPI.dataAvailable(dataRecived)){
    DSPI.restockData(dataSent);
    Serial.println("1");
  }
}
