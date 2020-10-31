#define SPI_SLAVE
#define USE_SPI0

uint32_t lastTime;

#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Comunication/DSPI.h>
#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Comunication/DSPI.cpp>

void setup() {
  Serial.begin(115200);
  DSPI.begin(10, SPI);

  lastTime = millis();
}

void loop() {
  DSPI.deleteCache();
  if(millis() - lastTime > 1000){
    lastTime = millis();
    Serial.println((long) DSPI.getCounter());
    DSPI.resetCounter();
  }
}
