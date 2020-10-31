#include <SPI.h>

void setup() {
  Serial.begin(115200);
  SPI.begin();
  SPI.setCS(10);
  SPI0_MCR = SPI_MCR_PCSIS(0x1F); //Slave mode
  pinMode(12, INPUT);
  pinMode(11, OUTPUT);
  pinMode(10, INPUT);
  pinMode(13, INPUT);
}

void loop() {
  Serial.println(SPI0_POPR, BIN);
}
