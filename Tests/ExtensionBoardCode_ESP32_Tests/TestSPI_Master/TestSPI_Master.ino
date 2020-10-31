#include <SPI.h> 
#define CS 5

SPISettings SPI_Settings(1000000, MSBFIRST, SPI_MODE0);

SPIClass * vspi = NULL;


void setup() {

  vspi = new SPIClass(VSPI);

  // initialize SPI:
  vspi->begin();
  // set the Slave Select Pins as outputs:
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
}

uint16_t val = 4; //send anything

void loop() {
  vspi->beginTransaction(SPI_Settings);
  digitalWrite(CS, LOW);
  vspi->transfer16(val);
  digitalWrite (CS, HIGH);
  vspi->endTransaction();
  delay(500);
}
