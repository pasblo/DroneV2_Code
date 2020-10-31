#include <SPI.h> 
#define CS 10
// set up the speed, mode and endianness of each device
SPISettings SPI_Settings(1000000, MSBFIRST, SPI_MODE0);


void setup() {
  // set the Slave Select Pins as outputs:
  pinMode(CS, OUTPUT);
  digitalWrite(CS, LOW);
  
  // initialize SPI:
  SPI.begin();
}

uint8_t val = 1235; //send anything

void loop() {
  SPI.beginTransaction(SPI_Settings);
  digitalWrite(CS, HIGH);
  SPI.transfer(val);
  digitalWrite(CS, LOW);
  SPI.endTransaction();
  delay(500);
}

