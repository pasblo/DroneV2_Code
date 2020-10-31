/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP388 Breakout
  ----> http://www.adafruit.com/products/3966

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <SimpleKalmanFilter.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 22

#define SEALEVELPRESSURE_HPA (1027.0)

Adafruit_BMP3XX bmp; // I2C
//Adafruit_BMP3XX bmp(BMP_CS); // hardware SPI
//Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

SimpleKalmanFilter pressureKalmanFilter(0.2, 1, 1);

double lastAltitude;

void setup() {
  Serial.begin(9600);
  //while (!Serial);
  //Serial.println("BMP388 test");

  if (!bmp.begin()) {
    //Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);
}

void loop() {
  if (! bmp.performReading()) {
    //Serial.println("Failed to perform reading :(");
    return;
  }
  /*Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");*/
  lastAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print(lastAltitude*100.0);
  Serial.print(",");
  Serial.println(pressureKalmanFilter.updateEstimate(lastAltitude)*100.0);

    /*Serial.println();
    delay(2000);*/
}

