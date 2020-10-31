#ifndef BMP388_H
#define BMP388_H

#include "Arduino.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Pinout.h" // TODO, sustitude with correct direction when using main
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/definitions.h" // TODO, sustitude with correct direction when using main
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Processing/Filter.h" // TODO, sustitude with correct direction when using main

#if defined(MAIN_PROCESSOR)
	#include <SPI.h> // TODO, to be sustituded by DSPI when finished

#elif defined(EXTERNAL_PROCESSOR)
	#include <Wire.h>

#else
	#error "This library is only intended to be used on the Main and Extension processor"

#endif
#include "bmp3.h"

//TODO, try and remove some libraries

//TODO change to enum
#define STANDARD_MODE UINT8_C(0x01)
#define HIGH_PERFORMANCE_MODE UINT8_C(0x02)
#define LOW_NOISE_MODE UINT8_C(0x03)

struct bmp388Data_t{
	double pressure;
	double altitude;
	float temperature;
};

class DroneBMP388{
	public:

		bool begin();
		void reset();

		bool calibrate();
		bool calibrateSeaLevel(double realAltitude);

		bool readSensor(bool useFiltering = true);

		double getPressure() {return pressure; }
		double getRawAltitude() {return rawAltitude; }
		double getFilteredAltitude() {return filteredAltitude; }
		float getTemperature() {return temperature; }
		bmp388Data_t getLastCalibration() {return lastCalibration; }
		double getSeaPressure() {return estimatedSeaPressure; }

		bool setSensorMode(uint8_t mode);

		int8_t getBmpStatus() {return bmpStatus; }

		#if defined(MAIN_PROCESSOR)
			static int8_t spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
			static int8_t spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
		#elif defined(EXTERNAL_PROCESSOR)
			static int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
			static int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
		#endif

		static void delay_msec(uint32_t ms);

	private:

		double calculateAltitude(double pressureUsed) {return 44330.7691999733615184348 * (pow(estimatedSeaPressure / pressureUsed, 0.1903505490000000016) - 1.0); }

		struct bmp3_dev sensor;
		Filter *bmpFilter;
		struct bmp388Data_t lastCalibration;

		double pressure; // Stored in Pascals
		double rawAltitude; // Stored in Meters
		double filteredAltitude; // Stored in Meters
		float temperature; // Stored in Celsius

		int8_t bmpStatus = BMP3_OK;

		double estimatedSeaPressure = SEA_PRESSURE; // Stored in Pascals
};
extern DroneBMP388 DBMP388;
#endif