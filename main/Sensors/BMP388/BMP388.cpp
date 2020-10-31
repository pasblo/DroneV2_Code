#include "Arduino.h"
#include "BMP388.h"
#include "bmp3.h"
#include "bmp3.c"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Pinout.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/definitions.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Processing/Filter.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Processing/Filter.cpp"
#if defined(MAIN_PROCESSOR)
	#include <SPI.h> // TODO, to be sustituded by DSPI when finished

#elif defined(EXTERNAL_PROCESSOR)
	#include <Wire.h>

#endif

DroneBMP388 DBMP388;

bool DroneBMP388::begin(){

	#if defined(MAIN_PROCESSOR)
		SPI.begin(); // TODO, initialize in the begin class
		pinMode(CS_BMP_PIN, OUTPUT); //TODO, initialize pinout class

		sensor.dev_id = CS_BMP_PIN;
		sensor.intf = BMP3_SPI_INTF;
		sensor.read = &(DroneBMP388::spi_read);
		sensor.write = &(DroneBMP388::spi_write);

	#elif defined(EXTERNAL_PROCESSOR)
		Wire.begin(); // TODO, initialize in the begin class

		sensor.dev_id = BMP388_ADDRESS;
		sensor.intf = BMP3_I2C_INTF;
		sensor.read = &(DroneBMP388::i2c_read);
		sensor.write = &(DroneBMP388::i2c_write);

	#endif

	sensor.delay_ms = (DroneBMP388::delay_msec);

	//Initializing the sensor, and checking if is detected and performing correctly
	bmpStatus = bmp3_init(&sensor);
	if(bmpStatus != BMP3_OK) return false;

	//Make the sensor only respond when we told it so
	sensor.settings.op_mode = BMP3_FORCED_MODE;

	//Initialize the bmp filter using the kalaman filter
	bmpFilter = new Filter(KALAMAN_FILTER);

	//Settings sampling rates and filtering to standard mode
	DBMP388.setSensorMode(HIGH_PERFORMANCE_MODE);

	return true;
}

bool DroneBMP388::readSensor(bool useFiltering){

	//Settings to be sent to the sensor
	uint16_t settings_sel = 0;

	//Selecting the sensors components that are going to be read
	uint8_t sensor_comp = 0;

	//Activating the pressure sensor to read it
	sensor.settings.press_en = BMP3_ENABLE;
	settings_sel |= BMP3_PRESS_EN_SEL;
	settings_sel |= BMP3_PRESS_OS_SEL;

	//Activating the pressure sensor oversampling if it is enabled
	if(sensor.settings.odr_filter.press_os != BMP3_NO_OVERSAMPLING) sensor_comp |= BMP3_PRESS;

	//Activating the temperature sensor to read it
	sensor.settings.temp_en = BMP3_ENABLE;
	settings_sel |= BMP3_TEMP_EN_SEL;
	settings_sel |= BMP3_TEMP_OS_SEL;

	//Activating the temperature sensor oversampling if it is enabled
	if(sensor.settings.odr_filter.temp_os != BMP3_NO_OVERSAMPLING) sensor_comp |= BMP3_TEMP;

	//Activating the IIR filter built on the sensor if the filter is enabled
	if(sensor.settings.odr_filter.iir_filter != BMP3_IIR_FILTER_DISABLE) settings_sel |= BMP3_IIR_FILTER_SEL;

	//Activating the specified output data rate
	settings_sel |= BMP3_ODR_SEL;

	//Sending the settings to the sensor
	bmpStatus = bmp3_set_sensor_settings(settings_sel, &sensor);
	if(bmpStatus != BMP3_OK) return false;

	//Setting and sending the power mode of the bmp sensor
	sensor.settings.op_mode = BMP3_FORCED_MODE;
	bmpStatus = bmp3_set_op_mode(&sensor);
	if(bmpStatus != BMP3_OK) return false;

	//Tempora variable to store the data readed from the sensor
	struct bmp3_data data;

	//Reading the sensor with the specified components selected
	bmpStatus = bmp3_get_sensor_data(sensor_comp, &data, &sensor);
	if(bmpStatus != BMP3_OK) return false;


	//Storing the temperature and pressure and calculating the altitude
	temperature = data.temperature;
	pressure = data.pressure;
	rawAltitude = DBMP388.calculateAltitude(pressure);

	//Apply filters to the altitude
	if(useFiltering) filteredAltitude = bmpFilter->updateEstimate(rawAltitude);

	return true;
}

bool DroneBMP388::setSensorMode(uint8_t mode){ //TODO, calibrate

	struct filterProperties properties;
	switch(mode){
		case STANDARD_MODE: //Ideal for most situations
			sensor.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
			sensor.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
			sensor.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_7;
			sensor.settings.odr_filter.odr = BMP3_ODR_100_HZ;
			properties.measurementError = 1;
			properties.estimateError = 1;
			properties.processNoise = 1;
			break;

		case HIGH_PERFORMANCE_MODE: //Ideal for fast moving objects
			sensor.settings.odr_filter.press_os = BMP3_OVERSAMPLING_16X;
			sensor.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
			sensor.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_7;
			sensor.settings.odr_filter.odr = BMP3_ODR_200_HZ;
			properties.measurementError = 1;
			properties.estimateError = 1;
			properties.processNoise = 1;
			break;

		case LOW_NOISE_MODE: //Ideal for precise altitude control
			sensor.settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
			sensor.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
			sensor.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_7;
			sensor.settings.odr_filter.odr = BMP3_ODR_50_HZ;
			properties.measurementError = 1;
			properties.estimateError = 1;
			properties.processNoise = 1;
			break;

		default:
			return false;
	}

	//Introducing the filter properties onto the filter
	bmpFilter->setFilterProperties(properties);
	return true;

}

bool DroneBMP388::calibrate(){
	int i = 0;
	double totalPressure = 0;
	double totalTemperature = 0;
	while(i < AUTO_CALIBRATION_CYCLES){ //TODO, change wffqile loops into the scheduler
		if(!readSensor(false)) return false;
		i++;
	}
	i = 0;
	while(i < CALIBRATION_CYCLES){
		if(!readSensor(false)) return false;
		totalPressure += pressure;
		totalTemperature += temperature;
		i++;
	}
	i = 0;
	lastCalibration.temperature = totalTemperature / (double) CALIBRATION_CYCLES;
	lastCalibration.pressure = totalPressure / (double) CALIBRATION_CYCLES;
	lastCalibration.altitude = DBMP388.calculateAltitude(lastCalibration.pressure);

	while(i < AUTO_CALIBRATION_CYCLES){ //TODO, change wffqile loops into the scheduler
		if(!readSensor(false)) return false;
		i++;
	}
	return true;
}

bool DroneBMP388::calibrateSeaLevel(double realAltitude){ //TODO, change to accept procedural executing, with variable that indicates the state of the barometer

	//First we calibrate the barometer
	if(!calibrate()) return false;

	//Second we estimate the sea pressure knowing the altitude we are at and the pressure calibrated
	estimatedSeaPressure = pow(1.0 + (0.0000225576956603 * realAltitude), 5.2534652789470021) * lastCalibration.pressure;
	return true;
}

#if defined(MAIN_PROCESSOR)
	int8_t DroneBMP388::spi_read(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
		//TODO, sustitude with DSPI functions
	    SPI.beginTransaction(SPISettings(DEFAULT_SCK_FREQUENCY, MSBFIRST, SPI_MODE0));

		digitalWrite(cspin, LOW);

		SPI.transfer(reg_addr | 0x80);
		while (len--) {
			*reg_data = SPI.transfer(0x00);
			reg_data++;
		}

		digitalWrite(cspin, HIGH);

		SPI.endTransaction();

		return 0;
	}

	int8_t DroneBMP388::spi_write(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
		//TODO, sustitude with DSPI functions
		SPI.beginTransaction(SPISettings(DEFAULT_SCK_FREQUENCY, MSBFIRST, SPI_MODE0));

		digitalWrite(cspin, LOW);

		SPI.transfer(reg_addr);
		while (len--) {
			SPI.transfer(*reg_data);
			reg_data++;
		}

		digitalWrite(cspin, HIGH);

		SPI.endTransaction();

		return 0;
	}
#elif defined(EXTERNAL_PROCESSOR)
	int8_t DroneBMP388::i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
		Wire.beginTransmission((uint8_t)dev_id);
		Wire.write((uint8_t)reg_addr);
		Wire.endTransmission();
		if (len != Wire.requestFrom((uint8_t)dev_id, (byte)len)) return 1;
		while (len--) {
			*reg_data = (uint8_t)Wire.read();
			reg_data++;
		}
		return 0;
	}

	int8_t DroneBMP388::i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
		Wire.beginTransmission((uint8_t)dev_id);
		Wire.write((uint8_t)reg_addr);
		while (len--) {
			Wire.write(*reg_data);
			reg_data++;
		}
		Wire.endTransmission();
		return 0;
	}
#endif

void DroneBMP388::delay_msec(uint32_t ms){
	delay(ms);
}