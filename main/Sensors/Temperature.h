#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Time/DroneTime.h"

//11 temperature sensors

// Teensy 3.0,3.1,3.2
#if defined(__MK20DX128__) || defined(__MK20DX256__)  
	#define DEFAULT_VAT25 		(0.719)
	#define DEFAULT_SLOPE 		(0.00172)
	#define TEMPERATURE_CHANNEL (26)
	#define TEMPERATURE_PIN 	(38)

// Teensy 3.5,3.6
#else
	#define DEFAULT_VAT25 		(0.716)
	#define DEFAULT_SLOPE 		(0.00162)
	#define TEMPERATURE_CHANNEL (26)
	#define TEMPERATURE_PIN 	(70)
#endif

//Teensy 4.0 works in a different principle

class DroneTemperature{

	public:

		bool updateTemperatureSensors();

		float getProcessorTemperature() {return processorTemperature; }

		float getPCBTemperature() {return pcbTemperature; }

		timeUs_t getLastExecutionTime() {return lastExecutionTime; }

		#if defined(EXTERNAL_PROCESSOR)

			float getExternalTemperature1() {return externalTemperature1; }
			float getExternalTemperature2() {return externalTemperature2; }

		#endif

	private:

		timeUs_t lastExecutionTime;

		//Temperature values
		float processorTemperature, pcbTemperature;
		#if defined(EXTERNAL_PROCESSOR)
			float externalTemperature1, externalTemperature2;
		#endif

		//Calibration values
		float vAt25 = DEFAULT_VAT25;
		float slope = DEFAULT_SLOPE;
};
extern DroneTemperature DTemperature;
#endif