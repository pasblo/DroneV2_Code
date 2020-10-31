#ifndef SERIAL_HELPER_H
#define SERIAL_HELPER_H

#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Time/DroneTime.h"

class DroneSerialHelper{
	public:

		void printDouble(double val, uint8_t precision);

		void printUint64(uint64_t val);

		timeUs_t getLastExecutionTime(){return lastExecutionTime; }

	private:

		bool readingTime = false;

		timeUs_t lastExecutionTime;
};
extern DroneSerialHelper DSerialHelper;
#endif