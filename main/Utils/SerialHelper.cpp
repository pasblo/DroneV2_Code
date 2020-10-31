#include "SerialHelper.h"

DroneSerialHelper DSerialHelper;

void DroneSerialHelper::printUint64(uint64_t val){

	//Checking if this is the main call and not a recursive counterpart of it
	if(!readingTime) {
		lastExecutionTime = micros();
		readingTime = true;
	}

	//Calling itself up to printing all digits of the 64bit number
	if (val >= 10) printUint64(val / 10);

	//Printing the digit that this call needs
	Serial.println((uint8_t) val % 10);

	//Saving the time that this function lasted
	lastExecutionTime = micros() - lastExecutionTime;
	readingTime = false;
}

//The precision must be the number of decimals that are going to be printed
void DroneSerialHelper::printDouble(double val, uint8_t precision){

	//Saving when the function call started
	lastExecutionTime = micros();

	//Converting the double value to a string 
	Serial.print(String(val, precision));

	//Saving the time that this function lasted
	lastExecutionTime = micros() - lastExecutionTime;
}