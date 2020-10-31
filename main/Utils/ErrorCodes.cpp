#include "ErrorCodes.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Definitions.h"

DroneErrorCodes DErrorCodes;

void DroneErrorCodes::setErrorCode(ErrorSource_e errorSource, uint8_t errorCode){

	//Saving when the function call started
	lastExecutionTime = micros();

	#if defined(LOG_ERROR_CODES)

		//Errasing the error log if it was requested
		if(erraseLog) loggedErrors.clear();
		erraseLog = false;

		//Creating the new error
		LoggedError_t newError;
		newError.source = errorSource;
		newError.error = errorCode;
		#if defined(LOG_ERROR_CODES_TIMESTAMPS)
			newError.errorTimestamp = lastExecutionTime;
		#endif

		//Adding the new error into the error vector
		loggedErrors.push_back(newError);

	#endif

	//Calculating last and total execution time
	lastExecutionTime = micros() - lastExecutionTime;
	totalExecutionTime += lastExecutionTime;
}

void DroneErrorCodes::printLastErrorCodes(){

	//Errasing the error log if it was requested
	if(erraseLog) loggedErrors.clear();
	erraseLog = false;

	if(loggedErrors.size() == 0){
		DEBUG_PRINTLN("No errors");
		return;
	}
	for(unsigned int i = 0; i < loggedErrors.size(); i++){
		DEBUG_PRINT("Error nº ");
		DEBUG_PRINT(i);
		DEBUG_PRINT(" {source: ");
		DEBUG_PRINTF(loggedErrors.at(i).source, HEX);
		DEBUG_PRINT(", error: ");
		DEBUG_PRINTF(loggedErrors.at(i).error, HEX);
		#if defined(LOG_ERROR_CODES_TIMESTAMPS)
			DEBUG_PRINT(", timestamp: ");
			DEBUG_PRINT(loggedErrors.at(i).errorTimestamp);
		#endif
		DEBUG_PRINTLN("}");
	}
	erraseLog = true;
}