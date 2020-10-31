#ifndef ERROR_CODES_H
#define ERROR_CODES_H

#include "Config.h"
#include <vector>
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Time/DroneTime.h"

using namespace std;

//Error sources
typedef enum {
	SPI_SOURCE = 0x00,
	ICM20948_SOURCE = 0x01,
} ErrorSource_e;

typedef ErrorSource_e ErrorSource_t;

//SPI error codes
/*typedef enum {
	NO_ERROR = 0x00,
} DSPI_ErrorCode_e;*/

//ICM20948 error codes, needs 4bits
typedef enum {
	ICM20948_ID_ERROR = 0x0,
	AK09916_ID_ERROR = 0x1,
	ICM20948_RESET_TIMEOUT_ERROR = 0x2,
	I2C_MASTER_RESET_TIMEOUT_ERROR = 0x3,
	AK09916_RESET_TIMEOUT_ERROR = 0x4,
	ICM20948_PWR_ERROR = 0x5,
	AK09916_TRANSMISSION_ERROR = 0x6,

	//Maybe a weird position of the sensor cause these
	GYRO_SELF_TEST_ERROR = 0x7,
	ACCEL_SELF_TEST_ERROR = 0x8,
	MAG_SELF_TEST_ERROR = 0x9,

	CHIP_UNINITIALIZED_ERROR = 0xA,
} ICM20948_ErrorCode_e;

//One type of error. TODO, add errorCounter for optimized function
typedef struct{
	ErrorSource_t source;
	uint8_t error;
	#if defined(LOG_ERROR_CODES_TIMESTAMPS)
		timeUs_t errorTimestamp;
	#endif
} LoggedError_t;

class DroneErrorCodes{
	public:

		//Sets a new error code
		void setErrorCode(ErrorSource_t errorSource, uint8_t errorCode);

		//Returns the last error codes since the last getLastErrorCodes
		vector<LoggedError_t> getLastErrorCodes(){
			erraseLog = true;
			return loggedErrors;
		}

		void printLastErrorCodes(); //TODO

		//Execution times
		timeUs_t getLastExecutionTime() {return lastExecutionTime; }
		timeUs_t getTotalExecutionTime() {
			timeUs_t temporalTime = totalExecutionTime;
			totalExecutionTime = 0;
			return temporalTime;
		}

	private:

		//Execution times
		timeUs_t lastExecutionTime;
		timeUs_t totalExecutionTime;

		//Error codes
		vector<LoggedError_t> loggedErrors;
		//vector<LoggedError_t>::iterator loggedErrorsBegin;

		//Control variables
		bool erraseLog;
		
};
extern DroneErrorCodes DErrorCodes;
#endif