#include "ICM20948.h"
#include "SPI.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Pinout.h" // TODO, sustitude with correct direction when using main
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/ErrorCodes.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Definitions.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/SerialHelper.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Math.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Filters.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/ICM20948/ICM20948_AK09916Driver.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/ICM20948/ICM20948_DMPDriver.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/ICM20948/ICM20948_DMPMath.h"

DroneICM20948 DICM20948;

bool DroneICM20948::begin(){

	//Setting the register bank selected to 4 because there is no 4 register bank so in the first change it will actually change
	regBank = 4;

	//TODO, sustitude by DSPI
	pinMode(CS_IMU_PIN, OUTPUT);
	digitalWrite(CS_IMU_PIN, HIGH);
	spiSettings = SPISettings(1000000, MSBFIRST, SPI_MODE0);
	SPI.begin();

	//Setting the bank to 0
	setRegBank(0);

	//Reading the ID of the chip
	uint8_t chipId;
	readICM20948Register(ICM20948_B0_WHOAMI, 1, &chipId, 1);

	//If the chip is not the ICM20948, something has gone pretty bad, now it seems to be the chip
	if(chipId != ICM20948_ID) {
		DErrorCodes.setErrorCode(ICM20948_SOURCE, ICM20948_ID_ERROR);
		return false;
	}

	//Resetting the sensor if it was all alright
	if(!reset()) return false;

	//Setting the bank to 0
	setRegBank(0, true);

	//Waking up the chip
	writeICM20948RegisterBits(ICM20948_B0_PWR_MGMT_1, 1, 0, 1, 1, 6);

	//Resetting the I2C slave and putting the serial interface in SPI mode only
	//writeICM20948RegisterBits(ICM20948_B0_USER_CTRL, 1, 1, 1, 1, 4);

	//Initializing I2C master and starting the magnetometer
	if(!beginMagnetometer()) return false; //TODO, magnetometer does not like the DMP for some reason

	//Initialize DMP
	beginDMP();

	/*
	mounting_matrix_secondary_compass[0] = 1 ;
	mounting_matrix_secondary_compass[4] = -1;
	mounting_matrix_secondary_compass[8] = -1;
	*/

	/*
	secondary_state.compass_sens[0] = 128;
	secondary_state.compass_sens[1] = 128;
	secondary_state.compass_sens[2] = 128;
	*/

	//secondary_state.mode_reg_addr = REG_AK09916_CNTL2;

	//Apply mounting matrix to the DMP
	applyMountingMatrix();

	//Run self-tests and calculate offsets maybe

	chipInitialized = true;

	//Creating low pass filters
	gyroLPF = DFilters.createLowPassFilter();
	accelLPF = DFilters.createLowPassFilter();
	magLPF = DFilters.createLowPassFilter();

	//Setting the sensors to the default state
	setGyroMode(ICM20948_GYRO_DMP_TESTING_MODE);
	setAccelMode(ICM20948_ACCEL_DMP_TESTING_MODE);
	setMagMode(ICM20948_MAG_STANDARD_MODE);
	setTempMode(ICM20948_TEMP_STANDARD_MODE);

	//Standard delay
	delay(20);

	//Setting the bank to 0
	setRegBank(0);

	//Reading the power management register 1
	uint8_t powerManagement1;
	readICM20948Register(ICM20948_B0_PWR_MGMT_1, 1, &powerManagement1, 1);
	
	//Checking if the power management register 1 was correct
	if(powerManagement1 > 7){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, ICM20948_PWR_ERROR);
		return false;
	}

	//Setting the bank to 1
	setRegBank(1);

	//Reading the standard self test values
	readICM20948Register(ICM20948_B1_SELF_TEST_DATA, 1, selfTestBase, 6);

	return true;
}

bool DroneICM20948::reset(){

	//Setting the bank register to 0
	setRegBank(0);

	//Setting the reset bit to one
	writeICM20948RegisterBits(ICM20948_B0_PWR_MGMT_1, 1, 1, 1, 1, 7);

	//Normalized wait before reading if the reset bit has been cleared
	delay(4);

	//Saving the initial time at wich the reset loop wait is going to be held
	internalTime = millis();

	//Waiting for the autoclear of the reset bit
	while (readICM20948RegisterBits(ICM20948_B0_PWR_MGMT_1, 1, 1, 1, 7)) {

		//Checking if we have not passed the timeout
		if(millis() - internalTime > RESET_TIMEOUT){
			DErrorCodes.setErrorCode(ICM20948_SOURCE, ICM20948_RESET_TIMEOUT_ERROR);
			return false;
		}

		//Waiting to not ask very fast
		delay(1);
	};

	//Normalized wait after the reset to make all settle
	delay(1);

	return true;
}

bool DroneICM20948::dynamicReset(){ //TODO, add the initialization of the DMP?

	//Setting the sensors mode to sleep to prepare them for a reset
	setGyroMode(ICM20948_GYRO_SLEEP_MODE);
	setAccelMode(ICM20948_ACCEL_SLEEP_MODE);
	setMagMode(ICM20948_MAG_SLEEP_MODE);
	setTempMode(ICM20948_TEMP_SLEEP_MODE);

	//Saving the init execution time
	lastExecutionTime = micros();

	//As we are going to reset if the chip is not able to complete the process the processor must know that the chip is not initialized
	chipInitialized = false;

	//Resetting the sensor
	if(!reset()){
		lastExecutionTime = micros() - lastExecutionTime;
		return false;
	}

	//As the reset process was succesfull the chip is now initialized
	chipInitialized = true;

	//Setting the bank to 0
	setRegBank(0, true);

	//Waking up the chip
	writeICM20948RegisterBits(ICM20948_B0_PWR_MGMT_1, 1, 0, 1, 1, 6);

	//Starting the magnetometer
	if(!beginMagnetometer()){
		lastExecutionTime = micros() - lastExecutionTime;
		return false;
	}

	//Setting the sensors to the default state
	setGyroMode(ICM20948_GYRO_STANDARD_MODE);
	setAccelMode(ICM20948_ACCEL_STANDARD_MODE);
	setMagMode(ICM20948_MAG_STANDARD_MODE);
	setTempMode(ICM20948_TEMP_STANDARD_MODE);

	//Standard delay
	delay(1);

	//Setting the bank to 0
	setRegBank(0);

	//Reading the power management register 1
	uint8_t powerManagement1;
	readICM20948Register(ICM20948_B0_PWR_MGMT_1, 1, &powerManagement1, 1);
	
	//Checking if the power management register 1 was correct
	if(powerManagement1 > 7){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, ICM20948_PWR_ERROR);
		lastExecutionTime = micros() - lastExecutionTime;
		return false;
	}

	//Calculating and storing the execution time of the function
	lastExecutionTime = micros() - lastExecutionTime;

	return true;
}

bool DroneICM20948::updateSensors(double localGroundGravity, dataToObtain_e dataToObtain, bool scaleData){

	//Checking if the sensor was initizalied
	if(!chipInitialized){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, CHIP_UNINITIALIZED_ERROR);
		return false;
	}

	//Saving the init execution time
	lastExecutionTime = micros();

	//Raw data only
	if(dataToObtain == OBTAIN_RAW){
		//Setting the bank to 0
		setRegBank(0);

		//Declaring the number of bytes to read and the buffer to store them
		uint8_t numbytes = 6 + 6 + 2 + 9; //6 bytes gyro, 6 bytes accel, 2 bytes temp, 6 bytes mag, 3 bytes mag extra
		uint8_t buffer[numbytes];

		//Reading all the sensor data bytes
		readICM20948Register(ICM20948_B0_ACCEL_DATA_OUT, 1, buffer, numbytes);

		//Joining the lower and upper parts of all the sensor data registers
		droneGyroscope.rawData.X = buffer[6] << 8 | buffer[7];
		droneGyroscope.rawData.Y = buffer[8] << 8 | buffer[9];
		droneGyroscope.rawData.Z = buffer[10] << 8 | buffer[11];

		droneAccelerometer.rawData.X = buffer[0] << 8 | buffer[1];
		droneAccelerometer.rawData.Y = buffer[2] << 8 | buffer[3];
		droneAccelerometer.rawData.Z = buffer[4] << 8 | buffer[5];

		droneTemperature.rawData = buffer[12] << 8 | buffer[13];

		droneMagnetometer.rawData.X = ((buffer[16] << 8) | (buffer[15] & 0xFF));
		droneMagnetometer.rawData.Y = ((buffer[18] << 8) | (buffer[17] & 0xFF));
		droneMagnetometer.rawData.Z = ((buffer[20] << 8) | (buffer[19] & 0xFF));

		//Scaling the data read to match configurations
		if(scaleData) scaleSensorsData(localGroundGravity);

		//Filtering the data, TODO
		/*droneGyroscope.filteredData.X = DFilters.applyLowPassFilter(&gyroLPF, droneGyroscope.scaledData.X);
		droneGyroscope.filteredData.Y = DFilters.applyLowPassFilter(&gyroLPF, droneGyroscope.scaledData.Y);
		droneGyroscope.filteredData.Z = DFilters.applyLowPassFilter(&gyroLPF, droneGyroscope.scaledData.Z);*/

	//DMP data only
	}else{

		/*do{

			//Mirror FIFO contents and stop processing FIFO if error detected
			if()
		}*/
	}

	//Calculating and storing the execution time of the function
	lastExecutionTime = micros() - lastExecutionTime;

	return true;
}

void DroneICM20948::scaleSensorsData(double gravity){

	//Correcting the scale of the offsets, cause the offsets are measured in a specific scale
	droneGyroscope.rawData.X -= gyroOffsetX * gyroCorrectionScale;
	droneGyroscope.rawData.Y -= gyroOffsetY * gyroCorrectionScale;
	droneGyroscope.rawData.Z -= gyroOffsetZ * gyroCorrectionScale;

	droneAccelerometer.rawData.X -= accelOffsetX * accelCorrectionScale;
	droneAccelerometer.rawData.Y -= accelOffsetY * accelCorrectionScale;
	droneAccelerometer.rawData.Z -= accelOffsetZ * accelCorrectionScale; //TODO, check if this is right

	//Scaling the raw values
	droneGyroscope.scaledData.X = droneGyroscope.rawData.X / gyroScale;
	droneGyroscope.scaledData.Y = droneGyroscope.rawData.Y / gyroScale;
	droneGyroscope.scaledData.Z = droneGyroscope.rawData.Z / gyroScale;

	droneAccelerometer.scaledData.X = droneAccelerometer.rawData.X / accelScale; //Probably not the best solution
	droneAccelerometer.scaledData.Y = droneAccelerometer.rawData.Y / accelScale;
	droneAccelerometer.scaledData.Z = droneAccelerometer.rawData.Z / accelScale;

	droneTemperature.scaledData = (droneTemperature.rawData / 333.87) + 21.0; // ((OUT - OFFSET) / SENSITIVITY) + 21

	droneMagnetometer.scaledData.X = droneMagnetometer.rawData.X * MAG_SCALE;
	droneMagnetometer.scaledData.Y = droneMagnetometer.rawData.Y * MAG_SCALE;
	droneMagnetometer.scaledData.Z = droneMagnetometer.rawData.Z * MAG_SCALE;

	//Units conversion
	droneGyroscope.scaledData.X = DMath.dpsToRads(droneGyroscope.scaledData.X);
	droneGyroscope.scaledData.Y = DMath.dpsToRads(droneGyroscope.scaledData.Y);
	droneGyroscope.scaledData.Z = DMath.dpsToRads(droneGyroscope.scaledData.Z);

	droneAccelerometer.scaledData.X *= gravity;
	droneAccelerometer.scaledData.Y *= gravity;
	droneAccelerometer.scaledData.Z *= gravity;

	droneMagnetometer.scaledData.X *= 0.01;
	droneMagnetometer.scaledData.Y *= 0.01;
	droneMagnetometer.scaledData.Z *= 0.01;
}

bool DroneICM20948::calibrateSensors(double gravity){

	icm20948_gyro_mode_t tempGyroMode = gyroMode;
	icm20948_accel_mode_t tempAccelMode = accelMode;
	icm20948_mag_mode_t tempMagMode = magMode;
	icm20948_temp_mode_t tempTempMode = tempMode;

	//Checking if the sensor was initizalied
	if(!chipInitialized){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, CHIP_UNINITIALIZED_ERROR);
		return false;
	}

	//Setting the sensors to the calivration state
	setGyroMode(ICM20948_GYRO_CALIB_MODE);
	setAccelMode(ICM20948_ACCEL_CALIB_MODE);
	setMagMode(ICM20948_MAG_CALIB_MODE);
	setTempMode(ICM20948_TEMP_CALIB_MODE);

	//Performing a self test to make sure all sensors are working norminal
	if(!gyroSelfTest()){ //TODO, check if the signals recived are in the good range
		setGyroMode(ICM20948_GYRO_SLEEP_MODE);
		return false;
	}
	if(!accelSelfTest()){ //TODO, check if the signals recived are in the good range
		setAccelMode(ICM20948_ACCEL_SLEEP_MODE);
		return false;
	}
	if(!magSelfTest()){ //TODO, check if the signals recived are in the good range
		setMagMode(ICM20948_MAG_SLEEP_MODE);
		return false;
	}

	//Calculating the offsets
	calculateOffsets(gravity);

	//Setting the sensors to the previous state
	setGyroMode(tempGyroMode);
	setAccelMode(tempAccelMode);
	setMagMode(tempMagMode);
	setTempMode(tempTempMode);

	/*DEBUG_PRINT("Gyro offset X: ");
	DEBUG_PRINT(gyroOffsetX);
	DEBUG_PRINT(", Y: ");
	DEBUG_PRINT(gyroOffsetY);
	DEBUG_PRINT(", Z: ");
	DEBUG_PRINTLN(gyroOffsetZ);

	DEBUG_PRINT("Accel offset X: ");
	DEBUG_PRINT(accelOffsetX);
	DEBUG_PRINT(", Y: ");
	DEBUG_PRINT(accelOffsetY);
	DEBUG_PRINT(", Z: ");
	DEBUG_PRINTLN(accelOffsetZ);*/

	return true;
}

bool DroneICM20948::gyroSelfTest(){

	//Declaring gyro self test variables
	int16_t outputX, outputSelfTestX;
	int16_t outputY, outputSelfTestY;
	int16_t outputZ, outputSelfTestZ;

	//Reading the averaged sensor data
	read3AxisAveragedSensor(ICM20948_B0_GYRO_DATA_OUT, &outputX, &outputY, &outputZ);

	//Setting the bank to 2
	setRegBank(2);

	//Enabling the self test
	writeICM20948RegisterBits(ICM20948_B2_GYRO_CONFIG_2, 1, 7, 1, 1, 3);

	//Standard delay
	delay(10);

	//Reading the averaged sensor data with self test enabled
	read3AxisAveragedSensor(ICM20948_B0_GYRO_DATA_OUT, &outputSelfTestX, &outputSelfTestY, &outputSelfTestZ);

	//Setting the bank to 2
	setRegBank(2);

	//Disabling the self test
	writeICM20948RegisterBits(ICM20948_B2_GYRO_CONFIG_2, 1, 0, 1, 1, 3);

	//Checking if the self test values are in the range specified
	if((abs(selfTestBase[0] - (outputSelfTestX - outputX)) > 5000) | (abs(selfTestBase[1] - (outputSelfTestY - outputY)) > 5000) | (abs(selfTestBase[2] - (outputSelfTestZ - outputZ)) > 5000)){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, GYRO_SELF_TEST_ERROR);
	}

	return true;
}

bool DroneICM20948::accelSelfTest(){

	//Declaring accel self test variables
	int16_t outputX, outputSelfTestX;
	int16_t outputY, outputSelfTestY;
	int16_t outputZ, outputSelfTestZ;

	//Reading the averaged sensor data
	read3AxisAveragedSensor(ICM20948_B0_ACCEL_DATA_OUT, &outputX, &outputY, &outputZ);

	//Setting the bank to 2
	setRegBank(2);

	//Enabling the self test
	writeICM20948RegisterBits(ICM20948_B2_GYRO_CONFIG_2, 1, 7, 1, 1, 3);

	delay(10); //ICM20948_B0_ACCEL_DATA_OUT

	read3AxisAveragedSensor(ICM20948_B0_ACCEL_DATA_OUT, &outputSelfTestX, &outputSelfTestY, &outputSelfTestZ);

	//Setting the bank to 2
	setRegBank(2);

	//Disabling the self test
	writeICM20948RegisterBits(ICM20948_B2_GYRO_CONFIG_2, 1, 0, 1, 1, 3);

	//Checking if the self test values are in the range specified
	if((abs(selfTestBase[3] - (outputSelfTestX - outputX)) > 5000) | (abs(selfTestBase[4] - (outputSelfTestY - outputY)) > 5000) | (abs(selfTestBase[5] - (outputSelfTestZ - outputZ)) > 5000)){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, ACCEL_SELF_TEST_ERROR);
	}

	return true;
}

void DroneICM20948::read3AxisAveragedSensor(uint16_t regAddr, int16_t *Xread, int16_t *Yread, int16_t *Zread){

	//Declaring variables
	uint8_t numbytes = 3 + 3;
	uint8_t buffer[numbytes];

	//Setting the bank to 0
	setRegBank(0);

	//Reading the buffer from the address specified
	readICM20948Register(regAddr, 1, buffer, numbytes);

	//Joining the lower and upper parts of all the sensor data registers
	*Xread = buffer[0] << 8 | buffer[1];
	*Yread = buffer[2] << 8 | buffer[3];
	*Zread = buffer[4] << 8 | buffer[5];
}

void DroneICM20948::calculateOffsets(double gravity){

	//Pre calibration measurements
	for(int i = 0; i < PRE_CALIB_MEASUREMENTS; i++){
		updateSensors(gravity, OBTAIN_RAW, false);
		delay(1);
	}

	//Declaring temporal variables to calculate the average
	int32_t tempGyroX = 0;
	int32_t tempGyroY = 0;
	int32_t tempGyroZ = 0;
	int32_t tempAccelX = 0;
	int32_t tempAccelY = 0;
	int32_t tempAccelZ = 0;
	//int32_t tempMagX, tempMagY, tempMagZ;

	for(int i = 0; i < CALIB_MEASUREMENTS; i++){

		//Updating the sensors
		updateSensors(gravity, OBTAIN_RAW, false);

		//Adding the measure data to a temporal variable
		tempGyroX += droneGyroscope.rawData.X;
		tempGyroY += droneGyroscope.rawData.Y;
		tempGyroZ += droneGyroscope.rawData.Z;
		tempAccelX += droneAccelerometer.rawData.X;
		tempAccelY += droneAccelerometer.rawData.Y;
		tempAccelZ += droneAccelerometer.rawData.Z;

		//Dampening delay
		delay(1);
	}

	//Obtaining the offsets
	gyroOffsetX = tempGyroX / CALIB_MEASUREMENTS;
	gyroOffsetY = tempGyroY / CALIB_MEASUREMENTS;
	gyroOffsetZ = tempGyroZ / CALIB_MEASUREMENTS;
	accelOffsetX = tempAccelX / CALIB_MEASUREMENTS;
	accelOffsetY = tempAccelY / CALIB_MEASUREMENTS;
	accelOffsetZ = tempAccelZ / CALIB_MEASUREMENTS;

	accelOffsetZ -= accelScale;
}

bool DroneICM20948::setGyroMode(icm20948_gyro_mode_e newMode){

	//Checking if the sensor was initizalied
	if(!chipInitialized){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, CHIP_UNINITIALIZED_ERROR);
		return false;
	}

	//Checking if the mode is already selected
	if(gyroMode == newMode) return false;
	gyroMode = newMode;

	//Choosing the characteristics dependig on the mode selected
	switch(newMode){
		case ICM20948_GYRO_RESET_MODE:
			return true;

		case ICM20948_GYRO_SLEEP_MODE:
			droneGyroscope.enabled = false;
			break;

		case ICM20948_GYRO_CALIB_MODE:
			droneGyroscope.enabled = true;
			droneGyroscope.rateDiv = 0;
			droneGyroscope.range = ICM20948_GYRO_RANGE_250_DPS;
			droneGyroscope.enabledDLPF = true;
			droneGyroscope.cutoffFreqDLPF = ICM20948_GYRO_CUTOFF_FREQ_5_7_HZ;
			droneGyroscope.averaging = ICM20948_GYRO_AVEG_128;
			break;

		case ICM20948_GYRO_STANDARD_MODE: //ALL VALUES ARE TEMPORAL FOR TESTING PURPOSES
			droneGyroscope.enabled = true;
			droneGyroscope.rateDiv = 0; // freq = 1100Hz/(1+rateDiv)
			droneGyroscope.range = ICM20948_GYRO_RANGE_250_DPS;
			droneGyroscope.enabledDLPF = true;
			droneGyroscope.cutoffFreqDLPF = ICM20948_GYRO_CUTOFF_FREQ_5_7_HZ;
			droneGyroscope.averaging = ICM20948_GYRO_AVEG_1;
			gyroLPF.k = 0;
			break;

		case ICM20948_GYRO_HIGH_ACCURACY_MODE:
			droneGyroscope.enabled = true;
			droneGyroscope.rateDiv = 0;
			droneGyroscope.range = ICM20948_GYRO_RANGE_250_DPS;
			droneGyroscope.enabledDLPF = true;
			droneGyroscope.cutoffFreqDLPF = ICM20948_GYRO_CUTOFF_FREQ_5_7_HZ;
			droneGyroscope.averaging = ICM20948_GYRO_AVEG_128;
			gyroLPF.k = 0.1;
			break;

		case ICM20948_GYRO_DMP_TESTING_MODE:
			droneGyroscope.enabled = true;
			droneGyroscope.rateDiv = 19;
			droneGyroscope.range = ICM20948_GYRO_RANGE_250_DPS;
			droneGyroscope.enabledDLPF = true;
			droneGyroscope.cutoffFreqDLPF = ICM20948_GYRO_CUTOFF_FREQ_5_7_HZ;
			droneGyroscope.averaging = ICM20948_GYRO_AVEG_1;
			gyroLPF.k = 0.1;
			break;
	}

	//Setting the scaling variables depending of the scale selected
	if (droneGyroscope.range == ICM20948_GYRO_RANGE_250_DPS) gyroScale = 131.0;
	if (droneGyroscope.range == ICM20948_GYRO_RANGE_500_DPS) gyroScale = 65.5;
	if (droneGyroscope.range == ICM20948_GYRO_RANGE_1000_DPS) gyroScale = 32.8;
	if (droneGyroscope.range == ICM20948_GYRO_RANGE_2000_DPS) gyroScale = 16.4;

	gyroCorrectionScale = gyroScale / 131.0;

	//Bank set to 0
	setRegBank(0);

	//Enabling or disabling the gyroscope
	writeICM20948RegisterBits(ICM20948_B0_PWR_MGMT_2, 1, (!droneGyroscope.enabled) & 0b111, 1, 3, 0);

	//Exiting the gyro confguration if the sensor was disabled
	if(!droneGyroscope.enabled) return true;

	//Bank set to 2
	setRegBank(2);

	//Setting the gyroscope rate divider, aka frequency
	writeICM20948Register(ICM20948_B2_GYRO_SMPLRT_DIV, 1, droneGyroscope.rateDiv, 1);

	//Setting the gyroscope range
	writeICM20948RegisterBits(ICM20948_B2_GYRO_CONFIG_1, 1, droneGyroscope.range, 1, 2, 1);

	//Setting the gyroscope DLPF
	writeICM20948RegisterBits(ICM20948_B2_GYRO_CONFIG_1, 1, droneGyroscope.enabledDLPF, 1, 1, 0);
	if(droneGyroscope.enabledDLPF) writeICM20948RegisterBits(ICM20948_B2_GYRO_CONFIG_1, 1, droneGyroscope.cutoffFreqDLPF, 1, 3, 3);

	//Setting the averaging functionality
	writeICM20948RegisterBits(ICM20948_B2_GYRO_CONFIG_2, 1, droneGyroscope.averaging, 1, 3, 0);

	return true;
}

bool DroneICM20948::setAccelMode(icm20948_accel_mode_e newMode){

	//Checking if the sensor was initizalied
	if(!chipInitialized){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, CHIP_UNINITIALIZED_ERROR);
		return false;
	}

	//Checking if the mode is already selected
	if(accelMode == newMode) return false;
	accelMode = newMode;

	//Choosing the characteristics dependig on the mode selected
	switch(newMode){
		case ICM20948_ACCEL_RESET_MODE:
			return true;

		case ICM20948_ACCEL_SLEEP_MODE:
			droneAccelerometer.enabled = false;
			break;

		case ICM20948_ACCEL_CALIB_MODE:
			droneGyroscope.enabled = true;
			droneAccelerometer.rateDiv = 10;
			droneAccelerometer.range = ICM20948_ACCEL_RANGE_2_G;
			droneAccelerometer.enabledDLPF = true;
			droneAccelerometer.cutoffFreqDLPF = ICM20948_ACCEL_CUTOFF_FREQ_5_7_HZ;
			droneAccelerometer.averaging = ICM20948_ACCEL_AVEG_32;
			break;

		case ICM20948_ACCEL_STANDARD_MODE: //ALL VALUES ARE TEMPORAL FOR TESTING PURPOSES
			droneAccelerometer.enabled = true;
			droneAccelerometer.rateDiv = 0; // freq = 1125Hz/(1+rateDiv)
			droneAccelerometer.range = ICM20948_ACCEL_RANGE_16_G;
			droneAccelerometer.enabledDLPF = true; 
			droneAccelerometer.cutoffFreqDLPF = ICM20948_ACCEL_CUTOFF_FREQ_5_7_HZ;
			droneAccelerometer.averaging = ICM20948_ACCEL_AVEG_1;
			accelLPF.k = 0;
			break;

		case ICM20948_ACCEL_DMP_TESTING_MODE:
			droneAccelerometer.enabled = true;
			droneAccelerometer.rateDiv = 19; // freq = 1125Hz/(1+rateDiv)
			droneAccelerometer.range = ICM20948_ACCEL_RANGE_2_G;
			droneAccelerometer.enabledDLPF = true; 
			droneAccelerometer.cutoffFreqDLPF = ICM20948_ACCEL_CUTOFF_FREQ_5_7_HZ;
			droneAccelerometer.averaging = ICM20948_ACCEL_AVEG_1;
			accelLPF.k = 0;
			break;
	}

	//Calculating the accel scale
	if (droneAccelerometer.range == ICM20948_ACCEL_RANGE_2_G) accelScale = 16384.0;
	if (droneAccelerometer.range == ICM20948_ACCEL_RANGE_4_G) accelScale = 8192.0;
	if (droneAccelerometer.range == ICM20948_ACCEL_RANGE_8_G) accelScale = 4096.0;
	if (droneAccelerometer.range == ICM20948_ACCEL_RANGE_16_G) accelScale = 2048.0;

	accelCorrectionScale = accelScale / 16384.0;

	//Bank set to 0
	setRegBank(0);

	//Enabling or disabling the accelerometer
	writeICM20948RegisterBits(ICM20948_B0_PWR_MGMT_2, 1, (!droneAccelerometer.enabled) & 0b111, 1, 3, 3);

	//Exiting the accel confguration if the sensor was disabled
	if(!droneAccelerometer.enabled) return true;

	//Bank set to 2
	setRegBank(2);

	//Setting the accelerometer rate divisor
	writeICM20948Register(ICM20948_B2_ACCEL_SMPLRT_DIV_1, 1, droneAccelerometer.rateDiv, 2, MSBFIRST);

	//Setting the accelerometer range
	writeICM20948RegisterBits(ICM20948_B2_ACCEL_CONFIG_1, 1, droneAccelerometer.range, 1, 2, 1);

	//Setting the DLPF
	writeICM20948RegisterBits(ICM20948_B2_ACCEL_CONFIG_1, 1, droneAccelerometer.enabledDLPF, 1, 1, 0);
	if(droneAccelerometer.enabledDLPF) writeICM20948RegisterBits(ICM20948_B2_ACCEL_CONFIG_1, 1, droneAccelerometer.cutoffFreqDLPF, 1, 3, 3);

	//Setting the averaging functionality
	writeICM20948RegisterBits(ICM20948_B2_ACCEL_CONFIG_2, 1, droneAccelerometer.averaging, 1, 2, 0);

	return true;
}

bool DroneICM20948::setTempMode(icm20948_temp_mode_e newMode){

	//Checking if the sensor was initizalied
	if(!chipInitialized){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, CHIP_UNINITIALIZED_ERROR);
		return false;
	}

	//Checking if the mode is already selected
	if(tempMode == newMode) return false;
	tempMode = newMode;

	//Choosing the characteristics dependig on the mode selected
	switch(newMode){
		case ICM20948_TEMP_RESET_MODE:
			return true;

		case ICM20948_TEMP_SLEEP_MODE:
			droneTemperature.enabled = false;
			break;

		case ICM20948_TEMP_CALIB_MODE:
			droneGyroscope.enabled = true;
			droneTemperature.cutoffFreqDLPF = ICM20948_TEMP_CUTOFF_FREQ_8_8_HZ;
			break;

		case ICM20948_TEMP_STANDARD_MODE:
			droneTemperature.enabled = true;
			droneTemperature.cutoffFreqDLPF = ICM20948_TEMP_CUTOFF_FREQ_8_8_HZ;
			break;
	}

	//Bank se to 0
	setRegBank(0);

	//Enabling or disabling the temperature
	writeICM20948RegisterBits(ICM20948_B0_PWR_MGMT_1, 1, !droneTemperature.enabled, 1, 1, 3);

	//Setting the lpf frequency
	writeICM20948RegisterBits(ICM20948_B2_TEMP_CONFIG, 1, droneTemperature.cutoffFreqDLPF, 1, 3, 0);

	return true;
}

uint8_t DroneICM20948::transfer(uint8_t send) {
	uint8_t data = send;
	SPI.transfer(&data, 1);
	return data;
}

void DroneICM20948::adaptRegAddress(uint16_t address, RegType_e regType){

	//As we overwrite on this function the address register we make sure is empty before using it
	_addReg[0] = 0;
	_addReg[1] = 0;

	//Parsing the 16bit address to two 8bit parts of the address
	_addReg[0] = (uint8_t)(address & 0xFF);
	_addReg[1] = (uint8_t)(address >> 8);

	//Adding to the address something?
	if (regType == ADDRBIT8_HIGH_TOREAD) _addReg[0] |= 0x80;
	if (regType == ADDRBIT8_HIGH_TOWRITE) _addReg[0] &= ~0x80;
}

uint32_t DroneICM20948::convertTo32(uint8_t *data, uint8_t width, uint8_t byteorder){
	uint32_t value = 0;

	for (int i = 0; i < width; i++) {
		value <<= 8;
		if (byteorder == LSBFIRST) {
			value |= data[width - i - 1];
		} else {
			value |= data[i];
		}
	}

  return value;
}

void DroneICM20948::writeICM20948Register(uint16_t regAddr, uint8_t regAddrLen, const uint8_t *writeBuff, uint8_t writeBuffLen, bool adaptAddress){

	//Adapting the register to write
	adaptRegAddress(regAddr, ADDRBIT8_HIGH_TOWRITE);

	//Starting the SPI hardware stuff
	SPI.beginTransaction(spiSettings);

	//Enabling the comunication
	digitalWrite(CS_IMU_PIN, LOW);

	//Write the register addres that is going to be written
	for (size_t i = 0; i < regAddrLen; i++) {
		transfer(_addReg[i]);
	}

	//Write the data to be inserted to the register selected
	for (size_t i = 0; i < writeBuffLen; i++) {
		transfer(writeBuff[i]);
	}

	//Disabling the comunication
	digitalWrite(CS_IMU_PIN, HIGH);

	//Stopping the SPI hardware stuff
	SPI.endTransaction();
}

void DroneICM20948::writeICM20948Register(uint16_t regAddr, uint8_t regAddrLen, uint32_t writeBuff, uint8_t writeBuffLen, uint8_t byteorder) {
	
	//As we overwrite on this function the main buffer we make sure is empty before using it
	_buffer[0] = 0;
	_buffer[1] = 0;
	_buffer[2] = 0;
	_buffer[3] = 0;

	//Not support for more than 4 bytes or 0 bytes OVIOUSLY
	if (writeBuffLen > 4 || writeBuffLen == 0) return;

	for (int i = 0; i < writeBuffLen; i++) {

		if (byteorder == LSBFIRST) {
			_buffer[i] = writeBuff & 0xFF;
		} else {
			_buffer[writeBuffLen - i - 1] = writeBuff & 0xFF;
		}
		writeBuff >>= 8;
	}
	writeICM20948Register(regAddr, regAddrLen, _buffer, writeBuffLen);
}

void DroneICM20948::writeICM20948RegisterBits(uint16_t regAddr, uint8_t regAddrLen, uint32_t writeBuff, uint8_t writeBuffLen, uint8_t bits, uint8_t shift, uint8_t byteorder){
	
	//As we overwrite on this function the main buffer we make sure is empty before using it
	_buffer[0] = 0;
	_buffer[1] = 0;
	_buffer[2] = 0;
	_buffer[3] = 0;

	readICM20948Register(regAddr, regAddrLen, _buffer, 1);

	uint32_t val = convertTo32(_buffer, 1, byteorder);

	// mask off the data before writing
	uint32_t mask = (1 << (bits)) - 1;
	writeBuff &= mask;

	mask <<= shift;
	val &= ~mask;          // remove the current data at that spot
	val |= writeBuff << shift; // and add in the new data

	writeICM20948Register(regAddr, regAddrLen, val, writeBuffLen, byteorder);
}

void DroneICM20948::readICM20948Register(uint16_t regAddr, uint8_t regAddrLen, uint8_t *readBuff, uint8_t readBuffLen, uint8_t sendValue){

	//Adapting the register to read
	adaptRegAddress(regAddr, ADDRBIT8_HIGH_TOREAD);

	//Starting the SPI hardware stuff
	SPI.beginTransaction(spiSettings);

	//Enabling the comunication
	digitalWrite(CS_IMU_PIN, LOW);

	//Write the register addres that is going to be read
	for (size_t i = 0; i < regAddrLen; i++) {
		transfer(_addReg[i]);
	}

	//Do the reading
	for (size_t i = 0; i < readBuffLen; i++) {
		readBuff[i] = transfer(sendValue);
	}

	//Disabling the comunication
	digitalWrite(CS_IMU_PIN, HIGH);

	//Stopping the SPI hardware stuff
	SPI.endTransaction();
}

uint32_t DroneICM20948::readICM20948RegisterBits(uint16_t regAddr, uint8_t regAddrLen, uint8_t readBuffLen, uint8_t bits, uint8_t shift, uint8_t byteorder, uint8_t sendValue){
	
	//As we overwrite on this function the main buffer we make sure is empty before using it
	_buffer[0] = 0;
	_buffer[1] = 0;
	_buffer[2] = 0;
	_buffer[3] = 0;

	//Reading the register and saving all the data read on the buffer
	readICM20948Register(regAddr, regAddrLen, _buffer, readBuffLen, sendValue);

	//Parsing the data read and cutting the bits requested
	uint32_t readData = convertTo32(_buffer, readBuffLen, byteorder);
	readData >>= shift;
	readData = readData & ((1 << (bits)) - 1);

	return readData;
}

void DroneICM20948::setRegBank(uint8_t bankNumber, bool mandatory){

	//Checking if the bank number that its going to be changed to its already selected and mandatory is not true
	if(regBank == bankNumber && !mandatory) return;

	//Writting to the parsed address the complete register modifing the bank number selected
	writeICM20948Register(ICM20948_REG_BANK_SEL, 1, (bankNumber & 0b11) << 4, 1);

	//Setting the actual register bank to the selected one
	regBank = bankNumber;
}

uint8_t *DroneICM20948::convert16to8(int16_t x, uint8_t *big8){
	big8[0] = (uint8_t)((x >> 8) & 0xff);
	big8[1] = (uint8_t)(x & 0xff);
	return big8;
}

uint8_t *DroneICM20948::convert32to8(int32_t x, uint8_t *big8){
	big8[0] = (uint8_t)((x >> 24) & 0xff);
	big8[1] = (uint8_t)((x >> 16) & 0xff);
	big8[2] = (uint8_t)((x >> 8) & 0xff);
	big8[3] = (uint8_t)(x & 0xff);
	return big8;
}