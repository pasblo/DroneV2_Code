#ifndef ICM20948_AK09916_DRIVER_H
#define ICM20948_AK09916_DRIVER_H

#include "ICM20948.h"

//Register directions for magnetometer (AK09916)
#define AK09916_WIA2 0x01							//< magnetometer chip id register
#define AK09916_HXL 0x11							//< magnetometer X axis low bits
#define AK09916_HXH 0x12							//< magnetometer X axis high bits
#define AK09916_HYL	0x13							//< magnetometer Y axis low bits
#define AK09916_HYH 0x14							//< magnetometer Y axis high bits
#define AK09916_HZL 0x15							//< magnetometer Z axis low bits
#define AK09916_HZH 0x16							//< magnetometer Z axis high bits
#define AK09916_CNTL2 0x31							//< magnetometer mode switching register
#define AK09916_CNTL3 0x32							//< magnetometer reset register

//Chip identifiers
#define AK09916_ID 0x09								//< Magnetometer chip id
#define MAG_ADDR 0x0C								//< magnetometer i2c address

//Other constants
#define I2C_MASTER_RESETS_BEFORE_FAIL 4				//< number of times trying to reset the i2c master at maximun
#define I2C_MAX_TRIES 100							//< number of times trying to transmit to the magnetometer at maximun

bool DroneICM20948::beginMagnetometer(){

	//Setting the bank to 0
	setRegBank(0);
	
	//Enabling the bypass I2c interface to make the internal master take control of the magnetometer
	writeICM20948RegisterBits(ICM20948_B0_REG_INT_PIN_CFG, 1, 1, 1, 1, 1);

	//Setting the bank to 3
	setRegBank(3);

	//Setting up the ORD configuration for external sensors, even though ig going to be rarely used
	writeICM20948Register(ICM20948_B3_I2C_MST_ODR_CONFIG, 1, 4, 1);

	//Setting up the i2c master clock frequency
	writeICM20948RegisterBits(ICM20948_B3_I2C_MST_CTRL, 1, 1, 1, 4, 0);

	//Setting up the 12c master transition action to a stop between reads
	writeICM20948RegisterBits(ICM20948_B3_I2C_MST_CTRL, 1, 1, 1, 1, 4);

	//Setting the bank to 0
	setRegBank(0);

	//Enabling the i2c master located on the sensor
	writeICM20948RegisterBits(ICM20948_B0_USER_CTRL, 1, 1, 1, 1, 5);

	//Resetting the magnetometer
	if(!resetMagnetometer()) return false;

	//Trying to identify the magnetometer
	bool auxI2cSetupFailed = true;
	for (int i = 0; i < I2C_MASTER_RESETS_BEFORE_FAIL; i++) {
		if (readExternalRegister(0x8C, AK09916_WIA2) != AK09916_ID) {
			if(!resetMagnetometer()) return false;
		} else {
			auxI2cSetupFailed = false;
			break;
		}
		delay(1);
	}

	//Checking if the id of the magnetometer was localized
	if(auxI2cSetupFailed){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, AK09916_ID_ERROR);
		return false;
	}

	//Setting the bank to 3
	setRegBank(3);

	//Set up slave 0 to proxy read Mag
	writeICM20948Register(ICM20948_B3_I2C_SLV0_ADDR, 1, 0x8C, 1, false);
	writeICM20948Register(ICM20948_B3_I2C_SLV0_REG, 1, 0x10, 1, false);
	writeICM20948Register(ICM20948_B3_I2C_SLV0_CTRL, 1, 0x89, 1, false);

	return true;
}

bool DroneICM20948::resetMagnetometer(){

	//Setting the bank register to 0
	setRegBank(0);

	//Resetting the I2C master
	writeICM20948RegisterBits(ICM20948_B0_USER_CTRL, 1, 1, 1, 1, 1);

	//Normalized wait before reading if the reset bit has been cleared
	delay(4);

	//Saving the initial time at wich the reset loop wait is going to be held
	internalTime = millis();

	//Waiting to the autoclearing of the reset bit
	while(readICM20948RegisterBits(ICM20948_B0_USER_CTRL, 1, 1, 1, 1)){

		//Checking if we have not passed the timeout
		if(millis() - internalTime > RESET_TIMEOUT){
			DErrorCodes.setErrorCode(ICM20948_SOURCE, I2C_MASTER_RESET_TIMEOUT_ERROR);
			return false;
		}

		//Waiting to not ask very fast
		delay(1);
	}

	//Normalized wait after the reset to make all settle
	delay(1);

	//Resetting the magnetometer itself
	if(!writeMagRegister(AK09916_CNTL3, 1)) return false;

	//Normalized wait before reading if the reset bit has been cleared
	delay(1);

	//Saving the initial time at wich the reset loop wait is going to be held
	internalTime = millis();

	//Waiting to the autoclearing of the reset bit
	while(readMagRegister(AK09916_CNTL3)){

		//Checking if we have not passed the timeout
		if(millis() - internalTime > RESET_TIMEOUT){
			DErrorCodes.setErrorCode(ICM20948_SOURCE, AK09916_RESET_TIMEOUT_ERROR);
			return false;
		}

		//Waiting to not ask very fast
		delay(1);
	}

	//Normalized wait after the reset to make all settle
	delay(1);

	return true;
}

bool DroneICM20948::setMagMode(icm20948_mag_mode_e newMode){

	//Checking if the sensor was initizalied
	if(!chipInitialized){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, CHIP_UNINITIALIZED_ERROR);
		return false;
	}

	//Checking if the mode is already selected
	if(magMode == newMode) return false;
	magMode = newMode;

	//Choosing the characteristics dependig on the mode selected
	switch(newMode){
		case ICM20948_MAG_SLEEP_MODE:
			droneMagnetometer.enabled = false;
			droneMagnetometer.rateDiv = AK09916_MAG_FREQ_SHUTDOWN;
			break;

		case ICM20948_MAG_CALIB_MODE:
			droneGyroscope.enabled = true;
			droneMagnetometer.rateDiv = AK09916_MAG_FREQ_SELF_TEST;
			break;

		case ICM20948_MAG_STANDARD_MODE:
			droneMagnetometer.enabled = true;
			droneMagnetometer.rateDiv = AK09916_MAG_FREQ_100_HZ;
			magLPF.k = 0;
			break;
	}

	//Following the datasheet instructions, we first shutdown it, then wait at least 1ms, then set the new mode
	if(!writeMagRegister(AK09916_CNTL2, AK09916_MAG_FREQ_SHUTDOWN)) return false;
	delay(1);
	if(!writeMagRegister(AK09916_CNTL2, droneMagnetometer.rateDiv)) return false;

	return true;
}

bool DroneICM20948::magSelfTest(){

	//Declaring mag self test variables
	int16_t outputSelfTestX, outputSelfTestY, outputSelfTestZ;

	//Reading the self test results
	outputSelfTestX = readMagRegister(AK09916_HXH) << 8 | readMagRegister(AK09916_HXL);
	outputSelfTestY = readMagRegister(AK09916_HYH) << 8 | readMagRegister(AK09916_HYL);
	outputSelfTestZ = readMagRegister(AK09916_HZH) << 8 | readMagRegister(AK09916_HZL);

	//Checking if the self test values are in the range specified
	if(abs(outputSelfTestX) > 5000 || abs(outputSelfTestY) > 5000 || abs(outputSelfTestZ) > 5000){
		DErrorCodes.setErrorCode(ICM20948_SOURCE, MAG_SELF_TEST_ERROR);
	}

	return true;
}

uint8_t DroneICM20948::auxillaryRegisterTransaction(bool read, uint8_t slvAddr, uint8_t regAddr, uint8_t value){

	//Bank set to 3
	setRegBank(3);

	if(read){
		slvAddr |= 0x80;
	}else{
		writeICM20948Register(ICM20948_B3_I2C_SLV4_DO, 1, value, 1);
	}

	//Introducing the slave addres and the register
	writeICM20948Register(ICM20948_B3_I2C_SLV4_ADDR, 1, slvAddr, 1);
	writeICM20948Register(ICM20948_B3_I2C_SLV4_REG, 1, regAddr, 1);
	writeICM20948Register(ICM20948_B3_I2C_SLV4_CTRL, 1, 0x80, 1);

	//Bank set to 0
	setRegBank(0);

	//Trying to transmit the data
	uint8_t tries = 0;
	while(!readICM20948RegisterBits(ICM20948_B0_I2C_MST_STATUS, 1, 1, 1, 6)){
		if(tries >= I2C_MAX_TRIES){
			DErrorCodes.setErrorCode(ICM20948_SOURCE, AK09916_TRANSMISSION_ERROR);
			return (uint8_t) false;
		}
		tries++;
	}

	//Obtaining the data
	if(read){

		//Bank set to 3
		setRegBank(3);

		//Reading the data
		uint8_t readData;
		readICM20948Register(ICM20948_B3_I2C_SLV4_DI, 1, &readData, 1);

		return readData;
	}

	return (uint8_t) true;
}

bool DroneICM20948::writeExternalRegister(uint8_t slvAddr, uint8_t regAddr, uint8_t value) {
	return (bool) auxillaryRegisterTransaction(false, slvAddr, regAddr, value);
}

bool DroneICM20948::writeMagRegister(uint8_t magRegAddr, uint8_t value) {
	return writeExternalRegister(MAG_ADDR, magRegAddr, value);
}

bool DroneICM20948::writeMagRegisterBits(uint8_t magRegAddr, uint8_t value, uint8_t bits, uint8_t shift){

	//Reading the actual data stored on the reqgister
	uint8_t val = readMagRegister(magRegAddr);

	uint32_t mask = (1 << (bits)) - 1;
	value &= mask;

	mask <<= shift;
	val &= ~mask;          // remove the current data at that spot
	val |= value << shift; // and add in the new data

	return writeMagRegister(magRegAddr, val);
}

uint8_t DroneICM20948::readExternalRegister(uint8_t slvAddr, uint8_t regAddr) {
	return auxillaryRegisterTransaction(true, slvAddr, regAddr);
}

uint8_t DroneICM20948::readMagRegister(uint8_t magRegAddr) {
	return readExternalRegister(MAG_ADDR | 0x80, magRegAddr);
}

uint8_t DroneICM20948::readMagRegisterBits(uint8_t magRegAddr, uint8_t bits, uint8_t shift){
	uint8_t readData = readMagRegister(magRegAddr);
	readData >>= shift;
	readData = readData & ((1 << (bits)) - 1);
	return readData;
}

#endif