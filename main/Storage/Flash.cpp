#include "Flash.h"
#include "Arduino.h"
#include "SPI.h" //TODO, sustitude by DSPI
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Time/DroneTime.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Pinout.h"

Flash DFlash;

bool Flash::begin(void){
	SPI.begin(); //TODO, this begin is on the begin class 

	flashStatus = WAITING;
	nextAddress = 0;
	lastRuntime = 0;

	pinMode(CS_FLASH_PIN, OUTPUT);
	digitalWrite(CS_FLASH_PIN, HIGH);
	return true;

}

bool Flash::reset(void){
	lastRuntime = micros();

	if(!_writeEnable() || !(_readStat1() & BUSY)) return false;

	noInterrupts(); //TODO, check if we can remove reduntant begin transactions and noInterrupts
	SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

	digitalWrite(CS_FLASH_PIN, LOW);

	SPI.transfer(CHIPERASE);

	digitalWrite(CS_FLASH_PIN, HIGH);

	SPI.endTransaction();
}

bool Flash::writeNext(void){ //DO not take lastRuntime as the correct time if the function returned false
	
	FlashStatus initStatus = flashStatus;
	uint8_t newByte;

	lastRuntime = micros();

	//Checking that there is something to write
	if(flashStatus == WAITING || flashStatus == UNINITIALIZED) return false;

	//Enabling the 4 byte addressing mode
	//if(flashStatus == WCONFIGURING) 
	_enable4ByteAddress();

	//Checking and preparing the flash for write
	//TODO, future BUGBUG, sustitude nextAddres with only the initial addess and do not perform this operation every write cycle
	//If the chip is busy we just pass to the next task, we are not going to spend time here waiting as silly people
	//if(flashStatus == WCONFIGURING){
	if(nextAddress + dataSize + USTIMEBYTES >= FLASH_CAPACITY || !_writeEnable() || !(_readStat1() & BUSY)) return false;
	//}
	//If the chip is busy we just pass to the next task, we are not going to spend time here waiting as silly people

	//if(flashStatus == WCONFIGURING){
	noInterrupts(); //TODO, check if we can remove reduntant begin transactions and noInterrupts
	SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
	//flashStatus = WDATA;
	//}

	digitalWrite(CS_FLASH_PIN, LOW);

	SPI.transfer(PAGEPROG);

	//Transfering address to write
	SPI.transfer(Highest(nextAddress));
	SPI.transfer(Higher(nextAddress));
	SPI.transfer(Hi(nextAddress));
	SPI.transfer(Lo(nextAddress));

	if(flashStatus == WDATA){ //TODO, introduce here the code to write the dataToWrite step by step
		newByte = *dataToWrite++;
		SPI.transfer(newByte); //Write here next byte from array

		dataElement++;

		if(dataElement >= dataSize){
			#if defined(USE_64BIT_TIME)
			flashStatus = WTIME_HIEST;
			#endif // USE_64BIT_TIME
			flashStatus = WTIME_HI;
		}

	}else {
		switch(flashStatus){
			#if defined(USE_64BIT_TIME)
			case WTIME_HIEST:
				SPI.transfer(Highest(transmissionStartTime));
				flashStatus = WTIME_HIER;
				break;
			case WTIME_HIER:
				SPI.transfer(Higher(transmissionStartTime));
				flashStatus = WTIME_HI;
				break;
			#endif // USE_64BIT_TIME
			case WTIME_HI:
				newByte = Hi(transmissionStartTime);
				SPI.transfer(Hi(transmissionStartTime));
				flashStatus = WTIME_LO;
				break;
			case WTIME_LO:
				newByte = Lo(transmissionStartTime);
				SPI.transfer(Lo(transmissionStartTime));
				flashStatus = WAITING;

				//Super temporal end SPI bus, TODO, move OUT HERE
				_disable4ByteAddress();
				SPI.endTransaction();
				interrupts();
				break;
		}
	}
	Serial.print("Flash status: ");
	Serial.print(initStatus);
	Serial.print(", data: ");
	Serial.print(newByte);
	Serial.print(" written to: ");
	Serial.print(nextAddress);
	Serial.print(" in: ");
	nextAddress++;
	digitalWrite(CS_FLASH_PIN, HIGH);
	lastRuntime = micros() - lastRuntime;
	Serial.print(lastRuntime);
	Serial.println("us");
	return true;
}

template <class T> bool Flash::writeStruct(const T& data) {
	transmissionStartTime = micros();
	dataToWrite = ((const uint8_t*)(const void*)&data);
	dataSize = sizeof(data);
	flashStatus = WDATA;
	dataElement = 0;
}

uint8_t Flash::_readStat1(void) {

	//Begin the spi com and send command to read stat 1. TODO, move to the DSPI class and sustitude all related to SPI in this class
	noInterrupts();
	SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

	digitalWrite(CS_FLASH_PIN, LOW);

	//Telling the flash that we want to read stat1
	SPI.transfer(READSTAT1);

	//Reading the next upcoming byte
	uint8_t stat1 = SPI.transfer(0x00);

	//Deselecting the flash chip
	digitalWrite(CS_FLASH_PIN, HIGH);

	return stat1;
}

uint8_t Flash::_writeEnable(void){ //Use flash mode to select between flight status and gound status

	noInterrupts();
	SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

	digitalWrite(CS_FLASH_PIN, LOW); //TODO, convert this to a definition

	SPI.transfer(WRITEENABLE);

	digitalWrite(CS_FLASH_PIN, HIGH); //TODO, convert this to a definition

	if (_readStat1() & 0x02){
		return true;
	}
	Serial.println("NotEnabled");
	return false;
}

void Flash::_enable4ByteAddress(void){
	noInterrupts();
	SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

	digitalWrite(CS_FLASH_PIN, LOW);

	SPI.transfer(ADDR4BYTE_EN);

	digitalWrite(CS_FLASH_PIN, HIGH);
}

void Flash::_disable4ByteAddress(void){
	noInterrupts();
	SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

	digitalWrite(CS_FLASH_PIN, LOW);

	SPI.transfer(ADDR4BYTE_DIS);

	digitalWrite(CS_FLASH_PIN, HIGH);
}