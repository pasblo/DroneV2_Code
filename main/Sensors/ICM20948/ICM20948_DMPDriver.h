#ifndef ICM20948_DMP_DRIVER_H
#define ICM20948_DMP_DRIVER_H

#include "ICM20948.h"

//Defining the DMP data
#include "ICM20948_DMP9dof_v6.12.h"
//#include "ICM20948_DMP6dof_v2.0.h"
//#include "ICM20948_DMP6dof_v6.12.h"
//#include "ICM20948_DMP9dof_v4.1.h"

// data output control
#define DATA_OUT_CTL1			(4 * 16)			//< 
#define DATA_OUT_CTL2			(4 * 16 + 2)		//< 
#define DATA_INTR_CTL			(4 * 16 + 12)		//< 
#define FIFO_WATERMARK			(31 * 16 + 14)		//< 

#define MOTION_EVENT_CTL		(4 * 16 + 14)		//< 

#define BAC_RATE                (48  * 16 + 10)
#define B2S_RATE                (48  * 16 +   8)

#define B2S_MTX_00              (208 * 16)
#define B2S_MTX_01              (208 * 16 + 4)
#define B2S_MTX_02              (208 * 16 + 8)
#define B2S_MTX_10              (208 * 16 + 12)
#define B2S_MTX_11              (209 * 16)
#define B2S_MTX_12              (209 * 16 + 4)
#define B2S_MTX_20              (209 * 16 + 8)
#define B2S_MTX_21              (209 * 16 + 12)
#define B2S_MTX_22              (210 * 16)

/* indicates to DMP which sensors are available
1: gyro samples available
2: accel samples available
8: secondary samples available	*/
#define DATA_RDY_STATUS			(8 * 16 + 10)		//< 

#define DMP_START_ADDRESS 0x1000					//< 
#define DMP_LOAD_START 0x90 						//< 
#define MAX_SERIAL_WRITE 16							//< Max number of bytes that can be written at once in SPI bus?
#define MAX_SERIAL_READ 16							//< Max number of bytes that can be written at once in SPI bus?

static const float cfg_mounting_matrix[9] = {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};

//Initialize DMP and FIFO
void DroneICM20948::beginDMP(){
	//I dont really like to write just a number to a register becuse an example tells me so, but in this case
	//there is literaly no information about the registers, seems like the company does not want to release
	//the information

	//Load firmware
	loadFirmwareDMP(dmp_image, sizeof(dmp_image));

	setRegBank(2);

	//Set DMP address
	writeICM20948Register(ICM20948_B2_PRGM_START_ADDRH, 1, DMP_START_ADDRESS, 2);

	unsigned char dataArr[4] = {0};

	static uint8_t data;

	//Reset data output control registers
	writeDMPMemory(DATA_OUT_CTL1, 2, &dataArr[0]);
	writeDMPMemory(DATA_OUT_CTL2, 2, &dataArr[0]);

	//Reset data interrupt control register
	writeDMPMemory(DATA_INTR_CTL, 2, &dataArr[0]);

	//Reset motion event control register
	writeDMPMemory(MOTION_EVENT_CTL, 2, &dataArr[0]);

	//Reset data ready status register
	writeDMPMemory(DATA_RDY_STATUS, 2, &dataArr[0]);

	//Setting FIFO watermark to 80% of the actual FIFO size (overflow triggered at 80%)
	uint8_t big8[2] = {0};
	writeDMPMemory(FIFO_WATERMARK, 2, convert16to8(800, big8));

	//FIFO overflow interrupt
	setRegBank(0);
	data = 0x02;
	writeICM20948Register(ICM20948_B0_INT_ENABLE, 1, &data, 1);
	data = 0x01;
	writeICM20948Register(ICM20948_B0_INT_ENABLE_2, 1, &data, 1);
	
	//Setting as the only priority the FIFO?
	data = 0xe4;
	writeICM20948Register(ICM20948_B0_SINGLE_FIFO_PRIORITY_SEL, 1, &data, 1);

	//Disable HW temp fix? TODO, maybe sustitude this by write bits
	readICM20948Register(ICM20948_B0_HW_FIX_DISABLE, 1, &data, 1);
	data |= 0x08;
	writeICM20948Register(ICM20948_B0_HW_FIX_DISABLE, 1, &data, 1);
	
	//Sample rate for BAC and STEPC
	writeDMPMemory(BAC_RATE, 2, convert16to8(0, dataArr)); //0 -> 56Hz, 1 -> 112Hz, 3 -> 225Hz, 7 -> 450Hz, 15 -> 900Hz
	writeDMPMemory(B2S_RATE, 2, convert16to8(0, dataArr)); //0 -> 56Hz, 1 -> 112Hz, 3 -> 225Hz, 7 -> 450Hz, 15 -> 900Hz

	//Enable FIFO
	writeICM20948Register(ICM20948_B0_FIFO_CFG, 1, (uint32_t)0, 1);

	//Reset all FIFOs
	writeICM20948Register(ICM20948_B0_FIFO_RST, 1, (uint32_t)31, 1);

	//Keep all but Gyro FIFO in reset
	writeICM20948Register(ICM20948_B0_FIFO_RST, 1, (uint32_t)30, 1);

	//Disconnect all sources to the FIFO for being only used by the DMP
	writeICM20948Register(ICM20948_B0_FIFO_EN, 1, (uint32_t)0, 1);
	writeICM20948Register(ICM20948_B0_FIFO_EN2, 1, (uint32_t)0, 1);
};

//Load the dmp firmware
bool DroneICM20948::loadFirmwareDMP(const uint8_t *data_start, uint16_t size_start){

	uint8_t memaddr;
	const uint8_t *data;
	uint16_t size;
	int writeSize;

	memaddr = DMP_LOAD_START;
	data = data_start;
	size = size_start;
	while(size > 0){
		writeSize = min(size, MAX_SERIAL_WRITE);

		//The data was moved across a memory page/bank
		if((memaddr & 0xff) + writeSize > 0x100){

			//Trimming the size to dont pass over multiple pages
			writeSize = (memaddr & 0xff) + writeSize - 0x100;
		}

		//Writting the seleted page to the DMP storage
		if(!writeDMPMemory(memaddr, writeSize, (uint8_t *)data)){ //Casting here??? Why
			return false;
		}

		//Changing to the data block to write
		data += writeSize;
		size -= writeSize;
		memaddr += writeSize;
	}

	return true;
};

void DroneICM20948::applyMountingMatrix(){ //TODO move to math

	int32_t mounting_mq30[9];

	//Convert matrix to mounting matrix int32_t
	for(uint8_t i = 0; i < 9; ++i)
		mounting_mq30[i] = (int32_t)(cfg_mounting_matrix[i] * (1 << 30));

	//Convert matrix to mounting matrix int8_t
	for (uint8_t i = 0; i < 9; ++i)
		mountingMatrix[i] = mounting_mq30[i] >> 30;

	//Apply matrix
	setChipToBodyAxisQuaternion(0.0);

	//Update DMP B2S according to new matrix in q30
	setB2SMatrix((int*)mounting_mq30);
}

void DroneICM20948::setB2SMatrix(int *b2sMtx){

	unsigned char big8[4] = {0};

	writeDMPMemory(B2S_MTX_00, 4, convert32to8(b2sMtx[0], big8));
	writeDMPMemory(B2S_MTX_01, 4, convert32to8(b2sMtx[1], big8));
	writeDMPMemory(B2S_MTX_02, 4, convert32to8(b2sMtx[2], big8));
	writeDMPMemory(B2S_MTX_10, 4, convert32to8(b2sMtx[3], big8));
	writeDMPMemory(B2S_MTX_11, 4, convert32to8(b2sMtx[4], big8));
	writeDMPMemory(B2S_MTX_12, 4, convert32to8(b2sMtx[5], big8));
	writeDMPMemory(B2S_MTX_20, 4, convert32to8(b2sMtx[6], big8));
	writeDMPMemory(B2S_MTX_21, 4, convert32to8(b2sMtx[7], big8));
	writeDMPMemory(B2S_MTX_22, 4, convert32to8(b2sMtx[8], big8));
}

bool DroneICM20948::writeDMPMemory(uint16_t address, uint16_t bytesDataLength, const uint8_t *data){
	
	uint16_t bytesWritten = 0;
	uint16_t burstWriteLen;
	uint8_t startAddr;

	//Checking if the data contains information
	if(!data) return false;

	//Turning off the LP functionality
	//writeICM20948RegisterBits(ICM20948_B0_PWR_MGMT_1, 1, 0, 1, 1, 5);

	//Setting the page bank to be able to write to that bank
	setDMPPageBank(address >> 8);

	while(bytesWritten < bytesDataLength){

		//Selecting the starting address of this burst write
		startAddr = (address & 0xff);

		//Writting the starting address of the burst write
		writeICM20948Register(ICM20948_B0_DMP_START_ADDR, 1, startAddr, 1);

		//Calculating the length of the packet to write
		burstWriteLen = min(MAX_SERIAL_WRITE, bytesDataLength - bytesWritten);

		//Writting the next packet of bits
		writeICM20948Register(ICM20948_B0_DMP_MEM_R_W, 1, data[bytesWritten], burstWriteLen); //Maybe sustitude data[bytesWritten] by &data[bytesWritten]

		//Incrementing counters to write
		bytesWritten += burstWriteLen;
		address += burstWriteLen;

	}

	//Turning on the LP functionality
	//writeICM20948RegisterBits(ICM20948_B0_PWR_MGMT_1, 1, 0, 1, 1, 5);

	return true;
}

bool DroneICM20948::readDMPMemory(uint16_t address, uint16_t bytesDataLength, uint8_t *data){

	uint16_t bytesRead = 0;
	uint16_t burstReadLen;
	uint8_t startAddr;
	uint8_t dat[MAX_SERIAL_READ] = {0};

	//Checking if the data contains information
	if(!data) return false;

	//Turning off the LP functionality
	//writeICM20948RegisterBits(ICM20948_B0_PWR_MGMT_1, 1, 0, 1, 1, 5);

	//Setting the page bank to be able to write to that bank
	setDMPPageBank(address >> 8);

	while(bytesRead < bytesDataLength){

		//Selecting the starting address of this burst write
		startAddr = (address & 0xff);

		//Writting the starting address of the burst read
		writeICM20948Register(ICM20948_B0_DMP_START_ADDR, 1, startAddr, 1);

		//Calculating the length of the packet to read
		burstReadLen = min(MAX_SERIAL_READ, bytesDataLength - bytesRead);

		//Reading the next packet of bits
		readICM20948Register(ICM20948_B0_DMP_MEM_R_W, 1, &dat[bytesRead], burstReadLen);

		//Incrementing counters to read
		bytesRead += burstReadLen;
		address += burstReadLen;

	}

	for(uint8_t i = 0; i < bytesDataLength; i++){
		*data = dat[i];
		data++;
	}

	//Turning on the LP functionality
	//writeICM20948RegisterBits(ICM20948_B0_PWR_MGMT_1, 1, 0, 1, 1, 5);

	return true;
}

/*bool DroneICM20948::writeDMPRegister(uint16_t reg, uint16_t bytesDataLength, const uint8_t *data){

	uint8_t bytesWritten = 0;
	uint16_t burstWriteLen;
	uint8_t regOnly = (reg & 0x7F);

	//Turning off the LP functionality
	//writeICM20948RegisterBits(ICM20948_B0_PWR_MGMT_1, 1, 0, 1, 1, 5);

	//Setting the page bank to be able to write to that bank
	setDMPPageBank(reg >> 7);

	while(bytesWritten < bytesDataLength){

		//Calculating the length of the packet to write
		burstWriteLen = min(MAX_SERIAL_WRITE, bytesDataLength - bytesWritten);

		writeICM20948Register()
	}
}

bool DroneICM20948::readDMPRegister(uint16_t address, uint16_t bytesDataLength, uint8_t *data){

}*/

void DroneICM20948::setDMPPageBank(uint8_t bankNumber, bool mandatory){

	//Checking if the bank number that its going to be changed to its already selected and mandatory is not true
	if(pageBank == bankNumber && !mandatory) return;

	//Bank set to 0
	setRegBank(0);

	//Writting to the parsed address the complete register modifing the bank number selected
	writeICM20948Register(ICM20948_B0_DMP_BANK_SEL, 1, bankNumber, 1);

	//Setting the actual page bank to the selected one
	pageBank = bankNumber;
}

/*void DroneICM20948::setDMPAddress(){

	uint8_t dmp_cfg[2] = {0};
	uint16_t config;

	//

	//Transform the addr to an array
	dmp_cfg[0] = (unsigned char)((config >> 8) & 0xff);
	dmp_cfg[1] = (unsigned char)(config & 0xff);

	//Write the DMP start address
	writeICM20948Register(ICM20948_B2_PRGM_START_ADDRH, 1, dmp_cfg, 2);
}*/

//Read and process dmp packet if new packet available
bool DroneICM20948::checkAndProcessPacketDMP();

//getters
/*void DroneICM20948::getQuaternionDMP();
void DroneICM20948::getGyroscopeDMP();
void DroneICM20948::getAccelerometerDMP();
void DroneICM20948::getMagnetometerDMP();*/
#endif