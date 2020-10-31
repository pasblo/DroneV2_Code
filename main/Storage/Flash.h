#ifndef FLASH_H
#define FLASH_H

#include "Arduino.h"
#include "SPI.h" //TODO, sustitude by DSPI
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Time/DroneTime.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/definitions.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Pinout.h"

/*
 * Pakets:
 * [PacketData, PacketWritingTime]
 * All packets of data must include as the first byte an ID
*/

//VAriables to separate states of the flash
#define BUSY 			(0x01)

//Commands to send to the flash
#define READSTAT1 		(0x05)
#define WRITEENABLE 	(0x06)
#define PAGEPROG 		(0x02)
#define ADDR4BYTE_EN 	(0xB7)
#define ADDR4BYTE_DIS 	(0xE9)
#define CHIPERASE 		(0x60)

#define Lo(param) ((char *)&param)[0]
#define Hi(param) ((char *)&param)[1]
#define Higher(param) ((char *)&param)[2]
#define Highest(param) ((char *)&param)[3]

#define FLASH_CAPACITY 	(268435456)

#if defined(USE_64BIT_TIME)
	enum FlashStatus { UNINITIALIZED, WAITING, WCONFIGURING, WDATA, WTIME_HIEST, WTIME_HIER, WTIME_HI, WTIME_LO};
#else
	enum FlashStatus { UNINITIALIZED, WAITING, WCONFIGURING, WDATA, WTIME_HI, WTIME_LO};
#endif

class Flash{
	public:
		Flash(void) {};
		~Flash(void) {};

		//Initializes the flash object
		bool begin();

		//Errases all data in the Flash and resets the flash status
		bool reset();

		//Procedural reading and writting
		bool writeNext();

		template <class T> bool writeStruct(const T& value);
		template <class T> bool readStruct(uint32_t _addr, T& value, uint32_t _sz, bool fastRead = false);

		int getRuntime() {return lastRuntime; }
	private:

		//Waits until the flash is ready
		bool _notBusy(uint32_t timeout = BYTE_WRITE_TIMEOUT);

		uint8_t _readStat1();

		uint8_t _writeEnable();

		void _enable4ByteAddress();
		void _disable4ByteAddress();

		FlashStatus flashStatus = UNINITIALIZED;

		uint32_t nextAddress;

		int lastRuntime;

		uint8_t *dataBuffer;
		uint16_t nextBufferId;

		timeUs_t transmissionStartTime;
		const uint8_t* dataToWrite;
		uint32_t dataSize;
		uint16_t dataElement; //We can only have structs up until 65536 Bytes long
};
extern Flash DFlash;
#endif