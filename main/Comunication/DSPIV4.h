#ifndef DRONE_SPI
#define DRONE_SPI

#include "arduino.h"
#include <SPI.h>
#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Pinout.h>

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define MAX_BUFFER_LENGHT 1

class DroneSPI{
	public:

		#if defined(MAIN_PROCESSOR)
			void begin(uint32_t clock, uint8_t bitOrder, uint8_t dataMode);
			void transferInformation(uint8_t csDirection, uint32_t bufferLength);
		#elif defined(EXTERNAL_PROCESSOR)
			void begin(uint8_t bitOrder, uint8_t dataMode);
		#elif defined(ESP_PROCESOR)
		#endif
	private:
		SPISettings *internalSettings;

		//Store the data being recived
		uint8_t rx_buffer[MAX_BUFFER_LENGHT] = {0};

		//Store the data being sent
		uint8_t tx_buffer[MAX_BUFFER_LENGHT] = {1};
};
extern DroneSPI DSPI;
#endif