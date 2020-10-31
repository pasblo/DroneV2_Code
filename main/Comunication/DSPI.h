#ifndef DRONE_SPI
#define DRONE_SPI

#include "SPI.h"

/*
Teensys:
- (__MKL26Z64__) Teensy LC
- (__MK20DX128__) Teensy 3.0
- (__MK20DX256__) Teensy 3.1 & 3.2
- (__MK64FX512__) Teensy 3.5
- (__MK66FX1M0__) Teensy 3.6
- (__IMXRT1052__) Beta Teensy 4.0 & 4.1
- (__IMXRT1062__) Teensy 4.0 & 4.1
*/
	
#if defined(SPI_SLAVE)
	#include <DMAChannel.h>
#endif

class DroneSPI{
	public:

		#if defined(SPI_MASTER) //DONE, Testing TODO

			#if defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
				void begin(uint32_t freq, SPIClass& _port, uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE0);
			#else
				void begin(uint32_t freq, uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE0);
			#endif

			bool transfer16Word(uint16_t word, uint8_t csPin);
			bool transfer8Word(uint8_t word, uint8_t csPin);
			bool comunication(const void* buffTX, void* buffRX, uint8_t csPin);

		#elif defined(SPI_SLAVE)

			#if defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
				void begin(uint8_t csPin, SPIClass& _port, uint8_t _intPin = NULL);
			#else
				void begin(uint8_t csPin, uint8_t _intPin = NULL);
			#endif

			bool deleteCache();

			uint16_t getRxBufferElement(uint8_t pos){ return rx_buffer[pos]; }

			void clearInterrupt(){ rx.clearInterrupt(); }

			uint32_t getCounter(){ return counter; }

			void resetCounter(){ counter = 0; }

			uint32_t counter;

		#endif

	private:

		SPIClass *port;

		SPISettings SPI_Settings;

		#if defined(SPI_SLAVE)
			DMAChannel rx;
			uint16_t rx_buffer[1]; //DMAMEM

			uint8_t intPin;

			volatile uint32_t spi_map;

			bool initSPISlaveDMA();

			bool initSPISlave();

			//Interrupt for reciveing??
			static void rxISR();



		#endif
};
extern DroneSPI DSPI;
#endif