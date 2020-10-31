#include "DSPI.h"
#include <SPI.h>

DroneSPI DSPI;

#if defined(SPI_MASTER)
	
	#if defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
		void DroneSPI::begin(uint32_t freq, SPIClass& _port, uint8_t bitOrder, uint8_t dataMode){
			port = &_port;
			port->begin();
			SPI_Settings = SPISettings(freq, bitOrder, dataMode);
		}
	#else
		void DroneSPI::begin(uint32_t freq, uint8_t bitOrder, uint8_t dataMode){
			port = &SPI;
			port->begin();
			SPI_Settings = SPISettings(freq, bitOrder, dataMode);
		}
	#endif

	bool DroneSPI::transfer16Word(uint16_t word, uint8_t csPin){
		port->beginTransaction(SPI_Settings);
		digitalWrite(csPin, LOW);
		port->transfer16(word);
		digitalWrite (csPin, HIGH);
		port->endTransaction();
		return 1;
	}

	bool DroneSPI::transfer8Word(uint8_t word, uint8_t csPin){
		port->beginTransaction(SPI_Settings);
		digitalWrite(csPin, LOW);
		port->transfer(word);
		digitalWrite (csPin, HIGH);
		port->endTransaction();
		return 1;
	}

	bool DroneSPI::comunication(const void* buffTX, void* buffRX, uint8_t csPin){
		port->beginTransaction(SPI_Settings);
		digitalWrite(csPin, LOW);
		port->transfer(buffTX, buffRX, sizeof(buffTX));
		digitalWrite(csPin, HIGH);
		port->endTransaction();
		return 1;
	}

#elif defined(SPI_SLAVE)

	#if defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
		void DroneSPI::begin(uint8_t csPin, SPIClass& _port, uint8_t _intPin){
			port = &_port;
			port->begin();
			port->setCS(csPin);
			initSPISlave();

			rx = DMAChannel(false);
			initSPISlaveDMA();

			intPin = _intPin;
			counter = 0;
		}
	#else

		void DroneSPI::begin(uint8_t csPin, uint8_t _intPin){
			port = &SPI;
			port->begin();
			port->setCS(csPin);
			initSPISlave();

			rx = DMAChannel(false);
			initSPISlaveDMA();

			intPin = _intPin;
			counter = 0;
		}
	#endif

	bool DroneSPI::initSPISlaveDMA(){
		rx.begin(true);
		rx.source((uint16_t &) SPI0_POPR); //SPI0_POPR LPSPI4_RDR
		rx.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI0_RX); //DMAMUX_SOURCE_SPI0_RX DMAMUX_SOURCE_LPSPI4_RX
		rx.attachInterrupt(rxISR);
		rx.interruptAtCompletion(); //TCD->CSR |= DMA_TCD_CSR_INTMAJOR;
		rx.destinationBuffer(rx_buffer, 1 + 1);
		rx.enable();
		return 1;
	}

	bool DroneSPI::initSPISlave(){
		#if defined(__IMXRT1062__)
			LPSPI4_CR &= ~LPSPI_CR_MEN; //Module shutdown
			LPSPI4_CR = LPSPI_CR_RST; //Master Logic reset
			LPSPI4_CR &= ~LPSPI_CR_RST; //Master Logic reset
			LPSPI4_TCR = LPSPI_TCR_FRAMESZ(15); //16Bit Mode
			LPSPI4_DER = LPSPI_DER_RDDE; //RX DMA Request Enable
			LPSPI4_CR |= LPSPI_CR_MEN; //Enable SPI Module
		#else
			/*SPCR |= _BV(SPE);*/
			SPI0_MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
			SPI0_MCR = 0x00000000;
			SPI0_MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
			SPI0_CTAR0_SLAVE = SPI_CTAR_FMSZ(15);
		#endif
		/*SIM_SCGC6 |= SIM_SCGC6_SPI0;
		spi_map = 0x4002C000;

		(*(KINETISK_SPI_t *)spi_map).MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
		(*(KINETISK_SPI_t *)spi_map).MCR = 0x00000000;
		(*(KINETISK_SPI_t *)spi_map).MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
		(*(KINETISK_SPI_t *)spi_map).CTAR0 = 0;
		(*(KINETISK_SPI_t *)spi_map).MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
		(*(KINETISK_SPI_t *)spi_map).CTAR0 = SPI_CTAR_FMSZ(15);
		(*(KINETISK_SPI_t *)spi_map).MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
		(*(KINETISK_SPI_t *)spi_map).MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
		(*(KINETISK_SPI_t *)spi_map).CTAR0 = (*(KINETISK_SPI_t *)spi_map).CTAR0 & ~(SPI_CTAR_CPOL | SPI_CTAR_CPHA);
		(*(KINETISK_SPI_t *)spi_map).MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
		(*(KINETISK_SPI_t *)spi_map).RSER = 0x00020000;*/
		return 1;
	}

	bool DroneSPI::deleteCache(){
		arm_dcache_delete(rx_buffer, 1);
	}

	void DroneSPI::rxISR(){
		Serial.println("Transmision recived");
		DSPI.clearInterrupt();
		asm volatile ("dsb");
		DSPI.counter++;
		//Serial.print("RX Interrupt --> Val = ");
		//Serial.println(DSPI.getRxBufferElement(0));
	}

#endif