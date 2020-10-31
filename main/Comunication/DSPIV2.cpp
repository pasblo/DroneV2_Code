#include "DSPIV2.h"
#include "core_pins.h"
#include "arduino.h"

DroneSPI DSPI;

//DroneSPISettings constants for the Teensy LC processor
const uint16_t SPISettings::br_div_table[30] = {
	2, 4, 6, 8, 10, 12, 14, 16, 20, 24,
	28, 32, 40, 48, 56, 64, 80, 96, 112, 128,
	160, 192, 224, 256, 320, 384, 448, 512, 640, 768,
};

const uint8_t SPISettings::br_clock_table[30] = {
	SPI_BR_SPPR(0) | SPI_BR_SPR(0),
	SPI_BR_SPPR(1) | SPI_BR_SPR(0),
	SPI_BR_SPPR(2) | SPI_BR_SPR(0),
	SPI_BR_SPPR(3) | SPI_BR_SPR(0),
	SPI_BR_SPPR(4) | SPI_BR_SPR(0),
	SPI_BR_SPPR(5) | SPI_BR_SPR(0),
	SPI_BR_SPPR(6) | SPI_BR_SPR(0),
	SPI_BR_SPPR(7) | SPI_BR_SPR(0),
	SPI_BR_SPPR(4) | SPI_BR_SPR(1),
	SPI_BR_SPPR(5) | SPI_BR_SPR(1),
	SPI_BR_SPPR(6) | SPI_BR_SPR(1),
	SPI_BR_SPPR(7) | SPI_BR_SPR(1),
	SPI_BR_SPPR(4) | SPI_BR_SPR(2),
	SPI_BR_SPPR(5) | SPI_BR_SPR(2),
	SPI_BR_SPPR(6) | SPI_BR_SPR(2),
	SPI_BR_SPPR(7) | SPI_BR_SPR(2),
	SPI_BR_SPPR(4) | SPI_BR_SPR(3),
	SPI_BR_SPPR(5) | SPI_BR_SPR(3),
	SPI_BR_SPPR(6) | SPI_BR_SPR(3),
	SPI_BR_SPPR(7) | SPI_BR_SPR(3),
	SPI_BR_SPPR(4) | SPI_BR_SPR(4),
	SPI_BR_SPPR(5) | SPI_BR_SPR(4),
	SPI_BR_SPPR(6) | SPI_BR_SPR(4),
	SPI_BR_SPPR(7) | SPI_BR_SPR(4),
	SPI_BR_SPPR(4) | SPI_BR_SPR(5),
	SPI_BR_SPPR(5) | SPI_BR_SPR(5),
	SPI_BR_SPPR(6) | SPI_BR_SPR(5),
	SPI_BR_SPPR(7) | SPI_BR_SPR(5),
	SPI_BR_SPPR(4) | SPI_BR_SPR(6),
	SPI_BR_SPPR(5) | SPI_BR_SPR(6)
};

//DroneSPISettings constants for the 3.x processors
const uint16_t SPISettings::ctar_div_table[23] = {
	2, 3, 4, 5, 6, 8, 10, 12, 16, 20, 24, 32, 40,
	56, 64, 96, 128, 192, 256, 384, 512, 640, 768
};
const uint32_t SPISettings::ctar_clock_table[23] = {
	SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0),
	SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0),
	SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0),
	SPI_CTAR_PBR(2) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0),
	SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0),
	SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1),
	SPI_CTAR_PBR(2) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0),
	SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1),
	SPI_CTAR_PBR(0) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2),
	SPI_CTAR_PBR(2) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(0),
	SPI_CTAR_PBR(1) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2),
	SPI_CTAR_PBR(0) | SPI_CTAR_BR(4) | SPI_CTAR_CSSCK(3),
	SPI_CTAR_PBR(2) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2),
	SPI_CTAR_PBR(3) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2),
	SPI_CTAR_PBR(0) | SPI_CTAR_BR(5) | SPI_CTAR_CSSCK(4),
	SPI_CTAR_PBR(1) | SPI_CTAR_BR(5) | SPI_CTAR_CSSCK(4),
	SPI_CTAR_PBR(0) | SPI_CTAR_BR(6) | SPI_CTAR_CSSCK(5),
	SPI_CTAR_PBR(1) | SPI_CTAR_BR(6) | SPI_CTAR_CSSCK(5),
	SPI_CTAR_PBR(0) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6),
	SPI_CTAR_PBR(1) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6),
	SPI_CTAR_PBR(0) | SPI_CTAR_BR(8) | SPI_CTAR_CSSCK(7),
	SPI_CTAR_PBR(2) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6),
	SPI_CTAR_PBR(1) | SPI_CTAR_BR(8) | SPI_CTAR_CSSCK(7)
};

DroneSPI::DroneSPI() {

	//Conecting a internal clock to the SPIn CLK hardware
	#if defined(USE_SPI0_CLK)
		SIM_SCGC6 |= SIM_SCGC6_SPI0;
	#elif defined(USE_SPI1_CLK) //Only available if using teensy that supports it
		SIM_SCGC6 |= SIM_SCGC6_SPI1;
	#endif

	//Configuring the MRC (Module Configuration Register) register to make the SPI device as master or slave
	stopSPI();
	#if defined(SPI_MASTER)
		#if defined(USE_SPI0)
			SPI0_MCR = SPI_MCR_MASTER;
		#elif defined(USE_SPI1)
			SPI1_MCR = SPI_MCR_MASTER;
		#elif defined(USE_SPI2)
			SPI2_MCR = SPI_MCR_MASTER;
		#endif
	#elif defined(SPI_SLAVE)
		#if defined(USE_SPI0)
			SPI0_MCR = SPI_MCR_SLAVE;
		#elif defined(USE_SPI1)
			SPI1_MCR = SPI_MCR_SLAVE;
		#elif defined(USE_SPI2)
			SPI2_MCR = SPI_MCR_SLAVE;
		#endif
	#endif
	startSPI();
}

void DroneSPI::begin(bool CTARn, bool csActiveState) {

	//Resetting the CTAR (Clock and Transfer Attributes Register) that is going to be use din this SPI comunication
	#if defined(SPI_MASTER)
		#if defined(USE_SPI0)
			if(CTARn == 0){ SPI0_CTAR0 = 0; }
			else if(CTARn == 1){ SPI0_CTAR1 = 0; }
		#elif defined(USE_SPI1)
			if(CTARn == 0){ SPI1_CTAR0 = 0; }
			else if(CTARn == 1){ SPI1_CTAR1 = 0; }
		#elif defined(USE_SPI2)
			if(CTARn == 0){ SPI2_CTAR0 = 0; }
			else if(CTARn == 1){ SPI2_CTAR1 = 0; }
		#endif
	#elif defined(SPI_SLAVE)
		#if defined(USE_SPI0)
			if(CTARn == 0){ SPI0_CTAR0_SLAVE = 0; }
		#elif defined(USE_SPI1)
			if(CTARn == 0){ SPI1_CTAR0_SLAVE = 0; }
		#elif defined(USE_SPI2)
			if(CTARn == 0){ SPI2_CTAR0_SLAVE = 0; }
		#endif
	#endif

	setSCK();
	setMISO();
	setMOSI();
	setCS(bool );
}

void DroneSPI::setSCK() {
	#if defined(USING_KINETIS)
	#elif defined(USING_IMXRT)

	#endif
}

void DroneSPI::setMISO() {
	#if defined(USING_KINETIS)
	#elif defined(USING_IMXRT)
	#endif
}

void DroneSPI::setMOSI() {
	#if defined(USING_KINETIS)
	#elif defined(USING_IMXRT)
	#endif
}

void DroneSPI::setCS(bool csActiveState, uint8_t csNumber) {
	#if defined(USING_KINETIS)
	#elif defined(USING_IMXRT)
		if(CS == 10){
			*(portConfigRegister(CS)) = 
		}else if(CS == 36){

		}else if(CS == 37){
			
		}
	#endif
}

void DroneSPI::startSPI() {
	#if defined(USE_SPI0)
		SPI0_MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
	#elif defined(USE_SPI1)
		SPI1_MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
	#elif defined(USE_SPI2)
		SPI2_MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
	#endif
}

void DroneSPI::stopSPI() {
	#if defined(USE_SPI0)
		SPI0_MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
	#elif defined(USE_SPI1)
		SPI1_MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
	#elif defined(USE_SPI2)
		SPI2_MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
	#endif
}

#if defined(SPI_MASTER)

#elif defined(SPI_SLAVE)

#endif