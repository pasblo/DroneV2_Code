#include "DSPIV4.h"
#include "arduino.h"
#include <SPI.h>
#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Pinout.h>

DroneSPI DSPI;

#if defined(MAIN_PROCESSOR)
	void DroneSPI::begin(uint32_t clock, uint8_t bitOrder, uint8_t dataMode){
		internalSettings = new SPISettings(clock, bitOrder, dataMode);
		SPI.begin();

	}
	void DroneSPI::transferInformation(uint8_t csDirection, uint32_t bufferLength){
		SPI.beginTransaction(*internalSettings);
		digitalWrite(csDirection, LOW);
		SPI.transfer(&tx_buffer, &rx_buffer, bufferLength);
		digitalWrite(csDirection, HIGH);
		SPI.endTransaction();
	}
#elif defined(EXTERNAL_PROCESSOR)
#elif defined(ESP_PROCESOR)
#endif