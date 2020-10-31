
//Including general libraries
#include "TeensySPI.h"
#include "core_pins.h"
#include "arduino.h"

//Including spcific libraries
#if defined(EXTERNAL_PROCESSOR)
	#include "DMAChannel.h"
#endif

/*
BASE STRUCTURE FOR FUNCTION:

#if defined(MAIN_PROCESSOR)
		#if defined(USING_KINETIS)
		#elif defined(USING_IMXRT)
		#endif
		return true;
	#elif defined(EXTERNAL_PROCESSOR)
		return true;
	#endif
	return false;
*/

//TeensySPISettings constants for the 3.2 & 3.5 processors
#if defined(USING_KINETIS) && defined(MAIN_PROCESSOR)
	const uint16_t TeensySPISettings::ctar_div_table[23] = {
		2, 3, 4, 5, 6, 8, 10, 12, 16, 20, 24, 32, 40,
		56, 64, 96, 128, 192, 256, 384, 512, 640, 768
	};
	const uint32_t TeensySPISettings::ctar_clock_table[23] = {
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
#endif

//Hardware definitions
#if defined(MAIN_PROCESSOR)
	#if defined(NOPROCESSPOR) //Teensy 3.5 __MK64FX512__
		const TeensySPI::SPI_Hardware_t TeensySPI::spi_hardware = {

			//Clock sourcing and misc config
			SIM_SCGC6, SIM_SCGC6_SPI0, 4, IRQ_SPI0,

			//DMA channel config
			32767, DMAMUX_SOURCE_SPI0_TX, DMAMUX_SOURCE_SPI0_RX, NULL,

			//MISO pin config
			SDO_PIN, PORT_PCR_MUX(2),

			//MOSI pin config
			SDI_PIN, PORT_PCR_MUX(2),

			//SCK pin config
			SCK_PIN, PORT_PCR_MUX(2),

			//CS pin config (This procesor acts as the main processor, so is does not have a CS reciveing pin, the pin is defined when doing the transaction to be able to transmit to different slaves)
			255, 0, 0};

		TeensySPI TSPI((uintptr_t)&KINETISK_SPI0, (uintptr_t)&TeensySPI::spi_hardware);

	#elif defined(__IMXRT1062__) //Teensy 4.1 __IMXRT1062__
		const TeensySPI::SPI_Hardware_t  TeensySPI::spi_hardware = {

			//Clock sourcing and misc config
			CCM_CCGR1, CCM_CCGR1_LPSPI4(CCM_CCGR_ON),

			//DMA channel config
			DMAMUX_SOURCE_LPSPI4_TX, DMAMUX_SOURCE_LPSPI4_RX, NULL,

			//MISO pin config
			SDO_PIN, 3 | 0x10, 0, IOMUXC_LPSPI4_SDI_SELECT_INPUT, //They really have a full liquor bar in their head about SPI

			//MOSI pin config
			SDI_PIN, 3 | 0x10, 0, IOMUXC_LPSPI4_SDO_SELECT_INPUT,

			//SCK pin config
			SCK_PIN, 3 | 0x10, 0, IOMUXC_LPSPI4_SCK_SELECT_INPUT,

			//CS pin config (This procesor acts as the main processor, so is does not have a CS reciveing pin, the pin is defined when doing the transaction to be able to transmit to different slaves)
			255, 0, 0, 0, 0};

		TeensySPI TSPI((uintptr_t)&IMXRT_LPSPI4_S, (uintptr_t)&TeensySPI::spi_hardware);
	#endif

#elif defined(EXTERNAL_PROCESSOR)
	#if defined(__MK64FX512__) //Teensy 3.2 __MK20DX256__
		const TeensySPI::SPI_Hardware_t TeensySPI::spi_hardware = {

			//Clock sourcing and misc config
			SIM_SCGC6, SIM_SCGC6_SPI0, 4, IRQ_SPI0,

			//DMA channel config
			32767, DMAMUX_SOURCE_SPI0_TX, DMAMUX_SOURCE_SPI0_RX, processData,

			//MISO pin config
			SDI_PIN, PORT_PCR_MUX(2),

			//MOSI pin config
			SDO_PIN, PORT_PCR_DSE | PORT_PCR_MUX(2),

			//SCK pin config
			SCK_PIN, PORT_PCR_DSE | PORT_PCR_MUX(2),

			//CS pin config, the chip select in the slave has to be CS0
			CS_PIN, PORT_PCR_MUX(2), 0x1};

		TeensySPI TSPI((uintptr_t)&KINETISK_SPI0, (uintptr_t)&TeensySPI::spi_hardware);
	#endif
#endif

//TeensySPI function definitions
bool TeensySPI::begin() {

	//*_moduleSettings = new TeensySPISettings();

	#if defined(MAIN_PROCESSOR)
		#if defined(USING_KINETIS) //TODO

			//Activating the clock gate
			/*hardware().clock_gate_register |= hardware().clock_gate_mask;

			//Initializing the CTAR registers
			port().MCR = SPI_MCR_MDIS | SPI_MCR_HALT | SPI_MCR_PCSIS(0x1F);
			port().CTAR0 = SPI_CTAR_FMSZ(7) | SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1);
			port().CTAR1 = SPI_CTAR_FMSZ(15) | SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1);
			port().MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F);

			//Linking the pins selected to the respective registers
			volatile uint32_t *reg;
			reg = portConfigRegister(hardware().mosi_pin);
			*reg = hardware().mosi_mux;
			reg = portConfigRegister(hardware().miso_pin);
			*reg= hardware().miso_mux;
			reg = portConfigRegister(hardware().sck_pin);
			*reg = hardware().sck_mux;*/

		#elif defined(USING_IMXRT)

			//Deactivating the clock gate to be able to perform cahnges in the bus clock multiplexer without damaging it
			hardware().clock_gate_register &= ~hardware().clock_gate_mask;

			//Redirecting the bus clock multiplexer register to be used in the SPI comunication
			CCM_CBCMR = (CCM_CBCMR & ~(CCM_CBCMR_LPSPI_PODF_MASK | CCM_CBCMR_LPSPI_CLK_SEL_MASK)) |
			CCM_CBCMR_LPSPI_PODF(2) | CCM_CBCMR_LPSPI_CLK_SEL(1);

			#if defined(FASTIO)

				//Fastio definition
				uint32_t fastio = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
				//uint32_t fastio = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(1);
				//uint32_t fastio = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);

				//Configuring the pins to accept fastio
				*(portControlRegister(hardware().miso_pin)) = fastio;
				*(portControlRegister(hardware().mosi_pin)) = fastio;
				*(portControlRegister(hardware().sck_pin)) = fastio;
			#endif

			//Activating the clock gate to enable the SPI module
			hardware().clock_gate_register |= hardware().clock_gate_mask;

			//Linking the pins selected to the respective registers
			*(portConfigRegister(hardware().miso_pin)) = hardware().miso_mux;
			*(portConfigRegister(hardware().mosi_pin)) = hardware().mosi_mux;
			*(portConfigRegister(hardware().sck_pin)) = hardware().sck_mux;

			//Selecting the pins that are going to be used, normal or alternative ones (PJRC didn't put the alteranative ones)
			hardware().sck_select_input_register = hardware().sck_select_val;
			hardware().miso_select_input_register = hardware().miso_select_val;
			hardware().mosi_select_input_register = hardware().mosi_select_val;

			//Resetting the LSPI control register
			port().CR = LPSPI_CR_RST;

			//Initializing the FIFO control register with a TX watermarck of 15, at 15 words the recive data flag is enabled
			port().FCR = LPSPI_FCR_TXWATER(15);

			//Initializing the SPI to be in a known default state.
			beginTransaction();
			endTransaction();
		#endif
		return true;

	#elif defined(EXTERNAL_PROCESSOR)

		//Activating the clock gate to enable the SPI module
		hardware().clock_gate_register |= hardware().clock_gate_mask;

		//Disabling, halting and clearing the TX and RX FIFO 
		port().MCR = SPI_MCR_MDIS | SPI_MCR_HALT | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;

		//Configuring the ctar as the defined on the TeensySPISettings class
		port().CTAR0 = _moduleSettings->ctar;

		//Activating the DMAChannel interrupts
		port().RSER =  SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS | SPI_RSER_TFFF_RE | SPI_RSER_TFFF_DIRS;

		//Staring the SPI module as slave and the inactive state of CS is HIGH
		port().MCR = SPI_MCR_PCSIS(0x1F);

		//Declaring intermediate variable volatile because its going to transfer register stuff
		volatile uint32_t *reg;

		//Linking the pins selected to the respective registers
		reg = portConfigRegister(hardware().mosi_pin);
		*reg = hardware().mosi_mux;
		reg = portConfigRegister(hardware().miso_pin);
		*reg= hardware().miso_mux;
		reg = portConfigRegister(hardware().sck_pin);
		*reg = hardware().sck_mux;

		//Declaring the class
		_rx = new DMAChannel(false);

		//Now we initialize the RX DMAChannel, MAYBE BUGBUG
		_rx->begin(true);

		//Declaring the source of the DMAChannel
		_rx->source((volatile uint32_t &) port().POPR);

		//Determinig the trigger of the DMA activation
		_rx->triggerAtHardwareEvent(hardware().rx_dma_channel);

		//When the DMA Channel is triggered, a IRQ is created calling the specified function
		_rx->attachInterrupt(hardware().dma_rxisr);

		//Determining when the interrupt function is goig to be called
		_rx->interruptAtCompletion();

		//Defining the destination buffer where data goes and determining the number of words which after introducing that number of words in the buffer the interrupt is called
		_rx->destinationBuffer(TeensySPI::rx_buffer, RX_BUFFER_LENGHT + 1);

		//Enabling the DMAChannel
		_rx->enable();

		//Declaring the _tx class
		_tx = new DMAChannel(false);

		//Now we initialize the TX DMAChannel, MAYBE BUGBUG
		_tx->begin(true);

		//Declaring the source buffer of the DMAChannel
		_tx->sourceBuffer(TeensySPI::tx_buffer, TX_BUFFER_LENGHT + 1);

		//Determining the trigger of the DMA activation
		_tx->triggerAtHardwareEvent(hardware().tx_dma_channel);

		//No interrupt because what are you going to do with it

		//Defining the destination of the DMAChannel
		_tx->destination((volatile uint32_t &) port().PUSHR);

		//Enabling the DMAChannel
		_tx->enable();

		return true;

	#endif
	return false;
}

bool TeensySPI::end(){
	#if defined(MAIN_PROCESSOR)
		#if defined(USING_KINETIS)
		#elif defined(USING_IMXRT)

			// only do something if we have begun
			if (hardware().clock_gate_register & hardware().clock_gate_mask) {

				//Disabling the SPI module
				port().CR = 0;

				//Disabling the SPI com pins
				pinMode(hardware().miso_pin, INPUT_DISABLE);
				pinMode(hardware().mosi_pin, INPUT_DISABLE);
				pinMode(hardware().sck_pin, INPUT_DISABLE);
			}
		#endif
		return true;

	#elif defined(EXTERNAL_PROCESSOR)

		//Only do something if we have begun
		if (hardware().clock_gate_register & hardware().clock_gate_mask) {

			//Declaring intermediate variable volatile because its going to transfer register stuff
			volatile uint32_t *reg;

			//Detaching the pins selected from the respective registers
			reg = portConfigRegister(hardware().mosi_pin);
			*reg = 0;
			reg = portConfigRegister(hardware().miso_pin);
			*reg = 0;
			reg = portConfigRegister(hardware().sck_pin);
			*reg = 0;

			//Deactivating the SPI module
			port().MCR = SPI_MCR_MDIS | SPI_MCR_HALT | SPI_MCR_PCSIS(0x1F);

			//Deactivating the RX DMAChannel
			_rx->disable();

			//Deactivating the TX DMAChannel
			_tx->disable();
		}
		return true;

	#endif
	return false;
}

#if defined(MAIN_PROCESSOR)
	bool TeensySPI::beginTransaction(){

		#if defined(USING_KINETIS) //Teensy 3.5
			return true;
		#elif defined(USING_IMXRT) //Teensy 4.1

			//If the settings of the clock have changed
			if (_moduleSettings->clock() != _clock) {

				//Saving the new settings
				_clock = _moduleSettings->clock();

				//Saving the bus clock multiplexer register, to extract information of it
				uint32_t cbcmr = CCM_CBCMR;

				//Calculating the khz at what the bus clock is performing
				uint32_t clkhz = clk_sel[(cbcmr >> 4) & 0x03] / (((cbcmr >> 26 ) & 0x07 ) + 1);
				
				//Calculating the real number that the bus clock have to be devided by to obain the desired clock speed
				uint32_t d, div;		
				d = _clock ? clkhz/_clock : clkhz;

				//Obtaining the possible number that the bus clock have to be divided by to obtain the desired clock speed
				if (d && clkhz/d > _clock) d++;
				if (d > 257) d= 257;  // max div
				if (d > 2) {
					div = d-2;
				} else {
					div =0;
				}
				
				//Obtaining the configuration for the clock configuration regster ready to set to the register
				_ccr = LPSPI_CCR_SCKDIV(div) | LPSPI_CCR_DBT(div/2) | LPSPI_CCR_PCSSCK(div/2);
			}

			//Shuting down the LSPI control system
			port().CR = 0;

			//Configuring the LSPI as a master and the instruction to sample the data on a delayed SCK edge
			port().CFGR1 = LPSPI_CFGR1_MASTER | LPSPI_CFGR1_SAMPLE;

			//Configuring the clock control register
			port().CCR = _ccr;

			//Configurating the transmit comand register, which configures the way data is captured and introduced onto the FIFO
			port().TCR = _moduleSettings->tcr;

			//Activating the LSPI control system
			port().CR = LPSPI_CR_MEN;

			return true;
		#endif
		return false;
	}

	bool TeensySPI::endTransaction(){
		#if defined(USING_KINETIS) //Teensy 3.5
			return true;
		#elif defined(USING_IMXRT) //Teensy 4.1
			return true;
		#endif
		return false;
	}

	bool TeensySPI::transfer(uint8_t *sendBuf, size_t byteCountTX, uint8_t *retBuf, size_t byteCountRX){

		//DO NOT USE COMPLEX TRANSFER
		#if defined(USING_KINETIS) //Teensy 3.5
			return true;
		#elif defined(USING_IMXRT) //Teensy 4.1
			if (byteCountTX == 0 && !usingChecksum) return 0;

			Serial.println("Sended");

		    //Reset the recive FIFO and make sure that the module is enabled Why not resetting also the TX FIFO?
		    port().CR = LPSPI_CR_RRF | LPSPI_CR_MEN;

		    if (usingChecksum) {
		    	TXchecksumCalculated = 0;
		    	RXchecksumRecived = 0;
		    	byteCountTX++;
		    }

		    //Send the sendBuf in eight bits packets
		    while (byteCountTX > 0) {

		    	//Saving the following byte to transfer
		    	if(usingChecksum && byteCountTX == 1){
		    		_byteToTransfer = TXchecksumCalculated;
		    	}else{
		    		_byteToTransfer = sendBuf? *sendBuf++ : _transferWriteFill;
		    	}

		    	//If the checksum is enabled, the next byte to transfer is added
		    	if(usingChecksum) addToChecksum(_byteToTransfer, &TXchecksumCalculated);

	    		//Push out the next word; 
				port().TDR = _byteToTransfer;

		    	byteCountTX--; // how many bytes left to output.

				//Make sure queue is not full before pushing next byte out, to be able to -> <- -> <-, instead of -> -> <- <-
				do {
					if ((port().RSR & LPSPI_RSR_RXEMPTY) == 0) {

						//Reading the next byte
						uint8_t b = port().RDR;
						if(usingChecksum && byteCountRX == 1){
							RXchecksumRecived = b;
						}else{
							if (retBuf) *retBuf++ = b;
						}
						byteCountRX--;
					}
				} while ((port().SR & LPSPI_SR_TDF) == 0); //Wait until all the requested transmit data is sended
			}

			//Now lets wait for all of the read bytes left to be returned...
			while (byteCountRX > 0) {

				//Check if the reciveing register has pending data to process
				if ((port().RSR & LPSPI_RSR_RXEMPTY) == 0) {

					//Reading the next byte
					uint8_t b = port().RDR;
					if(usingChecksum && byteCountRX == 1){
						RXchecksumRecived = b;
					}else{
						if (retBuf) *retBuf++ = b;
					}
					byteCountRX--;
				}
			}
			return true;
		#endif
		return false;
	}
#elif defined(EXTERNAL_PROCESSOR)
	void TeensySPI::processData(){

		//We assume that the data could be sent
		DSPI.byteTXCount++;

		//Detecting if the comunication is using checksum
		if(DSPI.usingChecksum){

			//Resetting the checksum
			DSPI.RXchecksumCalculated = 0;

			//Calculating the checksum with the new data recived, except the last word that is the checksum
			for(int i = 0; i < RX_BUFFER_LENGHT - 1; i++){
				DSPI.addToChecksum(DSPI.rx_buffer[i], &(DSPI.RXchecksumCalculated));
			}

			//Checking if the checksums are the same
			if(DSPI.RXchecksumCalculated == DSPI.RXchecksumRecived){

				//The information was correctly recived
				DSPI.byteRXCount++;

				//Setting the data ready flag to 1
				DSPI.dataReady = true;

			}else{

				//Errasing the rx buffer to avoid errors, incrementing the error counter and setting the data ready flag to 0
				memset(DSPI.rx_buffer, 0, sizeof(DSPI.rx_buffer));
				DSPI.RXchecksumErrors++;
				DSPI.dataReady = false;
			}
		}else{
			//The information was correctly recived
			DSPI.byteRXCount++;

			//Setting the data ready flag to 1
			DSPI.dataReady = true;
		}

		//The interrupt sequence is finished
		DSPI._rx->clearInterrupt();
	}

	//The data is stored to be sended when the master request it
	bool TeensySPI::restockData(uint8_t sendBuf[]){

		//Calculating the TX checksum
		for(int i = 0; i < TX_BUFFER_LENGHT - 1; i++){
			addToChecksum(sendBuf[i], &TXchecksumCalculated);
		}

		//Joining th buffer to send and the checksum
		*tx_buffer = *sendBuf;
		tx_buffer[TX_BUFFER_LENGHT - 1] = TXchecksumCalculated;
		return true;
	}
#endif