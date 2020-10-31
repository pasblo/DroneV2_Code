#ifndef DRONE_SPI
#define DRONE_SPI

//Including general libraries
#include "core_pins.h"
#include "arduino.h"
#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Pinout.h> // TODO, sustitude with correct direction when using main
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/definitions.h" // TODO, sustitude with correct direction when using main


//Defining general macros
#define FASTIO

#define DEFAULT_SPEED 1000000

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

//Including specific libraries and defining specific macros
#if defined(EXTERNAL_PROCESSOR)
	#include "DMAChannel.h"

	#define RX_BUFFER_LENGHT 1
	#define TX_BUFFER_LENGHT 1
#endif

/*
Supported teensys:
- (__MK20DX256__) Teensy 3.2
- (__MK64FX512__) Teensy 3.5
- (__IMXRT1062__) Teensy 4.0 & 4.1
*/

//Different bus clock selections that can be active
static const uint32_t clk_sel[4] = {664615384, // PLL3 PFD1
									720000000, // PLL3 PFD0
									528000000, // PLL2
									396000000}; // PLL2 PFD2		

//SPI Settings class, to adjust the functionality of the SPI class
class DroneSPISettings{
	public:
		#if defined(MAIN_PROCESSOR)
			#if defined(USING_KINETIS)
				DroneSPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode){
					if (__builtin_constant_p(clock)) {
						init_AlwaysInline(clock, bitOrder, dataMode);
					} else {
						init_MightInline(clock, bitOrder, dataMode);
					}
				}
				DroneSPISettings() {
					init_AlwaysInline(DEFAULT_SPEED, MSBFIRST, SPI_MODE0);
				}

			#elif defined(USING_IMXRT)
				DroneSPISettings(uint32_t clockIn, uint8_t bitOrderIn, uint8_t dataModeIn) : _clock(clockIn) {
					init_AlwaysInline(bitOrderIn, dataModeIn);
				}

				DroneSPISettings() : _clock(DEFAULT_SPEED) {
					init_AlwaysInline(MSBFIRST, SPI_MODE0);
				}

			#endif
		#elif defined(EXTERNAL_PROCESSOR) & defined(USING_KINETIS)
			DroneSPISettings(uint8_t bitOrderIn, uint8_t dataModeIn) {
				init_AlwaysInline(bitOrderIn, dataModeIn);
			}

			DroneSPISettings() {
				init_AlwaysInline(MSBFIRST, SPI_MODE0);
			}

		#elif defined(ESP_PROCESSOR)
		#endif
	private:
		#if defined(MAIN_PROCESSOR)
			#if defined(USING_KINETIS)
				void init_MightInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
					init_AlwaysInline(clock, bitOrder, dataMode);
				}

				void init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {
					
					//8 Bit SPI by default
					uint32_t t, c = SPI_CTAR_FMSZ(7);

					// Handle LSB setup 
					if (bitOrder == LSBFIRST) c |= SPI_CTAR_LSBFE;

					//Configurating the clock spceed
					if (__builtin_constant_p(clock)) {
						if (clock >= F_BUS / 2) t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0);
						else if (clock >= F_BUS / 3) t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0);
						else if (clock >= F_BUS / 4) t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0);
						else if (clock >= F_BUS / 5) t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0);
						else if (clock >= F_BUS / 6) t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0);
						else if (clock >= F_BUS / 8) t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1);
						else if (clock >= F_BUS / 10) t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0);
						else if (clock >= F_BUS / 12) t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1);
						else if (clock >= F_BUS / 16) t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
						else if (clock >= F_BUS / 20) t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(0);
						else if (clock >= F_BUS / 24) t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
						else if (clock >= F_BUS / 32) t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(4) | SPI_CTAR_CSSCK(3);
						else if (clock >= F_BUS / 40) t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
						else if (clock >= F_BUS / 56) t = SPI_CTAR_PBR(3) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
						else if (clock >= F_BUS / 64) t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(5) | SPI_CTAR_CSSCK(4);
						else if (clock >= F_BUS / 96) t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(5) | SPI_CTAR_CSSCK(4);
						else if (clock >= F_BUS / 128) t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(6) | SPI_CTAR_CSSCK(5);
						else if (clock >= F_BUS / 192) t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(6) | SPI_CTAR_CSSCK(5);
						else if (clock >= F_BUS / 256) t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6);
						else if (clock >= F_BUS / 384) t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6);
						else if (clock >= F_BUS / 512) t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(8) | SPI_CTAR_CSSCK(7);
						else if (clock >= F_BUS / 640) t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6);
						else t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(8) | SPI_CTAR_CSSCK(7);
					} else {
						for (uint32_t i=0; i<23; i++) {
							t = ctar_clock_table[i];
							if (clock >= F_BUS / ctar_div_table[i]) break;
						}
					}

					// Handle Data Mode
					if (dataMode & 0x08) {
						c |= SPI_CTAR_CPOL;
					}
					if (dataMode & 0x04) {
						c |= SPI_CTAR_CPHA;
						t = (t & 0xFFFF0FFF) | ((t & 0xF000) >> 4);
					}
					ctar = c | t;
				}

				static const uint16_t ctar_div_table[23];
				static const uint32_t ctar_clock_table[23];
				uint32_t ctar;

			#elif defined(USING_IMXRT)
				void init_AlwaysInline(uint8_t bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {
					
					//8 Bit SPI by default
					tcr = LPSPI_TCR_FRAMESZ(7);

					// Handle LSB setup 
					if (bitOrder == LSBFIRST) tcr |= LPSPI_TCR_LSBF;

					// Handle Data Mode
					if (dataMode & 0x08) tcr |= LPSPI_TCR_CPOL; //SCK active value high or low
					if (dataMode & 0x04) tcr |= LPSPI_TCR_CPHA; //SCK data capturation, falling or rising
				}

				inline uint32_t clock() {return _clock;}
				uint32_t _clock;
				uint32_t tcr;

			#endif
		#elif defined(EXTERNAL_PROCESSOR)
			void init_MightInline(uint8_t bitOrder, uint8_t dataMode) {
				init_AlwaysInline(bitOrder, dataMode);
			}

			void init_AlwaysInline(uint8_t bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {

				//8 Bit SPI by default
				ctar = SPI_CTAR_FMSZ(7);

				// Handle LSB setup 
				if (bitOrder == LSBFIRST) ctar |= SPI_CTAR_LSBFE;

				// Handle Data Mode
				if (dataMode & 0x08) ctar |= SPI_CTAR_CPOL; //SCK active value high or low
				if (dataMode & 0x04) ctar |= SPI_CTAR_CPHA; //SCK data capturation, falling or rising
			}

			uint32_t ctar;

		#elif defined(ESP_PROCESSOR)
		#endif

		//They are just friends
		friend class DroneSPI;
};

//SPI main class
class DroneSPI{
	public:

		#if defined(USING_KINETIS)
			typedef struct {

				//Clock register
				volatile uint32_t &clock_gate_register;
				uint32_t clock_gate_mask;

				//Other variables
				uint8_t queue_size;
				uint8_t spi_irq;

				//DMA channel configurations
				uint32_t max_dma_count;
				uint8_t tx_dma_channel;
				uint8_t rx_dma_channel;
				void (*dma_rxisr)();

				//MISO pin
				uint8_t miso_pin;
				uint32_t miso_mux;

				//MOSI pin
				uint8_t mosi_pin;
				uint32_t mosi_mux;

				//SCK pin
				uint8_t sck_pin;
				uint32_t sck_mux;

				//CS pin
				uint8_t cs_pin;
				uint32_t cs_mux;
				uint8_t cs_mask;
			} SPI_Hardware_t;

		#elif defined(USING_IMXRT)
			typedef struct {

				//Clock register
				volatile uint32_t &clock_gate_register;
				const uint32_t clock_gate_mask;

				//DMA channel
				uint8_t tx_dma_channel;
				uint8_t rx_dma_channel;
				void (*dma_rxisr)();

				// MISO pin
				const uint8_t miso_pin;
				const uint32_t miso_mux;
				const uint8_t miso_select_val;
				volatile uint32_t &miso_select_input_register;

				// MOSI pin
				const uint8_t mosi_pin;
				const uint32_t mosi_mux;
				const uint8_t mosi_select_val;
				volatile uint32_t &mosi_select_input_register;

				// SCK pin
				const uint8_t sck_pin;
				const uint32_t sck_mux;
				const uint8_t sck_select_val;
				volatile uint32_t &sck_select_input_register;

				// CS Pin
				const uint8_t cs_pin;
				const uint32_t cs_mux;
				const uint8_t cs_mask;
				const uint8_t pcs_select_val;
				volatile uint32_t *pcs_select_input_register;

			} SPI_Hardware_t;

		#endif
		static const SPI_Hardware_t spi_hardware;

		//Constructor
		constexpr DroneSPI(uintptr_t myport, uintptr_t myhardware) 
			: port_addr(myport), hardware_addr(myhardware) {

		}

		//Begin and end the SPI module
		bool begin();
		bool end();

		//Introduce the settings that are going to be used on this SPI module
		void configureSPIsettings(DroneSPISettings *settings) {_moduleSettings = settings; }

		//Configures if the comunications uses checksum
		void setUseChecksum(bool useChecksum) {usingChecksum = useChecksum; }

		//Returns if the module is using the complement of using a checksum
		bool isUsingChecksum() {return usingChecksum; }

		//Specific functions dependig on the mode of the SPI module
		#if defined(MAIN_PROCESSOR)

			//Begin and end transaction
			bool beginTransaction();
			bool endTransaction();

			//Transfer data and recive
			bool transfer(uint8_t *sendBuf, size_t bitCountTX, uint8_t *retBuf, size_t bitCountRX);

		#elif defined(EXTERNAL_PROCESSOR)

			//Check if there is new data, and sets the retBuff to the new data if available, expected to restock the data to send
			bool dataAvailable(uint8_t retBuf[]){
				if(dataReady){
					*retBuf = *rx_buffer; //Maybe BUGBUG
					dataReady = false;
					return true;
				}
				return false;
			};

			//Refill the data buffers to send data in the next transmit pulse
			bool restockData(uint8_t sendBuf[]);

			//Process the data being recived
			static void processData();

			//Store the data being recived
			uint8_t rx_buffer[RX_BUFFER_LENGHT] = {0};

			//Store the data being sent
			uint8_t tx_buffer[TX_BUFFER_LENGHT] = {0};

		#elif defined(ESP_PROCESSOR)
		#endif

	private:

		//Settings of the SPI module
		DroneSPISettings *_moduleSettings = nullptr;

		//port() & hardware() functions definition, used to acces direct register
		#if defined(USING_KINETIS)
			KINETISK_SPI_t & port() { return *(KINETISK_SPI_t *)port_addr; }
		#elif defined(USING_IMXRT)
			IMXRT_LPSPI_t & port() { return *(IMXRT_LPSPI_t *)port_addr; }
		#endif

		const SPI_Hardware_t & hardware() { return *(const SPI_Hardware_t *)hardware_addr; }
		uintptr_t port_addr;
		uintptr_t hardware_addr;

		#if defined(MAIN_PROCESSOR)

			//Clock related stuff
			uint32_t _clock = 0;
			uint32_t _ccr = 0;

			//Word that is going to be send if there is no more in the buffer but the desired length is longer
			uint8_t _transferWriteFill = 6;

			uint8_t _byteToTransfer = _transferWriteFill;

		#elif defined(EXTERNAL_PROCESSOR)

			//Initializing the RX DMAChannel to control incoming data from the SPI bus without allocating it
			DMAChannel *_rx = nullptr;

			//Initializing the TX DMAChannel to control outgoing data to the SPI bus without allocating it
			DMAChannel *_tx = nullptr;

			//This variable is set true if there is new data recived
			bool dataReady = false;

		#elif defined(ESP_PROCESSOR)
		#endif

		//RX Checksum related variables
		uint8_t RXchecksumRecived = 0;
		uint8_t RXchecksumCalculated = 0;

		//TX Checksum related variables
		uint8_t TXchecksumCalculated = 0;

		//Checksum function
		void addToChecksum(uint8_t value, uint8_t *checksum){
			uint8_t parityValue = value;
			uint8_t numberOfOnes = 0;
			for(int i = 0; i < 8; i++){
				if(parityValue & 1) numberOfOnes++;

				parityValue >>= 1;
		    }
		    if(numberOfOnes > 4) *checksum = *checksum + 1;
			//*checksum = *checksum + value; //By the moment is as simple as a sum, but it can get more complicated
		};

		//Variable to determine if the comunication is using the checksum protocol
		bool usingChecksum = false;

		//Information variables
		#if defined(USE_64BIT_INFORMATION)
			//Maximun time of use, at maximun frequency 48MHz, constantly transmitting it will last for 97490 years roughly
			volatile uint64_t byteTXCount = 0;
			volatile uint64_t byteRXCount = 0;
			uint64_t RXchecksumErrors = 0;
		#else
			//Maximun time of use, at maximun frequency 48MHz, constantly transmitting it will last for 12min roughly
			volatile uint32_t byteTXCount = 0;
			volatile uint32_t byteRXCount = 0;
			uint32_t RXchecksumErrors = 0;
		#endif

		//Setting pins
		void setSCK();
		void setMISO();
		void setMOSI();
		void setCS(bool csActiveState, uint8_t csNumber);

		//Global functions
		void startSPI();
		void stopSPI();
};
extern DroneSPI DSPI;
#endif