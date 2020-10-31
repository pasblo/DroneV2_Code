#ifndef DRONE_SPI
#define DRONE_SPI

#include "core_pins.h"
#include "arduino.h"

#if defined(__MKL26Z64__) //TEENSY-LC

	//SPI 0
	#if defined(USE_SPI0)
		#define SCK 13 //ALT_SCK -> 14
		#define MOSI 11 //ALT_MOSI -> 7
		#define MISO 12 //ALT_MISO -> 8
	
	//SPI 1
	#elif defined(USE_SPI1)
		#define SCK 20
		#define MOSI 0 //ALT_MOSI -> 21
		#define MISO 1 //ALT_MISO -> 5

	#endif
	
	//Chip select, really only used properly in slave mode, becuase in master mode the cs pin does not need to be one of this pins
	#define CS0 10
	#define CS1 6
	
	#define USING_KINETIS //When using Teensy 3.x

#elif defined(__MK20DX128__) //TEENSY 3.0
	
	//SPI 0
	#if defined(USE_SPI0)
		#define SCK 13 //ALT_SCK -> 14
		#define MOSI 11 //ALT_MOSI -> 7
		#define MISO 12 //ALT_MISO -> 8
	#endif
	
	//Chip select
	#define CS0 10 //ALT_CS0 -> 2
	#define CS1 9 //ALT_CS1 -> 6
	#define CS2 20 //ALT_CS2 -> 23
	#define CS3 21 //ALT_CS3 -> 22
	#define CS4 15
	
	#define USING_KINETIS //When using Teensy 3.x

#elif defined(__MK20DX256__) //TENSY 3.1 & 3.2
	
	//SPI 0
	#if defined(USE_SPI0)
		#define SCK 13 //ALT_SCK -> 14
		#define MOSI 11 //ALT_MOSI -> 7
		#define MISO 12 //ALT_MISO -> 8
	#endif

	//Chip select
	#define CS0 10 //ALT_CS0 -> 9, 15, 20 & 21
	
	#define USING_KINETIS //When using Teensy 3.x

#elif defined(__MK64FX512__) | defined(__MK66FX1M0__) //TEENSY 3.5 & 3.6

	//SPI 0
	#if defined(USE_SPI0)
		#define SCK 13 //ALT_SCK -> 14, 27
		#define MOSI 11 //ALT_MOSI -> 7, 28
		#define MISO 12 //ALT_MISO -> 8, 39
	
	//SPI 1
	#elif defined(USE_SPI1)
		#define SCK 32 //ALT_SCK -> 20
		#define MOSI 0 //ALT_MOSI -> 21
		#define MISO 1 //ALT_MISO -> 5
	
	//SPI 2
	#elif defined(USE_SPI2)
		#define SCK 46 //ALT_SCK -> 53
		#define MOSI 44 //ALT_MOSI -> 52
		#define MISO 45 //ALT_MISO -> 51

	#endif

	//Chip select
	#define CS0 10 //ALT_CS0 -> 9, 15, 20 & 21
	#define CS1 31
	#define CS2 43 //ALT_CS2 -> 54
	
	#define USING_KINETIS //When using Teensy 3.x

#elif defined(__IMXRT1052__) | defined(__IMXRT1062__)
	#if defined(ARDUINO_TEENSY41) //TEENSY 4.1

		//SPI 0
		#if defined(USE_SPI0)
			#define SCK 13
			#define MOSI 11
			#define MISO 12
		
		//SPI 1
		#elif defined(USE_SPI1)
			#define SCK 27
			#define MOSI 26
			#define MISO 39 //ALT_MISO -> 1

		#endif

		//Chip select
		#define CS0 10 //ALT_CS0 -> 36 & 37
		#define CS1 38 //ALT_CS1 -> 0

	#else //TEENSY 4.0

		//SPI 0
		#if defined(USE_SPI0)
			#define SCK 13
			#define MOSI 11
			#define MISO 12
		
		//SPI 1
		#elif defined(USE_SPI1)
			#define SCK 27
			#define MOSI 26
			#define MISO NULL //Why this pin does not exixt?
		
		//SPI 2
		#elif defined(USE_SPI2)
			#define SCK 37
			#define MOSI 35
			#define MISO 34

		#endif

		//Chip select
		#define CS0 10
		#define CS2 36 //Why there is not CS1?

	#endif
	
	#define USING_IMXRT //When using Teensy 4.x

#endif

#define SPI_SR_TXCTR 0x0000f000

#define SPI_MCR_MASTER 0x80000000
#define SPI_MCR_SLAVE 0x00000000

// SPI_MASTER / SPI_SLAVE -> Selects whether the code is going to be used as a slave SPI device or a master SPI device
// USE_SPI0 / USE_SPI1 / USE_SPI2 -> Selects what SPI port is going to be used in the teensy, note that not all teensys have all the SPI ports available
// USE_SPI0_CLK / USE_SPI1_CLK -> Selects what SCK pin is going to be used in the SPI transmision

#define SPI_WRITE_8(data, CTARn, csPin)
	do{
		//Waiting until ??
		SPI_WRITE_WAIT();

		//Inserting in the correspondant register (SPIn_PUSHR) the data to send
		#if defined(SPI_MASTER)
			#if defined(USE_SPI0)
				SPI0_PUSHR = ((c) & 0xff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#elif defined(USE_SPI1)
				SPI1_PUSHR = ((c) & 0xff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#elif defined(USE_SPI2)
				SPI2_PUSHR = ((c) & 0xff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#endif
		#elif defined(SPI_SLAVE)
			SPI0_PUSHR_SLAVE = ((c) & 0xff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#if defined(USE_SPI0)
				SPI0_PUSHR_SLAVE = ((c) & 0xff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#elif defined(USE_SPI1)
				SPI1_PUSHR_SLAVE = ((c) & 0xff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#elif defined(USE_SPI2)
				SPI2_PUSHR_SLAVE = ((c) & 0xff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#endif
		#endif
	} while(0)

#define SPI_WRITE_16(data, CTARn, csPin)
	do{

		//Waiting until ??
		SPI_WRITE_WAIT();

		//Inserting in the correspondant register (SPIn_PUSHR) the data to send
		#if defined(SPI_MASTER)
			#if defined(USE_SPI0)
				SPI0_PUSHR = ((c) & 0xffff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#elif defined(USE_SPI1)
				SPI1_PUSHR = ((c) & 0xffff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#elif defined(USE_SPI2)
				SPI2_PUSHR = ((c) & 0xffff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#endif
		#elif defined(SPI_SLAVE)
			SPI0_PUSHR_SLAVE = ((c) & 0xffff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#if defined(USE_SPI0)
				SPI0_PUSHR_SLAVE = ((c) & 0xffff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#elif defined(USE_SPI1)
				SPI1_PUSHR_SLAVE = ((c) & 0xffff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#elif defined(USE_SPI2)
				SPI2_PUSHR_SLAVE = ((c) & 0xffff) | SPI_PUSHR_CTAS(CTARn) | SPI_PUSHR_PCS(0x1f & csPin);
			#endif
		#endif
	} while(0)

#define SPI_WRITE_WAIT()

	#if defined(USE_SPI0)
		while((SPI0_SR & SPI_SR_TXCTR) >= 0x00004000);
	#elif defined(USE_SPI1)
		while((SPI1_SR & SPI_SR_TXCTR) >= 0x00004000);
	#elif defined(USE_SPI2)
		while((SPI2_SR & SPI_SR_TXCTR) >= 0x00004000);
	#endif

#define SPI_WAIT()
	#if defined(USE_SPI0)
		while ((SPI0_SR & SPI_SR_TXCTR) != 0);
		while (!(SPI0_SR & SPI_SR_TCF));
		SPI0_SR |= SPI_SR_TCF;
	#elif defined(USE_SPI1)
		while ((SPI1_SR & SPI_SR_TXCTR) != 0);
		while (!(SPI1_SR & SPI_SR_TCF));
		SPI1_SR |= SPI_SR_TCF;
	#elif defined(USE_SPI2)
		while ((SPI2_SR & SPI_SR_TXCTR) != 0);
		while (!(SPI2_SR & SPI_SR_TCF));
		SPI2_SR |= SPI_SR_TCF;
	#endif

/*
Supported teensys:
- (__MKL26Z64__) Teensy LC
- (__MK20DX128__) Teensy 3.0
- (__MK20DX256__) Teensy 3.1 & 3.2
- (__MK64FX512__) Teensy 3.5
- (__MK66FX1M0__) Teensy 3.6
- (__IMXRT1052__) Beta Teensy 4.0 & 4.1
- (__IMXRT1062__) Teensy 4.0 & 4.1
*/

class DroneSPISettings{
	public:
		#if defined(__MKL26Z64__)
			SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
				if (__builtin_constant_p(clock)) {
					init_AlwaysInline(clock, bitOrder, dataMode);
				} else {
					init_MightInline(clock, bitOrder, dataMode);
				}
			}
			SPISettings() {
				init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0);
			}
		#elif defined(__MK20DX128__) | defined(__MK20DX256__) | defined(__MK64FX512__) | defined(__MK66FX1M0__)
			SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode){
				if (__builtin_constant_p(clock)) {
					init_AlwaysInline(clock, bitOrder, dataMode);
				} else {
					init_MightInline(clock, bitOrder, dataMode);
				}
			}
			SPISettings() {
				init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0);
			}
		#elif defined(__IMXRT1052__) | defined(__IMXRT1062__)
			SPISettings(uint32_t clockIn, uint8_t bitOrderIn, uint8_t dataModeIn) : _clock(clockIn) {
				init_AlwaysInline(bitOrderIn, dataModeIn);
			}

			SPISettings() : _clock(4000000) {
				init_AlwaysInline(MSBFIRST, SPI_MODE0);
			}
		#endif
	private:
		#if defined(__MKL26Z64__)
			void init_MightInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
				init_AlwaysInline(clock, bitOrder, dataMode);
			}
			void init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {
				uint8_t c = SPI_C1_MSTR | SPI_C1_SPE;
				if (dataMode & 0x04) c |= SPI_C1_CPHA;
				if (dataMode & 0x08) c |= SPI_C1_CPOL;
				if (bitOrder == LSBFIRST) c |= SPI_C1_LSBFE;
				c1 = c;
				if (__builtin_constant_p(clock)) {
					if (clock >= F_BUS /   2) { c = SPI_BR_SPPR(0) | SPI_BR_SPR(0); }
					else if (clock >= F_BUS /   4) { c = SPI_BR_SPPR(1) | SPI_BR_SPR(0); }
					else if (clock >= F_BUS /   6) { c = SPI_BR_SPPR(2) | SPI_BR_SPR(0); }
					else if (clock >= F_BUS /   8) { c = SPI_BR_SPPR(3) | SPI_BR_SPR(0); }
					else if (clock >= F_BUS /  10) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(0); }
					else if (clock >= F_BUS /  12) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(0); }
					else if (clock >= F_BUS /  14) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(0); }
					else if (clock >= F_BUS /  16) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(0); }
					else if (clock >= F_BUS /  20) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(1); }
					else if (clock >= F_BUS /  24) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(1); }
					else if (clock >= F_BUS /  28) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(1); }
					else if (clock >= F_BUS /  32) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(1); }
					else if (clock >= F_BUS /  40) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(2); }
					else if (clock >= F_BUS /  48) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(2); }
					else if (clock >= F_BUS /  56) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(2); }
					else if (clock >= F_BUS /  64) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(2); }
					else if (clock >= F_BUS /  80) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(3); }
					else if (clock >= F_BUS /  96) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(3); }
					else if (clock >= F_BUS / 112) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(3); }
					else if (clock >= F_BUS / 128) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(3); } 
					else if (clock >= F_BUS / 160) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(4); }
					else if (clock >= F_BUS / 192) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(4); }
					else if (clock >= F_BUS / 224) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(4); }
					else if (clock >= F_BUS / 256) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(4); }
					else if (clock >= F_BUS / 320) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(5); }
					else if (clock >= F_BUS / 384) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(5); }
					else if (clock >= F_BUS / 448) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(5); }
					else if (clock >= F_BUS / 512) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(5); }
					else if (clock >= F_BUS / 640) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(6); }
					else { c = SPI_BR_SPPR(5) | SPI_BR_SPR(6); }
				} else {
					for (uint32_t i=0; i<30; i++) {
						c = br_clock_table[i];
						if (clock >= F_BUS / br_div_table[i]) break;
					}
				}
				br[0] = c;
				if (__builtin_constant_p(clock)) {
					if (clock >= (F_PLL/2) /   2) { c = SPI_BR_SPPR(0) | SPI_BR_SPR(0); }
					else if (clock >= (F_PLL/2) /   4) { c = SPI_BR_SPPR(1) | SPI_BR_SPR(0); }
					else if (clock >= (F_PLL/2) /   6) { c = SPI_BR_SPPR(2) | SPI_BR_SPR(0); }
					else if (clock >= (F_PLL/2) /   8) { c = SPI_BR_SPPR(3) | SPI_BR_SPR(0); }
					else if (clock >= (F_PLL/2) /  10) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(0); }
					else if (clock >= (F_PLL/2) /  12) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(0); }
					else if (clock >= (F_PLL/2) /  14) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(0); }
					else if (clock >= (F_PLL/2) /  16) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(0); }
					else if (clock >= (F_PLL/2) /  20) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(1); }
					else if (clock >= (F_PLL/2) /  24) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(1); }
					else if (clock >= (F_PLL/2) /  28) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(1); }
					else if (clock >= (F_PLL/2) /  32) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(1); }
					else if (clock >= (F_PLL/2) /  40) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(2); }
					else if (clock >= (F_PLL/2) /  48) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(2); }
					else if (clock >= (F_PLL/2) /  56) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(2); }
					else if (clock >= (F_PLL/2) /  64) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(2); }
					else if (clock >= (F_PLL/2) /  80) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(3); }
					else if (clock >= (F_PLL/2) /  96) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(3); }
					else if (clock >= (F_PLL/2) / 112) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(3); }
					else if (clock >= (F_PLL/2) / 128) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(3); }
					else if (clock >= (F_PLL/2) / 160) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(4); }
					else if (clock >= (F_PLL/2) / 192) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(4); }
					else if (clock >= (F_PLL/2) / 224) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(4); }
					else if (clock >= (F_PLL/2) / 256) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(4); }
					else if (clock >= (F_PLL/2) / 320) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(5); }
					else if (clock >= (F_PLL/2) / 384) { c = SPI_BR_SPPR(5) | SPI_BR_SPR(5); }
					else if (clock >= (F_PLL/2) / 448) { c = SPI_BR_SPPR(6) | SPI_BR_SPR(5); }
					else if (clock >= (F_PLL/2) / 512) { c = SPI_BR_SPPR(7) | SPI_BR_SPR(5); }
					else if (clock >= (F_PLL/2) / 640) { c = SPI_BR_SPPR(4) | SPI_BR_SPR(6); }
					else { c = SPI_BR_SPPR(5) | SPI_BR_SPR(6); }
				} else {
					for (uint32_t i=0; i<30; i++) {
						c = br_clock_table[i];
						if (clock >= (F_PLL/2) / br_div_table[i]) break;
					}
				}
				br[1] = c;
			}
			static const uint8_t  br_clock_table[30];
			static const uint16_t br_div_table[30];
			uint8_t c1, br[2];

		#elif defined(__MK20DX128__) | defined(__MK20DX256__) | defined(__MK64FX512__) | defined(__MK66FX1M0__)

			void init_MightInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
				init_AlwaysInline(clock, bitOrder, dataMode);
			}

			void init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {
				uint32_t t, c = SPI_CTAR_FMSZ(7);
				if (bitOrder == LSBFIRST) c |= SPI_CTAR_LSBFE;
				if (__builtin_constant_p(clock)) { //Maybe delete this option??
					if (clock >= F_BUS / 2) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0); }
					else if (clock >= F_BUS / 3) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0); }
					else if (clock >= F_BUS / 4) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0); } 
					else if (clock >= F_BUS / 5) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0); }
					else if (clock >= F_BUS / 6) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0); }
					else if (clock >= F_BUS / 8) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1); }
					else if (clock >= F_BUS / 10) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0); }
					else if (clock >= F_BUS / 12) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1); }
					else if (clock >= F_BUS / 16) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2); }
					else if (clock >= F_BUS / 20) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(0); }
					else if (clock >= F_BUS / 24) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2); }
					else if (clock >= F_BUS / 32) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(4) | SPI_CTAR_CSSCK(3); }
					else if (clock >= F_BUS / 40) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2); }
					else if (clock >= F_BUS / 56) { t = SPI_CTAR_PBR(3) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2); }
					else if (clock >= F_BUS / 64) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(5) | SPI_CTAR_CSSCK(4); }
					else if (clock >= F_BUS / 96) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(5) | SPI_CTAR_CSSCK(4); }
					else if (clock >= F_BUS / 128) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(6) | SPI_CTAR_CSSCK(5); }
					else if (clock >= F_BUS / 192) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(6) | SPI_CTAR_CSSCK(5); }
					else if (clock >= F_BUS / 256) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6); }
					else if (clock >= F_BUS / 384) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6); }
					else if (clock >= F_BUS / 512) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(8) | SPI_CTAR_CSSCK(7); }
					else if (clock >= F_BUS / 640) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6); }
					else { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(8) | SPI_CTAR_CSSCK(7); }
				} else {
					for (uint32_t i=0; i<23; i++) {
						t = ctar_clock_table[i];
						if (clock >= F_BUS / ctar_div_table[i]) break;
					}
				}
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

		#elif defined(__IMXRT1052__) | defined(__IMXRT1062__)
			void init_AlwaysInline(uint8_t bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {
				
				//8 Bit SPI by default
				tcr = LPSPI_TCR_FRAMESZ(7);

				// handle LSB setup 
				if (bitOrder == LSBFIRST) tcr |= LPSPI_TCR_LSBF;

				// Handle Data Mode
				if (dataMode & 0x08) tcr |= LPSPI_TCR_CPOL;

				// Note: On T3.2 when we set CPHA it also updated the timing.  It moved the 
				// PCS to SCK Delay Prescaler into the After SCK Delay Prescaler	
				if (dataMode & 0x04) tcr |= LPSPI_TCR_CPHA; 
			}

			inline uint32_t clock() {return _clock;}

			uint32_t _clock;
			uint32_t tcr;
		#endif

		friend class DroneSPI;
};

class DroneSPI{
	public:

		DroneSPI();

		void begin(bool CTARn, bool csActiveState);

	private:

		//Information variables
		#if defined(USE_64BIT_INFORMATION)
			//Maximun time of use, at maximun frequency 48MHz, on 8bit mode, constantly transmitting it will last for 97490 years roughly
			volatile uint64_t packetTXCount;
			volatile uint64_t packetRXCount;
		#else
			//Maximun time of use, at maximun frequency 48MHz, on 8bit mode, constantly transmitting it will last for 12min roughly
			volatile uint32_t packetTXCount;
			volatile uint32_t packetRXCount;
		#endif

		//Setting pins
		void setSCK();
		void setMISO();
		void setMOSI();
		void setCS(bool csActiveState, uint8_t csNumber);

		//Global functions
		void startSPI();
		void stopSPI();

		#if defined(SPI_SLAVE)

		#endif
};
extern DroneSPI DSPI;
#endif