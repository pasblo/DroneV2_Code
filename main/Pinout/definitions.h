/*
 * This file is only intended for hardware level variabales definitions,
 * aka variables that depends on the hardware
*/

#if !defined(MAIN_PROCESSOR) && !defined(EXTERNAL_PROCESSOR) && !defined(ESP_PROCESSOR)
	#error "A processor where this program is going to run is needed to be defined"
#endif

#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Config.h"

#if defined(__MK20DX256__) || defined(__MK64FX512__)
	#define USING_KINETIS
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)
	#define USING_IMXRT
#endif

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

//BMP388 general definitions
#define SEA_PRESSURE 102000.0 //Default sea pressure if no calibrations are performed
#define CALIBRATION_CYCLES 1000 //Number of cycles to be performed when calibrating the BMP388 sensor
#define AUTO_CALIBRATION_CYCLES 200 //Number of cycles to be performed before any measurement of the barometer to discard first measurements

#if defined(MAIN_PROCESSOR)

	//SPI comunications
	#define DEFAULT_SCK_FREQUENCY (1000000) //In Hz
	#define MAXIMUN_BMP388_SCK_FREQUENCY (10000000) //In Hz
	#define MAXIMUN_ICM20948_SCK_FREQUENCY (7000000) //In Hz

	//Flash definitions
	#define BYTE_WRITE_TIMEOUT (10000) //In microseconds

#elif defined(EXTERNAL_PROCESSOR)

	//BMP 388 pressure and temperature sensor
	#define BMP388_ADDRESS (0x76)
	
#elif defined(ESP_PROCESSOR)
#endif