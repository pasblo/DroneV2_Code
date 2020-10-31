#ifndef PROCESSOR_H
#define PROCESSOR_H

#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Pinout.h" // TODO, sustitude with correct direction when using main
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/definitions.h" // TODO, sustitude with correct direction when using main

/*
 * Operations that this class supports:
 * -Modification in the processor frequency if the processor allows it.
 * -Configuration of the ADC modules. See: ADC0_CFG1, ADC0_CFG2, ADC0_SC1A, ADC0_SC2, ADC0_SC3, ADC0_SC1A and other register for configurate the ADC better
 * -Configuration of watchdog timer.
 * -Returns the different processor frequencys
*/

#define DEFAULT_VREF 3.3 //In volts

// Teensy 3.2
#if defined(__MK20DX256__)  
	#define VREF_PIN 	(39)

// Teensy 3.5
#elif defined(__MK64FX512__)
	#define VREF_PIN 	(71)

//Teensy 4.1
#elif defined(__IMXRT1062__)
	#define VREF_PIN	(NULL)
#endif

//ADC processor modes
#define STANDARD_MODE (0x01)
#define HIGH_RESOLUTION_MODE (0x02)
#define HIGH_PERFORMANCE_MODE (0x03)

typedef struct{
	uint8_t mode;
	uint8_t resolution; //In bits 
	uint8_t avgSamples; //In number of samples per calculation (1, 4, 8, 16, 32)
} ModeConfig_t;

class DroneProcessor{

	public:

		//Initializates the class
		bool begin();

		//Recalculates all the information that can be later obtained by getters
		bool updateProcessor();

		//Returns the frequency at witch the CPU running
		uint32_t getProcessorFrequency(){
			#if defined(USING_IMXRT)
				return F_CPU_ACTUAL;
			#elif defined(USING_KINETIS)
				return F_CPU;
			#endif
		}

		//Returns the frequency of the main bus, which is used by all peripherals along with their respective clock dividers
		uint32_t getBusFrequency(){
			#if defined(USING_IMXRT)
				return F_BUS_ACTUAL;
			#elif defined(USING_KINETIS)
				return F_BUS;
			#endif
		}

		#if defined(USING_IMXRT)

			//Changes the CPU frequency, this change can modify all the frequencies used in the processor
			//bool setProcessorFrequency(uint32_t newFrequency) {set_arm_clock(newFrequency); } //BUGBUG, set_arm_clock is not detected

		#elif defined(USING_KINETIS)

			bool saveADCConfiguration();
			bool loadADCConfiguration();

			//Returns the voltage of the theoretical 3.3V rail
			double getVrefVoltage() {return lastVrefVoltage; }

			//Returns the frquency running at the PLL section
			uint32_t getPllFrequency() {return F_PLL; }

			//Returns the frequency at witch the different memories are running
			uint32_t getMemFrequency() {return F_MEM; }
		#endif

		//Changes the mode of the ADC, if the mode requested is already in use it will return false
		bool setADCMode(uint8_t modeSelection){
			switch(modeSelection){
				case STANDARD_MODE:
					if(actualMode.mode != STANDARD_MODE){
						actualMode = standardMode;
						modeUpdatedAndCalibrated = false;
					}
					return true;
				case HIGH_RESOLUTION_MODE:
					if(actualMode.mode != HIGH_RESOLUTION_MODE){
						actualMode = resolutionMode;
						modeUpdatedAndCalibrated = false;
					}
					return true;
				case HIGH_PERFORMANCE_MODE:
					if(actualMode.mode != HIGH_PERFORMANCE_MODE){
						actualMode = performanceMode;
						modeUpdatedAndCalibrated = false;
					}
					return true;
				return false;
			}
		}
		double analogToVoltage(uint16_t analogReading){ return ((double) analogReading) * voltsPerBit; };
		double voltageRead(uint8_t pin);

		static const ModeConfig_t standardMode;
		static const ModeConfig_t resolutionMode;
		static const ModeConfig_t performanceMode;

	private:

		bool updateADCMode();

		#if defined(USING_IMXRT)
		#elif defined(USING_KINETIS)
			int previousADC0_CFG1;
			int previousADC0_CFG2;
			int previousADC0_SC1A;
			int previousADC0_SC2;
			int previousADC0_SC3;
			double lastVrefVoltage = DEFAULT_VREF;
		#endif

		double voltsPerBit;

		ModeConfig_t actualMode = standardMode;
		bool modeUpdatedAndCalibrated = false;
};
extern DroneProcessor DProcessor;
#endif