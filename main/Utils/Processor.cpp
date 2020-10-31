#include "Processor.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Pinout.h" // TODO, sustitude with correct direction when using main
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/definitions.h" // TODO, sustitude with correct direction when using main
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/SerialHelper.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/SerialHelper.cpp"

// Teensy 3.2
#if defined(__MK20DX256__)  
	const ModeConfig_t DroneProcessor::standardMode = {STANDARD_MODE, 0, 0};
	const ModeConfig_t DroneProcessor::resolutionMode = {HIGH_RESOLUTION_MODE, 0, 0};
	const ModeConfig_t DroneProcessor::performanceMode = {0, 0};

// Teensy 3.5
#elif defined(__MK64FX512__)
	const ModeConfig_t DroneProcessor::standardMode = {STANDARD_MODE, 12, 8};
	const ModeConfig_t DroneProcessor::resolutionMode = {HIGH_RESOLUTION_MODE, 16, 16};
	const ModeConfig_t DroneProcessor::performanceMode = {HIGH_PERFORMANCE_MODE, 8, 4};

//Teensy 4.1
#elif defined(__IMXRT1062__)
	const ModeConfig_t DroneProcessor::standardMode = {STANDARD_MODE, 10, 8};
	const ModeConfig_t DroneProcessor::resolutionMode = {HIGH_RESOLUTION_MODE, 12, 16};
	const ModeConfig_t DroneProcessor::performanceMode = {HIGH_PERFORMANCE_MODE, 8, 4};

#endif

DroneProcessor DProcessor;

bool DroneProcessor::begin(){

	#if defined(USING_IMXRT)

	#elif defined(USING_KINETIS)
		//Configuring the reference of the analog to be internal
		analogReference(EXTERNAL);

	#endif
}

bool DroneProcessor::updateADCMode(){

	//Updating the resolution in bits if the ADC
	analogReadRes(actualMode.resolution);

	//Calculating the number of volts per bit of resolution
	voltsPerBit = 3.3 / pow(2, actualMode.resolution);
	DSerialHelper.printDouble(voltsPerBit, 8);

	//Updating the samples that are used to average the reading
	analogReadAveraging(actualMode.avgSamples);

	//The module is updated and calibrated
	modeUpdatedAndCalibrated = true;
}

#if defined(USING_IMXRT)
	bool DroneProcessor::updateProcessor(){

		//Updating the ADC mode if was changed sinde the last update on the processor
		if(!modeUpdatedAndCalibrated) updateADCMode();

		return true;

	}
#elif defined(USING_KINETIS)
	bool DroneProcessor::saveADCConfiguration(){
		previousADC0_CFG1 = ADC0_CFG1;
		previousADC0_CFG2 = ADC0_CFG2;
		previousADC0_SC1A = ADC0_SC1A;
		previousADC0_SC2  = ADC0_SC2;
		previousADC0_SC3  = ADC0_SC3;

	}
	bool DroneProcessor::loadADCConfiguration(){
		ADC0_CFG1 = previousADC0_CFG1;
		ADC0_CFG2 = previousADC0_CFG2;
		ADC0_SC1A = previousADC0_SC1A;
		ADC0_SC2  = previousADC0_SC2;
		ADC0_SC3  = previousADC0_SC3;
		ADC0_SC1A = previousADC0_SC1A;

	}
	bool DroneProcessor::updateProcessor(){

		//Updating the ADC mode if was changed sinde the last update on the processor
		if(!modeUpdatedAndCalibrated) updateADCMode();

		//Save the ADC0 status in memory to later recover it
		saveADCConfiguration();

		// disable compare (prevents the flag COCO to be activated)
		ADC0_SC2 &= ~ADC_SC2_ACFE;

		//Read refference voltage
		lastVrefVoltage = voltageRead(VREF_PIN); //TODO, sustitude by function that returns processor voltage

		//Restoring the ADC0 status from memory to the registers
		loadADCConfiguration();

		return true;
	}
#endif

double DroneProcessor::voltageRead(uint8_t pin){
	return ((double) analogRead(pin)) * voltsPerBit;
}