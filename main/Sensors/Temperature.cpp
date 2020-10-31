#include "Temperature.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/Pinout.h" // TODO, sustitude with correct direction when using main
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Pinout/definitions.h" // TODO, sustitude with correct direction when using main
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Processor.h" // TODO, sustitude with correct direction when using main
//#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Processor.cpp" // TODO, sustitude with correct direction when using main

DroneTemperature DTemperature;

bool DroneTemperature::updateTemperatureSensors(){

	//Record the init time of the function
	lastExecutionTime = micros();

	DProcessor.updateProcessor(); //TODO, move to schedule

	//First we caculate the processor temperature
	#if defined(USING_IMXRT)

		//Calculating the temperature
		processorTemperature = tempmonGetTemp();

	#elif defined(USING_KINETIS)

		//Save the ADC0 status in memory to later recover it
		DProcessor.saveADCConfiguration();

		// enable long sample times, not necessary
		ADC0_CFG1 |= ADC_CFG1_ADLSMP;

		// change to 2 cycles reading time, not necesary
		ADC0_CFG2 &= ~ADC_CFG2_ADLSTS(3);

		// disable compare (prevents COCO), maybe necesary
		ADC0_SC2 &= ~ADC_SC2_ACFE;

		//Reading the temperature
		float rawVolts = DProcessor.voltageRead(TEMPERATURE_PIN);

		//Converting the voltage calculated to celsius
		processorTemperature = 25.0 - ((rawVolts - vAt25) / slope);

		//Restoring the ADC0 status from memory to the registers
		DProcessor.loadADCConfiguration();
			
	#endif

	//Second we calculate the pcb temperature
	#if defined(MAIN_PROCESSOR)
		pcbTemperature = DProcessor.voltageRead(TEMPERATURE_SENSOR_PIN);
	#elif defined(EXTERNAL_PROCESSOR)
		pcbTemperature = DProcessor.voltageRead(TEMPERATURE_SENSOR1_PIN);
	#endif
	pcbTemperature = pcbTemperature - 0.5;
	pcbTemperature = pcbTemperature / 0.01;

	lastExecutionTime = micros() - lastExecutionTime;
}