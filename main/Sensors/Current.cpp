#include "Current.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Processor.h" // TODO, sustitude with correct direction when using main
//#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Processor.cpp" // TODO, sustitude with correct direction when using main
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/SerialHelper.h"

DroneCurrent DCurrent;

void DroneCurrent::updateCurrentSensors(){
	double rawVolts = DProcessor.voltageRead(PCB_CURRENT_PIN);

	pcbCurrent = (rawVolts - PCBvref) * PCBsensitivity * 1000.0;
}