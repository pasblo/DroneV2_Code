#define MAIN_PROCESSOR

#include "Pinout/Pinout.h"
#include "Control/Init.h"
//#include "FPS_Controller.h"
//#include "Scheduler.h"
//#include "SerialTelemetry.h" // TO IMPLEMENT

extern "C" int main(void){

	Init::init();

	while (1) {
		digitalWrite(CS_BMP_PIN, HIGH);
		delay(100);
		digitalWrite(CS_BMP_PIN, LOW);
		delay(100);
		/*if(droneMode == FLIGHT_MODE){
			//Scheduler::scheduler();
		}else if(droneMode == CONFIG_MODE){ //Configurign the drone via the computer with the desktop app
			//TODO
		}
		if(directTelemetry == true){
			//serialTelemetry();
		}
		if(droneMode != WAITING){
			//controllFPS();
		}*/
	}
}
