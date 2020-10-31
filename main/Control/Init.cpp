#include "Init.h"

#include "Pinout/Pinout.h"
#include "Scheduler.h"

void Init::init(){

	//Initializing the pins used in the 
	Pinout::init();

	//Initializing the base tasks of the program
	Scheduler::init();
}