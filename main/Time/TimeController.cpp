#include <Arduino.h>
#include "TimeController.h"

void fpsController_t_setFpsTrigger(fpsController_t *controller, functionCall_t function, uint32_t fpsTrigger){
	controller->fpsErrorTrigger = fpsTrigger;
	controller->triggerCall = function;
}

void TimeController::initializeFPSController(fpsController_t *controller){

	controller->lastMicros = micros();

	controller->lastRecordedFPS = 0.0;
	controller->averageFPS = 0.0;
	controller->fpsReadings = 0;
	controller->lastExecutionTime = 0;

	controller->fpsErrorTrigger = 0;
	controller->triggerCall = NULL;
}

void TimeController::controlFunctions(){

	if(timedFunctions.size() > 0){
		isTimedFunctions = true;
	}else{
		isTimedFunctions = false;
	}

	if(alarmFunctions.size() > 0){
		isAlarmFunctions = true;
	}else{
		isAlarmFunctions = false;
	}

	if(isTimedFunctions){

		// Determining if each timed function needs to be executed on this loop
		for(unsigned int i = timedFunctions.size(); i > 0; i--){
			if(micros() - timedFunctions.at(i).lastCall >= 1000000.0/timedFunctions.at(i).frequency){
				uint64_t initExecutionTime = micros();
				bool endFunction = timedFunctions.at(i).function();
				timedFunctions.at(i).lastExecutionTime = micros() - initExecutionTime;
				timedFunctions.at(i).lastCall = initExecutionTime;
				/*if(endFunction){ TODO fix functionsToErraseBegin
					functionsToErrase.insert(functionsToErraseBegin, i);
				}*/
			}
		}

		// Returning the pointer to the begining
		//functionsToErraseBegin.begin();

		// Errasing each timed functions that requested to be erraed
		for(unsigned int i = 0; i < functionsToErrase.size(); i++){
			timedFunctions.errase(functionsToErrase.at(i));
		}

		// Errasing all the pointers to the functions to be errased, because they were already errased
		timedFunctions.clear();
	}

	/*if(isAlarmFunctions){ TODO

		// Determining if each alarm function must be called in this loop
		for(unsigned int i = 0; i < alarmFunctions.size(); i++){
			if(DroneTime.getActualTimeUS() >= alarmFunctions.at(i).)
		}
	}*/
}

void TimeController::controlFPS(fpsController_t *controller){

	// Calculating the fps of this execution
	uint64_t newMicros = micros();
	uint64_t microsDifference = newMicros - controller->lastMicros;
	controller->lastMicros = newMicros;
	controller->lastRecordedFPS = (1000000.0/microsDifference);

	// Calculating the average fps of all the readings
	controller->averageFPS = (controller->averageFPS*controller->fpsReadings) + controller->lastRecordedFPS;
	controller->fpsReadings++;
	controller->averageFPS = controller->averageFPS/controller->fpsReadings;

	// Checking if the fpsTrigger is activated
	if(controller->lastRecordedFPS < controller->fpsErrorTrigger){
		controller->triggerCall();
	}

	controller->lastExecutionTime = micros() - newMicros;
}

vector<timedFunction_t>::iterator TimeController::addTimedFunction(functionCall_t function, double callFreq, bool freqFirstCall){
	timedFunction_t newTimerFunction;
	newTimerFunction.function = *function;
	newTimerFunction.frequency = callFreq;

	// Determining if the first call is going to be after the period stablished by the frequency or in the first call to controlFPS
	if(freqFirstCall){
		newTimerFunction.lastCall = micros(); // After the period stablished
	}else{
		newTimerFunction.lastCall = 0; // Immediately
	}
	newTimerFunction.lastExecutionTime = 0;
	timedFunctionsBegin = timedFunctions.insert(timedFunctionsBegin, newTimerFunction);

	return timedFunctionsBegin;
}

vector<timedFunction_t>::iterator TimeController::addTimedFunction(functionCall_t function, double callFreq){
	return addTimedFunction(function, callFreq, false);
}

vector<timedFunction_t>::iterator TimeController::addAlarmFunction(functionCall_t function, droneDateTime_t callTime){
	alarmFunction_t newAlarmFunction;
	newAlarmFunction.function = function;
	newAlarmFunction.alarmTime = callTime;
	newAlarmFunction.executionTime = 0;

	alarmFunctionsBegin = alarmFunctions.insert(alarmFunctionsBegin, newAlarmFunction);

	return alarmFunctionsBegin;
}