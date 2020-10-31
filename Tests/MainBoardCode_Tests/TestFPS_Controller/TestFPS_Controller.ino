#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Time/TimeController.h>
#include <C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Time/TimeController.cpp>

TimeController timeController = TimeController();

fpsController_t controller;

void printFPS(){
  Serial.print("Average FPS: ");
  Serial.println(controller.averageFPS);
}

void func1(){
  Serial.print("Last FPS: ");
  Serial.println(controller.lastRecordedFPS);
}

void errorWithTheFPS(){
  Serial.println("There was a error in the fps");
}

void setup() {
  Serial.begin(115200);
  timeController.initializeFPSController(&controller);
  //fpsController_t_setFpsTrigger(&controller, &errorWithTheFPS, 100);
  timeController.addTimedFunction(&printFPS, 1.0, true);
  timeController.addTimedFunction(&func1, 1.0, true);

}

void loop() {
  timeController.controlFPS(&controller);
  if(Serial.available()){
    Serial.read();
    delay(1000);
  }
  
}
