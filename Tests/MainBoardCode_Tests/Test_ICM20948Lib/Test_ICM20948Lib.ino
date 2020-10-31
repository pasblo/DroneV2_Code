#define MAIN_PROCESSOR

#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/ICM20948/ICM20948.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Sensors/ICM20948/ICM20948.cpp"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/SerialHelper.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/SerialHelper.cpp"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/ErrorCodes.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/ErrorCodes.cpp"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Math.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Math.cpp"
//#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/MahonyAHRS/MahonyAHRS.h"
//#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/MahonyAHRS/MahonyAHRS.c"
//#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/MadgwickAHRS/MadgwickAHRS.h"
//#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/MadgwickAHRS/MadgwickAHRS.c"

//Tests to perform
//#define TEST_FREQUENCY
//#define TEST_RANGE
//#define TEST_RAW
#define TEST_SCALED
//#define TEST_FILTERED
//#define TEST_RAW_GRAPH
//#define TEST_SCALED_GRAPH
//#define TEST_FILTERED_GRAPH
//#define TEST_RESET
//#define TEST_PDMP //Pseudo DMP
//#define TEST_DMP

//Sensors
//#define GYROSCOPE
//#define ACCELEROMETER
//#define TEMPERATURE
#define MAGNETOMETER

//Axis
#define AXIS_X
//#define AXIS_Y
//#define AXIS_Z

#define UPDATE_RATE 1000 //Frequency
#define RESET_RATE 100 //Frequency
#define DMP_RATE 512 //Frequency
#define SCALE_RATE 10000

#if !defined(AXIS_X) && !defined(AXIS_Y) && !defined(AXIS_Z) && !defined(TEMPERATURE) && !defined(TEST_DMP)
  #error "An axis should be defined if in gyro/accel/mag mode"
#endif

#if defined(TEST_FREQUENCY)
  #if defined(AXIS_X)
    double gyroLastX, accelLastX, magLastX;
  #endif
  #if defined(AXIS_Y)
    double gyroLastY, accelLastY, magLastY;
  #endif
  #if defined(AXIS_Z)
    double gyroLastZ, accelLastZ, magLastZ;
  #endif
  int sensorHzCounter;
  int processorHzCounter;
#endif //TEST_FREQUENCY

#if defined(TEST_RANGE)
  #if defined(AXIS_X)
    double gyroLastX, gyroTotalX, accelLastX, accelTotalX;
  #endif
  #if defined(AXIS_Y)
    double gyroLastY, gyroTotalY, accelLastY, accelTotalY;
  #endif
  #if defined(AXIS_Z)
    double gyroLastZ, gyroTotalZ, accelLastZ, accelTotalZ;
  #endif
#endif //TEST_RANGE

#if defined(TEST_RAW) || defined(TEST_SCALED) || defined(TEST_FILTERED)
  uint32_t packetsPrinted;
#endif //TEST_RAW & TEST_SCALED & TEST_FILTERED

double lastMillis;

bool changed;

double timeLastMillis;

double gravity;

void setup() {
  Serial.begin(115200);
  delay(5000);
  bool started = false;
  gravity = DMath.calculateGravity(40.343590, 640);
  bool initialized;
  for(int i = 0; i < 10; i++){
    #if !defined(TEST_RAW_GRAPH) && !defined(TEST_SCALED_GRAPH) && !defined(TEST_FILTERED_GRAPH)
      Serial.println("Initializing...");
    #endif
    initialized = DICM20948.begin();
    if(!initialized && i != 3){
      #if !defined(TEST_RAW_GRAPH) && !defined(TEST_SCALED_GRAPH) && !defined(TEST_FILTERED_GRAPH)
        Serial.println("A problem occurred while initializing the sensor");
        Serial.println("Reading error codes: ");
        DErrorCodes.printLastErrorCodes();
        Serial.println();
      #endif
      delay(1000);
    }else if(initialized){
      #if !defined(TEST_RAW_GRAPH) && !defined(TEST_SCALED_GRAPH) && !defined(TEST_FILTERED_GRAPH)
        Serial.println("IMU initialized correctly");
        Serial.println("Reading error codes: ");
        DErrorCodes.printLastErrorCodes();
        Serial.println();
      #endif
      started = true;
      break;
    }
  }
  if(!started){
    #if !defined(TEST_RAW_GRAPH) && !defined(TEST_SCALED_GRAPH) && !defined(TEST_FILTERED_GRAPH)
      Serial.println("Reading error codes: ");
      DErrorCodes.printLastErrorCodes();
      Serial.println("There was an error while initializing the IMU, entering in a self sustained infinite loop");
    #endif
    while(true){}
  }
  #if !defined(TEST_RAW_GRAPH) && !defined(TEST_SCALED_GRAPH) && !defined(TEST_FILTERED_GRAPH)
    Serial.println("Calibrating sensor...");
  #endif
  DICM20948.calibrateSensors(gravity);
  #if !defined(TEST_RAW_GRAPH) && !defined(TEST_SCALED_GRAPH) && !defined(TEST_FILTERED_GRAPH)
    Serial.println("Reading error codes: ");
    DErrorCodes.printLastErrorCodes();
    Serial.println();
  #endif

  #if !defined(TEST_RAW_GRAPH) && !defined(TEST_SCALED_GRAPH) && !defined(TEST_FILTERED_GRAPH)
    Serial.println("First test: ");
  #endif
  DICM20948.updateSensors(gravity, OBTAIN_RAW);
  executeTests();
  #if !defined(TEST_RAW_GRAPH) && !defined(TEST_SCALED_GRAPH) && !defined(TEST_FILTERED_GRAPH)
    Serial.println("Reading error codes: ");
    DErrorCodes.printLastErrorCodes();
    Serial.println();
  #endif

  Serial.println(gravity);

  timeLastMillis = millis();
}

//Raw data printing
void printRawGyroData(bool plot = false){
  if(!plot) Serial.print("Raw gyroscope data, ");
  #if defined(AXIS_X)
    if(!plot) Serial.print("X: ");
    Serial.print(DICM20948.getRawGyroscopeData().X);
  #endif
  #if defined(AXIS_Y)
    if(!plot) Serial.print(" Y: ");
    Serial.print(DICM20948.getRawGyroscopeData().Y);
  #endif
  #if defined(AXIS_Z)
    if(!plot) Serial.print(" Z: ");
    Serial.print(DICM20948.getRawGyroscopeData().Z);
  #endif
  if(!plot) Serial.println();
}

void printRawAccelData(bool plot = false){
  if(!plot) Serial.print("Raw accelerometer data, ");
  #if defined(AXIS_X)
    if(!plot) Serial.print("X: ");
    Serial.print(DICM20948.getRawAccelerometerData().X);
  #endif
  #if defined(AXIS_Y)
    if(!plot) Serial.print(" Y: ");
    Serial.print(DICM20948.getRawAccelerometerData().Y);
  #endif
  #if defined(AXIS_Z)
    if(!plot) Serial.print(" Z: ");
    Serial.print(DICM20948.getRawAccelerometerData().Z);
  #endif
  if(!plot) Serial.println();
}

void printRawMagData(bool plot = false){
  if(!plot) Serial.print("Raw magnetometer data, ");
  #if defined(AXIS_X)
    if(!plot) Serial.print("X: ");
    Serial.print(DICM20948.getRawMagnetometerData().X);
  #endif
  #if defined(AXIS_Y)
    if(!plot) Serial.print(" Y: ");
    Serial.print(DICM20948.getRawMagnetometerData().Y);
  #endif
  #if defined(AXIS_Z)
    if(!plot) Serial.print(" Z: ");
    Serial.print(DICM20948.getRawMagnetometerData().Z);
  #endif
  if(!plot) Serial.println();
}

//Scaled data printing
void printScaledGyroData(bool plot = false){
  double gyroTemp;
  if(!plot) Serial.print("Scaled gyroscope data, ");
  #if defined(AXIS_X)
    if(!plot) Serial.print("X: ");
    gyroTemp = DICM20948.getScaledGyroscopeData().X;
    if(plot){
      Serial.print(gyroTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(gyroTemp, 6);
    }
    if(!plot) Serial.print("rad/s ");
  #endif
  #if defined(AXIS_Y)
    if(!plot) Serial.print("Y: ");
    gyroTemp = DICM20948.getScaledGyroscopeData().Y;
    if(plot){
      Serial.print(gyroTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(gyroTemp, 6);
    }
    if(!plot) Serial.print("rad/s ");
  #endif
  #if defined(AXIS_Z)
    if(!plot) Serial.print("Z: ");
    gyroTemp = DICM20948.getScaledGyroscopeData().Z;
    if(plot){
      Serial.print(gyroTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(gyroTemp, 6);
    }
    if(!plot) Serial.print("rad/s");
  #endif
  if(!plot) Serial.println();
}

void printScaledAccelData(bool plot = false){
  double accelTemp;
  if(!plot) Serial.print("Scaled accelerometer data, ");
  #if defined(AXIS_X)
    if(!plot) Serial.print("X: ");
    accelTemp = DICM20948.getScaledAccelerometerData().X;
    if(plot){
      Serial.print(accelTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(accelTemp, 6);
    }
    if(!plot) Serial.print("m/s^2 ");
  #endif
  #if defined(AXIS_Y)
    if(!plot) Serial.print("Y: ");
    accelTemp = DICM20948.getScaledAccelerometerData().Y;
    if(plot){
      Serial.print(accelTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(accelTemp, 6);
    }
    if(!plot) Serial.print("m/s^2 ");
  #endif
  #if defined(AXIS_Z)
    if(!plot) Serial.print("Z: ");
    accelTemp = DICM20948.getScaledAccelerometerData().Z;
    if(plot){
      Serial.print(accelTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(accelTemp, 6);
    }
    if(!plot) Serial.print("m/s^2");
  #endif
  if(!plot) Serial.println();
}

void printScaledMagData(bool plot = false){
  double magTemp;
  if(!plot) Serial.print("Scaled magnetometer data, ");
  #if defined(AXIS_X)
    if(!plot) Serial.print("X: ");
    magTemp = DICM20948.getScaledMagnetometerData().X;
    if(plot){
      Serial.print(magTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(magTemp, 6);
    }
  #endif
  #if defined(AXIS_Y)
    if(!plot) Serial.print(" Y: ");
    magTemp = DICM20948.getScaledMagnetometerData().Y;
    if(plot){
      Serial.print(magTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(magTemp, 6);
    }
  #endif
  #if defined(AXIS_Z)
    if(!plot) Serial.print(" Z: ");
    magTemp = DICM20948.getScaledMagnetometerData().Z;
    if(plot){
      Serial.print(magTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(magTemp, 6);
    }
  #endif
  if(!plot) Serial.println();
}

//Filtered data printing
void printFilteredGyroData(bool plot = false){
  double gyroTemp;
  if(!plot) Serial.print("Filtered gyroscope data, ");
  #if defined(AXIS_X)
    if(!plot) Serial.print("X: ");
    gyroTemp = DICM20948.getFilteredGyroscopeData().X;
    if(plot){
      Serial.print(gyroTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(gyroTemp, 6);
    }
    if(!plot) Serial.print("rad/s ");
  #endif
  #if defined(AXIS_Y)
    if(!plot) Serial.print("Y: ");
    gyroTemp = DICM20948.getFilteredGyroscopeData().Y;
    if(plot){
      Serial.print(gyroTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(gyroTemp, 6);
    }
    if(!plot) Serial.print("rad/s ");
  #endif
  #if defined(AXIS_Z)
    if(!plot) Serial.print("Z: ");
    gyroTemp = DICM20948.getFilteredGyroscopeData().Z;
    if(plot){
      Serial.print(gyroTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(gyroTemp, 6);
    }
    if(!plot) Serial.print("rad/s");
  #endif
  if(!plot) Serial.println();
}

void printFilteredAccelData(bool plot = false){
  double accelTemp;
  if(!plot) Serial.print("Filtered accelerometer data, ");
  #if defined(AXIS_X)
    if(!plot) Serial.print("X: ");
    accelTemp = DICM20948.getFilteredAccelerometerData().X;
    if(plot){
      Serial.print(accelTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(accelTemp, 6);
    }
    if(!plot) Serial.print("m/s^2 ");
  #endif
  #if defined(AXIS_Y)
    if(!plot) Serial.print("Y: ");
    accelTemp = DICM20948.getFilteredAccelerometerData().Y;
    if(plot){
      Serial.print(accelTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(accelTemp, 6);
    }
    if(!plot) Serial.print("m/s^2 ");
  #endif
  #if defined(AXIS_Z)
    if(!plot) Serial.print("Z: ");
    accelTemp = DICM20948.getFilteredAccelerometerData().Z;
    if(plot){
      Serial.print(accelTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(accelTemp, 6);
    }
    if(!plot) Serial.print("m/s^2");
  #endif
  if(!plot) Serial.println();
}

void printFilteredMagData(bool plot = false){
  double magTemp;
  if(!plot) Serial.print("Filtered magnetometer data, ");
  #if defined(AXIS_X)
    if(!plot) Serial.print("X: ");
    magTemp = DICM20948.getFilteredMagnetometerData().X;
    if(plot){
      Serial.print(magTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(magTemp, 6);
    }
  #endif
  #if defined(AXIS_Y)
    if(!plot) Serial.print(" Y: ");
    magTemp = DICM20948.getFilteredMagnetometerData().Y;
    if(plot){
      Serial.print(magTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(magTemp, 6);
    }
  #endif
  #if defined(AXIS_Z)
    if(!plot) Serial.print(" Z: ");
    magTemp = DICM20948.getFilteredMagnetometerData().Z;
    if(plot){
      Serial.print(magTemp * SCALE_RATE);
      Serial.print(" ");
    }else{
      DSerialHelper.printDouble(magTemp, 6);
    }
  #endif
  if(!plot) Serial.println();
}

void executeTests(){
  #if defined(TEST_FREQUENCY)
    changed = false;

    #if defined(GYROSCOPE)
      //Gyroscope
      #if defined(AXIS_X)
        if(DICM20948.getScaledGyroscopeData().X != lastX){
          lastX = DICM20948.getScaledGyroscopeData().X;
          changed = true;
        }
      #endif
      #if defined(AXIS_Y)
        if(DICM20948.getScaledGyroscopeData().Y != lastY){
          lastY = DICM20948.getScaledGyroscopeData().Y;
          changed = true;
        }
      #endif
      #if defined(AXIS_Z)
        if(DICM20948.getScaledGyroscopeData().Z != lastZ){
          lastZ = DICM20948.getScaledGyroscopeData().Z;
          changed = true;
        }
      #endif
    #endif //GYROSCOPE

    #if defined(ACCELEROMETER)
      //Accelerometer
      #if defined(AXIS_X)
        if(DICM20948.getScaledAccelerometerData().X != lastX){
          lastX = DICM20948.getScaledAccelerometerData().X;
          changed = true;
        }
      #endif
      #if defined(AXIS_Y)
        if(DICM20948.getScaledAccelerometerData().Y != lastY){
          lastY = DICM20948.getScaledAccelerometerData().Y;
          changed = true;
        }
      #endif
      #if defined(AXIS_Z)
        if(DICM20948.getScaledAccelerometerData().Z != lastZ){
          lastZ = DICM20948.getScaledAccelerometerData().Z;
          changed = true;
        }
      #endif
    #endif //ACCELEROMETER

    #if defined(MAGNETOMETER)
      //Magnetometer
      #if defined(AXIS_X)
        if(DICM20948.getScaledMagnetometerData().X != lastX){
          lastX = DICM20948.getScaledMagnetometerData().X;
          changed = true;
        }
      #endif
      #if defined(AXIS_Y)
        if(DICM20948.getScaledMagnetometerData().Y != lastY){
          lastY = DICM20948.getScaledMagnetometerData().Y;
          changed = true;
        }
      #endif
      #if defined(AXIS_Z)
        if(DICM20948.getScaledMagnetometerData().Z != lastZ){
          lastZ = DICM20948.getScaledMagnetometerData().Z;
          changed = true;
        }
      #endif
    #endif //MAGNETOMETER
  
    //General stuff
    if(changed){
      sensorHzCounter++;
    }
    processorHzCounter++;
    if(millis() - lastMillis > 1000){
      lastMillis = millis();
      Serial.print("Processor real frequency: ");
      Serial.print(processorHzCounter);
      Serial.print("Hz ,sensor real frequency: ");
      Serial.print(sensorHzCounter);
      Serial.println("Hz");
      sensorHzCounter = 0;
      processorHzCounter = 0;
    }
  #endif //TEST_FREQUENCY

  #if defined(TEST_RANGE)

    #if defined(GYROSCOPE)
      #if defined(AXIS_X)
        gyroTotalX += abs(DICM20948.getScaledGyroscopeData().X - gyroLastX);
        gyroLastX = DICM20948.getScaledGyroscopeData().X;
      #endif
      #if defined(AXIS_Y)
        gyroTotalY += abs(DICM20948.getScaledGyroscopeData().Y - gyroLastY);
        gyroLastY = DICM20948.getScaledGyroscopeData().Y;
      #endif
      #if defined(AXIS_Z)
        gyroTotalZ += abs(DICM20948.getScaledGyroscopeData().Z - gyroLastZ);
        gyroLastZ = DICM20948.getScaledGyroscopeData().Z;
      #endif
    #endif //GYROSCOPE

    #if defined(ACCELEROMETER)
      #if defined(AXIS_X)
        accelTotalX += abs(DICM20948.getScaledAccelerometerData().X - accelLastX);
        accelLastX = DICM20948.getScaledAccelerometerData().X;
      #endif
      #if defined(AXIS_Y)
        accelTotalY += abs(DICM20948.getScaledAccelerometerData().Y - accelLastY);
        accelLastY = DICM20948.getScaledAccelerometerData().Y;
      #endif
      #if defined(AXIS_Z)
        accelTotalZ += abs(DICM20948.getScaledAccelerometerData().Z - accelLastZ);
        accelLastZ = DICM20948.getScaledAccelerometerData().Z;
      #endif
    #endif //ACCELEROMETER

    if(millis() - lastMillis > 1000){
      lastMillis = millis();

      #if defined(GYROSCOPE)
        Serial.print("Gyroscope");
        #if defined(AXIS_X)
          Serial.print(" range X: ");
          Serial.print(gyroTotalX);
          gyroTotalX = 0;
        #endif
        #if defined(AXIS_Y)
          Serial.print(" range Y: ");
          Serial.print(gyroTotalY);
          gyroTotalY = 0;
        #endif
        #if defined(AXIS_Z)
          Serial.print(" range Z: ");
          Serial.print(gyroTotalZ);
          gyroTotalZ = 0;
        #endif
      #endif //GYROSCOPE
      
      #if defined(ACCELEROMETER)
        Serial.print("Accelerometer");
        #if defined(AXIS_X)
          Serial.print(" range X: ");
          Serial.print(accelTotalX);
          accelTotalX = 0;
        #endif
        #if defined(AXIS_Y)
          Serial.print(" range Y: ");
          Serial.print(accelTotalY);
          accelTotalY = 0;
        #endif
        #if defined(AXIS_Z)
          Serial.print(" range Z: ");
          Serial.print(accelTotalZ);
          accelTotalZ = 0;
        #endif
      #endif //ACCELEROMETER

      Serial.println();
    }
    
  #endif //TEST_RANGE

  #if defined(TEST_RAW)
    Serial.print("Packet No ");
    Serial.println(packetsPrinted);

    #if defined(GYROSCOPE)
      printRawGyroData();
    #endif //GYROSCOPE
    
    #if defined(ACCELEROMETER)
      printRawAccelData();
    #endif //ACCELEROMETER

    #if defined(MAGNETOMETER)
      printRawMagData();
    #endif //MAGNETOMETER

    #if defined(TEMPERATURE)
      Serial.print("Raw temperature data: ");
      Serial.println(DICM20948.getRawTemperature());
    #endif //TEMPERATURE

    packetsPrinted++;

  #endif //TEST_RAW

  #if defined(TEST_SCALED)
    Serial.print("Packet No ");
    Serial.println(packetsPrinted);

    #if defined(GYROSCOPE)
      printScaledGyroData();
    #endif //GYROSCOPE

    #if defined(ACCELEROMETER)
      printScaledAccelData();
    #endif //ACCELEROMETER

    #if defined(MAGNETOMETER)
      printScaledMagData();
    #endif //MAGNETOMETER

    #if defined(TEMPERATURE)
      Serial.print("Scaled temperature data: ");
      Serial.print(DICM20948.getScaledTemperature());
      Serial.println(" ºC");
    #endif //TEMPERATURE

    packetsPrinted++;

  #endif //TEST_SCALED

  #if defined(TEST_FILTERED)
    Serial.print("Packet No ");
    Serial.println(packetsPrinted);

    #if defined(GYROSCOPE)
      printFilteredGyroData();
    #endif //GYROSCOPE

    #if defined(ACCELEROMETER)
      printFilteredAccelData();
    #endif //ACCELEROMETER

    #if defined(MAGNETOMETER)
      printFilteredMagData();
    #endif //MAGNETOMETER

    packetsPrinted++;

  #endif //TEST_FILTERED


  //TODO
  #if defined(TEST_RAW_GRAPH)
    #if defined(GYROSCOPE)
      printRawGyroData(true);
      Serial.print(" ");
    #endif
    #if defined(ACCELEROMETER)
      printRawAccelData(true);
      Serial.print(" ");
    #endif
    #if defined(MAGNETOMETER)
      printRawMagData(true);
      Serial.print(" ");
    #endif
    #if defined(TEMPERATURE)
      Serial.print(DICM20948.getRawTemperature());
      Serial.print(" ");
    #endif
  #endif //TEST_RAW_GRAPH

  #if defined(TEST_SCALED_GRAPH)
    #if defined(GYROSCOPE)
      printScaledGyroData(true);
      Serial.print(" ");
    #endif
    #if defined(ACCELEROMETER)
      printScaledAccelData(true);
      Serial.print(" ");
    #endif
    #if defined(MAGNETOMETER)
      printScaledMagData(true);
      Serial.print(" ");
    #endif
    #if defined(TEMPERATURE)
      Serial.print(DICM20948.getScaledTemperature()*SCALE_RATE);
      Serial.print(" ");
    #endif
  #endif //TEST_SCALED_GRAPH

  #if defined(TEST_FILTERED_GRAPH)
    #if defined(GYROSCOPE)
      printFilteredGyroData(true);
      Serial.print(" ");
    #endif
    #if defined(ACCELEROMETER)
      printFilteredAccelData(true);
      Serial.print(" ");
    #endif
    #if defined(MAGNETOMETER)
      printFilteredMagData(true);
      Serial.print(" ");
    #endif
  #endif //TEST_FILTERED_GRAPH

  #if defined(TEST_RAW_GRAPH) || defined(TEST_SCALED_GRAPH) || defined(TEST_FILTERED_GRAPH)
    Serial.println();
  #endif
}
void loop() {
  #if defined(TEST_RESET)
    if(millis() - timeLastMillis >= 1000/RESET_RATE){
      Serial.println("Resetting...");
      DICM20948.dynamicReset();
      Serial.print("Reset time: ");
      Serial.print(DICM20948.getLastExecutionTime());
      Serial.println("us");
      Serial.println("Reset error codes: ");
      DErrorCodes.printLastErrorCodes();
      delay(15); //To give time to the sensors to normalize, has to be dynamic in the processor
      DICM20948.updateSensors(gravity, OBTAIN_RAW);
      executeTests();
      Serial.println("Reading error codes: ");
      DErrorCodes.printLastErrorCodes();
      Serial.println();
      timeLastMillis = millis();
    }
  #elif defined(TEST_PDMP)
    if(millis() - timeLastMillis >= 1000/DMP_RATE){
      DICM20948.updateSensors(gravity, OBTAIN_RAW);
      //MadgwickAHRSupdateIMU(DICM20948.getScaledGyroscopeData().X, DICM20948.getScaledGyroscopeData().Y, DICM20948.getScaledGyroscopeData().Z, DICM20948.getScaledAccelerometerData().X, DICM20948.getScaledAccelerometerData().Y, DICM20948.getScaledAccelerometerData().Z);//, DICM20948.getScaledMagnetometerData().X, DICM20948.getScaledMagnetometerData().Y, DICM20948.getScaledMagnetometerData().Z);
      droneQuaternion testQuaternion;
      testQuaternion.x = q0;
      testQuaternion.y = q1;
      testQuaternion.z = q2;
      testQuaternion.w = q3;

      droneEulerAngles testEulerAngles = DMath.toEulerAngles(testQuaternion);
      /*Serial.print("Quaternion representation, q0: ");
      DSerialHelper.printDouble(q0, 6);
      Serial.print(", q1: ");
      DSerialHelper.printDouble(q1, 6);
      Serial.print(", q2: ");
      DSerialHelper.printDouble(q2, 6);
      Serial.print(", q3: ");
      DSerialHelper.printDouble(q3, 6);
      Serial.println();*/
      Serial.print("Roll: ");
      DSerialHelper.printDouble(DMath.radsToDps(testEulerAngles.roll) * SCALE_RATE + 180.0, 2); //10Min still - 41.7º Notable
      Serial.print(", pitch: ");
      DSerialHelper.printDouble(DMath.radsToDps(testEulerAngles.pitch) * SCALE_RATE, 2); //10Min still - 3.5º Neglegible
      Serial.print(", yaw: ");
      DSerialHelper.printDouble(DMath.radsToDps(testEulerAngles.yaw) * SCALE_RATE, 2); //10Min still - 4.5º Neglegible
      Serial.println();
      timeLastMillis = millis();
    }
  #else
    DICM20948.updateSensors(gravity, OBTAIN_RAW);
    if(millis() - timeLastMillis >= 1000/UPDATE_RATE){
      #if !defined(TEST_RAW_GRAPH) && !defined(TEST_SCALED_GRAPH) && !defined(TEST_FILTERED_GRAPH)
        Serial.println("Testing...");
      #endif
      executeTests();
      #if !defined(TEST_RAW_GRAPH) && !defined(TEST_SCALED_GRAPH) && !defined(TEST_FILTERED_GRAPH)
        Serial.print("Test time: ");
        Serial.print(DICM20948.getLastExecutionTime());
        Serial.println("us");
        Serial.println("Reading error codes: ");
        DErrorCodes.printLastErrorCodes();
        Serial.println();
      #endif
      timeLastMillis = millis();
    }
  #endif
}
