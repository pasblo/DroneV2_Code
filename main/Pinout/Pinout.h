#ifndef PINOUT_H
#define PINOUT_H

#ifdef MAIN_PROCESSOR
	
	//Main thread SPI comunication
	#define SDI_PIN 11
	#define SDO_PIN 12
	#define SCK_PIN 13
	#define CS_TEENSY_PIN 9
	#define CS_IMU_PIN 20
	#define CS_BMP_PIN 22
	#define CS_FLASH_PIN 10
	#define INT_IMU_PIN 21
	#define INT_BMP_PIN 23
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(ARDUINO_TEENSY41) //Only for teensy 3.5, 3.6 & 4.1
		#define CS_OSD_PIN 25
		#define CS_ESP_PIN 26
		#define INT_TEENSY_PIN 27
		#define COMPLETE_SPI_AVAILABLE
	#endif

	//ESC comunication
	#define MOTOR1_PIN 3
	#define MOTOR2_PIN 4
	#define MOTOR3_PIN 5
	#define MOTOR4_PIN 6
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) //Only for teensy 3.5 & 3.6
		#define MOTOR5_PIN 29
		#define MOTOR6_PIN 30
		#define HEXACOPTER_AVAILABLE
	#endif

	//PPM comunication
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) //Only for teensy 3.5 & 3.6
		#define PPM_PIN 38
		#define PPM_AVAILABLE
	#endif

	//Failsafe checking
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(ARDUINO_TEENSY41) //Only for teensy 3.5, 3.6 & 4.1
		#define FAILSAFE_PIN 24
		#define FAILSAFE_AVAILABLE
	#endif

	//Voltage sensing
	#define CELL1_VOLTAGE_PIN 14
	#define CELL2_VOLTAGE_PIN 15
	#define CELL3_VOLTAGE_PIN 16

	//Current sensing
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(ARDUINO_TEENSY41) //Only for teensy 3.5, 3.6 & 4.1
		#define PCB_CURRENT_PIN A20
		#define PCB_CURRENT_AVAILABLE
	#endif
	#define DRONE_CURRENT_PIN 19

	//Temperature sensing
	#define TEMPERATURE_SENSOR_PIN 17

	//PCB RGB led
	#define LED_BLUE_PIN 18
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) //Only for teensy 3.5 & 3.6
		#define LED_RED_PIN A11
		#define LED_GREEN_PIN A10
		#define COMPLETE_RGB_AVAILABLE
	#endif

	//Servo motor controlling
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) //Only for teensy 3.5 & 3.6. Teensy 4.1 can controll servos 2 & 3 TODO
		#define SERVO1_PIN 35
		#define SERVO2_PIN 36
		#define SERVO3_PIN 37
		#define SERVOS_AVAILABLE
	#endif

	//Buzzer controlling
	#define BUZZER_PIN 2
	#if defined(__IMXRT1062__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) //Only for teensy 3.5, 3.6, 4.0 & 4.1
		#define PWM_BUZZER_AVAILABLE
	#endif

#elif defined(EXTERNAL_PROCESSOR)
	
	//SPI comunication
	#define SDI_PIN 12
	#define SDO_PIN 11
	#define SCK_PIN 13
	#define CS_MAIN_PIN 10
	#define INT_MAIN_PIN 33

	//I2C comunication
	#define SCL 19
	#define SDA 18

	//ESC failsafe comunication
	#define MOTOR1_PIN 20
	#define MOTOR2_PIN 21
	#define MOTOR3_PIN 22
	#define MOTOR4_PIN 23
	#define MOTOR5_PIN 25
	#define MOTOR6_PIN 32

	//Failsafe check
	#define FAILSAFE_PIN A10

	//Temperature sensing
	#define TEMPERATURE_SENSOR1_PIN 29 //Internal temperatue sensor
	#define TEMPERATURE_SENSOR2_PIN 30
	#define TEMPERATURE_SENSOR3_PIN 31

	//PCB RGB led
	#define PCB_LED_RED_PIN 26
	#define PCB_LED_GREEN_PIN 27
	#define PCB_LED_BLUE_PIN 28

	//ARM RGB led
	#define ARM_LED_RED_PIN 14
	#define ARM_LED_GREEN_PIN 15
	#define ARM_LED_BLUE_PIN 16

	//High altitude led
	#define HIGH_ALT1_PIN 2
	#define HIGH_ALT2_PIN 24

#elif defined(ESP_PROCESSOR)
	
	//SPI comunication
	#define SDI_PIN 19
	#define SDO_PIN 23
	#define CS_MAIN_PIN 5
	
	//Servo comunication
	#define SERVO1_PIN 27
	#define SERVO2_PIN 13
	#define SERVO3_PIN 0
	#define SERVO4_PIN 22

	//Ultrasonic sensing
	#define TRIG1_PIN 25
	#define ECHO1_PIN 26
	#define TRIG2_PIN 14
	#define ECHO2_PIN 12
	#define TRIG3_PIN 15
	#define ECHO3_PIN 2
	#define TRIG4_PIN 4
	#define ECHO4_PIN 21

#endif

class Pinout{
	public:
		static void init();
};
#endif