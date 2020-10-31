#include "Pinout.h"
#include "Arduino.h"

void Pinout::init(){
	
	#if defined(MAIN_PROCESSOR)

		//SPI CS & INT pins initialization
		pinMode(CS_TEENSY_PIN, OUTPUT);
		digitalWrite(CS_TEENSY_PIN, HIGH);
		pinMode(CS_IMU_PIN, OUTPUT);
		digitalWrite(CS_IMU_PIN, HIGH);
		pinMode(INT_IMU_PIN, INPUT);
		pinMode(CS_BMP_PIN, OUTPUT);
		digitalWrite(CS_BMP_PIN, HIGH);
		pinMode(INT_BMP_PIN, INPUT);
		pinMode(CS_FLASH_PIN, OUTPUT);
		digitalWrite(CS_FLASH_PIN, HIGH);
		#ifdef COMPLETE_SPI_AVAILABLE
			pinMode(CS_OSD_PIN, OUTPUT);
			digitalWrite(CS_OSD_PIN, HIGH);
			pinMode(CS_ESP_PIN, OUTPUT);
			digitalWrite(CS_ESP_PIN, HIGH);
			pinMode(INT_TEENSY_PIN, INPUT);
		#endif

		//ESC pins initialization
		pinMode(MOTOR1_PIN, OUTPUT);
		digitalWrite(MOTOR1_PIN, LOW);
		pinMode(MOTOR2_PIN, OUTPUT);
		digitalWrite(MOTOR2_PIN, LOW);
		pinMode(MOTOR3_PIN, OUTPUT);
		digitalWrite(MOTOR3_PIN, LOW);
		pinMode(MOTOR4_PIN, OUTPUT);
		digitalWrite(MOTOR4_PIN, LOW);
		#ifdef HEXACOPTER_AVAILABLE
			pinMode(MOTOR5_PIN, OUTPUT);
			digitalWrite(MOTOR5_PIN, LOW);
			pinMode(MOTOR6_PIN, OUTPUT);
			digitalWrite(MOTOR6_PIN, LOW);
		#endif

		//PPM pin initialization
		#ifdef PPM_AVAILABLE
			pinMode(PPM_PIN, INPUT);
		#endif

		//Failsafe pin initialization
		#ifdef FAILSAFE_AVAILABLE
			pinMode(FAILSAFE_PIN, OUTPUT);
		#endif

		//Voltage sensing pins initialization
		pinMode(CELL1_VOLTAGE_PIN, INPUT);
		pinMode(CELL2_VOLTAGE_PIN, INPUT);
		pinMode(CELL3_VOLTAGE_PIN, INPUT);

		//PCB current sensing pin initializations
		#ifdef PCB_CURRENT_AVAILABLE
			pinMode(PCB_CURRENT_PIN, INPUT);
		#endif
		pinMode(DRONE_CURRENT_PIN, INPUT);

		//Temperature sensing pin initialization
		pinMode(TEMPERATURE_SENSOR_PIN, INPUT);

		//PCB RGB pins initialization
		pinMode(LED_BLUE_PIN, OUTPUT);
		#ifdef COMPLETE_RGB_AVAILABLE
			pinMode(LED_RED_PIN, OUTPUT);
			pinMode(LED_GREEN_PIN, OUTPUT);
		#endif

		//Servo pins initialization
		#ifdef SERVOS_AVAILABLE
			pinMode(SERVO1_PIN, OUTPUT);
			pinMode(SERVO2_PIN, OUTPUT);
			pinMode(SERVO3_PIN, OUTPUT);
		#endif

		//Buzzer pin initialization
		pinMode(BUZZER_PIN, OUTPUT);
		
	#elif defined(EXTERNAL_PROCESSOR)

		//SPI CS & INT pins initialization
		pinMode(CS_MAIN_PIN, INPUT);
		pinMode(INT_MAIN_PIN, OUTPUT);

		//Esc failsafe pins initialization
		pinMode(MOTOR1_PIN, OUTPUT);
		pinMode(MOTOR2_PIN, OUTPUT);
		pinMode(MOTOR3_PIN, OUTPUT);
		pinMode(MOTOR4_PIN, OUTPUT);
		pinMode(MOTOR5_PIN, OUTPUT);
		pinMode(MOTOR6_PIN, OUTPUT);

		//Failsafe check pin initialization
		pinMode(FAILSAFE_PIN, INPUT);

		//Temperature sensing pins initialization
		pinMode(TEMPERATURE_SENSOR1_PIN, INPUT);
		pinMode(TEMPERATURE_SENSOR2_PIN, INPUT);
		pinMode(TEMPERATURE_SENSOR3_PIN, INPUT);

		//PCB RGB led pins initialization
		pinMode(PCB_LED_RED_PIN, OUTPUT);
		pinMode(PCB_LED_GREEN_PIN, OUTPUT);
		pinMode(PCB_LED_BLUE_PIN, OUTPUT);

		//ARM RGB led pins initialization
		pinMode(ARM_LED_RED_PIN, OUTPUT);
		pinMode(ARM_LED_GREEN_PIN, OUTPUT);
		pinMode(ARM_LED_BLUE_PIN, OUTPUT);

		//High altitude leds pins initilization
		pinMode(HIGH_ALT1_PIN, OUTPUT);
		pinMode(HIGH_ALT2_PIN, OUTPUT);

	#elif defined(ESP_PROCESSOR)

		//SPI CS pin initialization
		pinMode(CS_MAIN_PIN, INPUT);

		//Servo pins initialization
		pinMode(SERVO1_PIN, OUTPUT);
		pinMode(SERVO2_PIN, OUTPUT);
		pinMode(SERVO3_PIN, OUTPUT);
		pinMode(SERVO4_PIN, OUTPUT);

		//Ultrasonic pins initialization
		pinMode(TRIG1_PIN, OUTPUT);
		pinMode(ECHO1_PIN, INPUT);
		pinMode(TRIG2_PIN, OUTPUT);
		pinMode(ECHO2_PIN, INPUT);
		pinMode(TRIG3_PIN, OUTPUT);
		pinMode(ECHO3_PIN, INPUT);
		pinMode(TRIG4_PIN, OUTPUT);
		pinMode(ECHO4_PIN, INPUT);

	#endif
}