#ifndef ICM20948_H
#define ICM20948_H

//Detecting if this library is being used correctly
#ifndef MAIN_PROCESSOR
	#error "This library is only intended to be used on the Main processor"
#endif

//Libraries used
#include "SPI.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Time/DroneTime.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/ErrorCodes.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/SerialHelper.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Math.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Filters.h"
#include "C:/Users/pabri/Desktop/Electronica/DronProyect/V2/DroneV2_Code/main/Utils/Filters.cpp"

//Register directions for all banks
#define ICM20948_REG_BANK_SEL 0x7F				//< register bank selection register

//Register directions for bank 0
#define ICM20948_B0_WHOAMI 0x00						//< chip ID register
#define ICM20948_B0_USER_CTRL 0x03					//< reset and enable functionalities control register
#define ICM20948_B0_PWR_MGMT_1 0x06					//< primary power management register
#define ICM20948_B0_PWR_MGMT_2 0x07					//< secondary power management register
#define ICM20948_B0_INT_ENABLE 0x10					//< 
#define ICM20948_B0_INT_ENABLE_2 0x12				//< 
#define ICM20948_B0_REG_INT_PIN_CFG 0x0f			//< multiple pin configuration register
#define ICM20948_B0_I2C_MST_STATUS 0x17				//< status of the I2C master register
#define ICM20948_B0_SINGLE_FIFO_PRIORITY_SEL 0x26	//< 
#define ICM20948_B0_ACCEL_DATA_OUT 0x2d				//< starting point of the accelerometer data registers
#define ICM20948_B0_GYRO_DATA_OUT 0x33				//< starting point of the gyroscope data registers
#define ICM20948_B0_MAG_DATA_OUT 0x3b				//< starting point of the magnetometer data registers
#define ICM20948_B0_FIFO_EN 0x66					//< 
#define ICM20948_B0_FIFO_EN2 0x67					//< 
#define ICM20948_B0_FIFO_RST 0x68					//< 
#define ICM20948_B0_HW_FIX_DISABLE 0x75				//< 
#define ICM20948_B0_FIFO_CFG 0x76					//< 
#define ICM20948_B0_DMP_START_ADDR 0x7C				//< 
#define ICM20948_B0_DMP_MEM_R_W 0x7D				//< 
#define ICM20948_B0_DMP_BANK_SEL 0x7E				//< 

//Register directions for bank 1
#define ICM20948_B1_SELF_TEST_DATA 0x02				//< starting point of all the self test data registers

//Register directions for bank 2
#define ICM20948_B2_GYRO_SMPLRT_DIV 0x00    		//< gyro data rate divisor
#define ICM20948_B2_GYRO_CONFIG_1 0x01				//< primary gyro configuration register
#define ICM20948_B2_GYRO_CONFIG_2 0x02				//< secondary gyro configuration register
#define ICM20948_B2_ACCEL_SMPLRT_DIV_1 0x10 		//< accel data rate divisor MSByte
#define ICM20948_B2_ACCEL_CONFIG_1 0x14				//< primary accel configuration register
#define ICM20948_B2_ACCEL_CONFIG_2 0x15				//< secondary accel configuration register
#define ICM20948_B2_PRGM_START_ADDRH 0x50			//< 
#define ICM20948_B2_TEMP_CONFIG 0x53				//< temperature configuration register

//Register directions for bank 3
#define ICM20948_B3_I2C_MST_ODR_CONFIG 0x00			//< i2c master configuration register
#define ICM20948_B3_I2C_MST_CTRL 0x01				//< 
#define ICM20948_B3_I2C_SLV0_ADDR 0x03				//< slave 0 I2C address register
#define ICM20948_B3_I2C_SLV0_REG 0x04				//< slave 0 register address register
#define ICM20948_B3_I2C_SLV0_CTRL 0x05				//< slave 0 control register
#define ICM20948_B3_I2C_SLV4_ADDR 0x13				//< slave 4 I2C address register
#define ICM20948_B3_I2C_SLV4_REG 0x14				//< slave 4 register address register
#define ICM20948_B3_I2C_SLV4_CTRL 0x15				//< slave 4 control register
#define ICM20948_B3_I2C_SLV4_DO 0x16				//< slave 4 data output register
#define ICM20948_B3_I2C_SLV4_DI 0x17				//< slave 4 data input register

//Chip identifiers
#define ICM20948_ID 0xEA							//< ICM20948 chip id

//Other constants
#define RESET_TIMEOUT 100							//< time in ms that reset functions are going to last at maximun
#define MAG_SCALE 0.15								//< magnetometer scaling factor
#define PRE_CALIB_MEASUREMENTS 1000					//< number of measuremnets that are performed before the calibration process
#define CALIB_MEASUREMENTS 1000						//< number of measuremenets that are performed during the calibration process

//Data to obtain enum
typedef enum{
	OBTAIN_RAW, // Raw/Scaled/Filtered Gyro/Accel/Mag
	OBTAIN_COMPLETE_DMP, //Raw/Scaled/Filtered Gyro/Accel/Mag(DMP), 9dof
	OBTAIN_SHORT_DMP, // 9dof
	OBTAIN_ALL, // OBTAIN_RAW + OBTAIN_COMPLETE_DMP
} dataToObtain_e;

//Raw data structure
typedef struct {
	int16_t X;
	int16_t Y;
	int16_t Z;

} rawData_t;

//Sacled data structure
typedef struct {
	double X;
	double Y;
	double Z;

} scaledData_t;

//Filtered data structure
typedef struct {
	double X;
	double Y;
	double Z;

} filteredData_t;

//Register operations
typedef enum _RegType_e {
	ADDRBIT8_HIGH_TOREAD = 0,						//< indicates that the register is going to be read
	ADDRBIT8_HIGH_TOWRITE = 1,						//< indicates that the register is going to be writen
} RegType_e;

//Magnetometer status
typedef enum {
	AK09916_MAG_FREQ_SHUTDOWN = 0x00,				//< stops measurement updates
	AK09916_MAG_FREQ_SINGLE = 0x01,					//< sakes a single measurement then switches to AK09916_MAG_DATARATE_SHUTDOWN
	AK09916_MAG_FREQ_10_HZ = 0x02,					//< updates at 10Hz
	AK09916_MAG_FREQ_20_HZ = 0x04,					//< updates at 20Hz
	AK09916_MAG_FREQ_50_HZ = 0x06,					//< updates at 50Hz
	AK09916_MAG_FREQ_100_HZ = 0x08,					//< updates at 100Hz
	AK09916_MAG_FREQ_SELF_TEST = 0x10				//< self tests the sensor and saves the result onto the data registers
} ak09916_mag_rate_div_e;

//Gyroscope ranges
typedef enum {
	ICM20948_GYRO_RANGE_250_DPS = 0x0,				//< 4.3Rad/s, highest precision
	ICM20948_GYRO_RANGE_500_DPS = 0x1,				//< 8.7Rad/s
	ICM20948_GYRO_RANGE_1000_DPS = 0x2,				//< 17.4Rad/s
	ICM20948_GYRO_RANGE_2000_DPS = 0x3,				//< 34.9Rad/s, lowest precision
} icm20948_gyro_range_e;

//Accelerometer ranges
typedef enum {
	ICM20948_ACCEL_RANGE_2_G = 0x0,					//< +-1G, highest precision
	ICM20948_ACCEL_RANGE_4_G = 0x1,					//< +-2G
	ICM20948_ACCEL_RANGE_8_G = 0x2,					//< +-4G
	ICM20948_ACCEL_RANGE_16_G = 0x3,				//< +-8G, lowest precision
} icm20948_accel_range_e;

//Gyroscope LPF cutoff frequency
typedef enum {
	ICM20948_GYRO_CUTOFF_FREQ_196_6_HZ = 0x0,		//< The filter starts to be noticed at 196.6Hz signals
	ICM20948_GYRO_CUTOFF_FREQ_151_8_HZ = 0x1,		//< The filter starts to be noticed at 151.8Hz signals
	ICM20948_GYRO_CUTOFF_FREQ_119_5_HZ = 0x2,		//< The filter starts to be noticed at 119.5Hz signals
	ICM20948_GYRO_CUTOFF_FREQ_51_2_HZ = 0x3,		//< The filter starts to be noticed at 51.2Hz signals
	ICM20948_GYRO_CUTOFF_FREQ_23_9_HZ = 0x4,		//< The filter starts to be noticed at 23.9Hz signals
	ICM20948_GYRO_CUTOFF_FREQ_11_6_HZ = 0x5,		//< The filter starts to be noticed at 11.6Hz signals
	ICM20948_GYRO_CUTOFF_FREQ_5_7_HZ = 0x6,			//< The filter starts to be noticed at 5.7Hz signals
	ICM20948_GYRO_CUTOFF_FREQ_361_4_HZ = 0x7,		//< The filter starts to be noticed at 361.4Hz signals
	ICM20948_GYRO_CUTOFF_FREQ_DISABLED = 0x8,		//< TODO check if this is the correct number
} icm20948_gyro_cutoff_freq_e;

//Acceleroemeter LPF cutoff frequency
typedef enum {
	ICM20948_ACCEL_CUTOFF_FREQ_246_0_HZ = 0x1,		//< The filter starts to be noticed at 246.0Hz signals
	ICM20948_ACCEL_CUTOFF_FREQ_111_4_HZ = 0x2,		//< The filter starts to be noticed at 111.4Hz
	ICM20948_ACCEL_CUTOFF_FREQ_50_4_HZ = 0x3,		//< The filter starts to be noticed at 50.4Hz
	ICM20948_ACCEL_CUTOFF_FREQ_23_9_HZ = 0x4,		//< The filter starts to be noticed at 23.9Hz
	ICM20948_ACCEL_CUTOFF_FREQ_11_5_HZ = 0x5,		//< The filter starts to be noticed at 11.5Hz
	ICM20948_ACCEL_CUTOFF_FREQ_5_7_HZ = 0x6,		//< The filter starts to be noticed at 5.7Hz
	ICM20948_ACCEL_CUTOFF_FREQ_473_HZ = 0x7,		//< The filter starts to be noticed at 473.0Hz
	ICM20948_ACCEL_CUTOFF_FREQ_DISABLED = 0x8,		//< TODO check if this is the correct number
} icm20948_accel_cutoff_freq_e;

//Temperature LPF cutoff frequency
typedef enum {
	ICM20948_TEMP_CUTOFF_FREQ_217_HZ = 0x1,			//< The filter starts to be noticed at 217.0Hz
	ICM20948_TEMP_CUTOFF_FREQ_123_5_HZ = 0x2,		//< The filter starts to be noticed at 123.5Hz
	ICM20948_TEMP_CUTOFF_FREQ_65_9_HZ = 0x3,		//< The filter starts to be noticed at 65.9Hz
	ICM20948_TEMP_CUTOFF_FREQ_34_1_HZ = 0x4,		//< The filter starts to be noticed at 34.1Hz
	ICM20948_TEMP_CUTOFF_FREQ_17_3_HZ = 0x5,		//< The filter starts to be noticed at 17.3Hz
	ICM20948_TEMP_CUTOFF_FREQ_8_8_HZ = 0x6,			//< The filter starts to be noticed at 8.8Hz
	ICM20948_TEMP_CUTOFF_FREQ_7932_HZ = 0x7,		//< The filter starts to be noticed at 7932.0Hz
	ICM20948_TEMP_CUTOFF_FREQ_DISABLED = 0x8,		//< TODO check if this is the correct number
} icm20948_temp_cutoff_freq_e;

//Gyroscope averaging
typedef enum {
	ICM20948_GYRO_AVEG_1 = 0x0,						//< Each value is averaged 1 times
	ICM20948_GYRO_AVEG_2 = 0x1,						//< Each value is averaged 2 times
	ICM20948_GYRO_AVEG_4 = 0x2,						//< Each value is averaged 4 times
	ICM20948_GYRO_AVEG_8 = 0x3,						//< Each value is averaged 8 times
	ICM20948_GYRO_AVEG_16 = 0x4,					//< Each value is averaged 16 times
	ICM20948_GYRO_AVEG_32 = 0x5,					//< Each value is averaged 32 times
	ICM20948_GYRO_AVEG_64 = 0x6,					//< Each value is averaged 64 times
	ICM20948_GYRO_AVEG_128 = 0x7,					//< Each value is averaged 128 times
} icm20948_gyro_averaging_e;

//Accelerometer averaging
typedef enum {
	ICM20948_ACCEL_AVEG_1 = 0x0,					//< Each value is averaged 1 times
	ICM20948_ACCEL_AVEG_4 = 0x4,					//< Each value is averaged 1 times
	ICM20948_ACCEL_AVEG_8 = 0x1,					//< Each value is averaged 1 times
	ICM20948_ACCEL_AVEG_16 = 0x2,					//< Each value is averaged 1 times
	ICM20948_ACCEL_AVEG_32 = 0x3,					//< Each value is averaged 1 times
} icm20948_accel_averaging_e;

//Gyroscope data structure
typedef struct {

	//Sensor activation configuration
	bool enabled;

	//Frequency of measures following the formula freq = 1100Hz/(1+rateDiv)
	uint8_t rateDiv;

	//Data range configuration
	icm20948_gyro_range_e range;

	//Data low pass filter configuration
	bool enabledDLPF;
	icm20948_gyro_cutoff_freq_e cutoffFreqDLPF;

	//Data averaging configuration
	icm20948_gyro_averaging_e averaging;

	//Data
	rawData_t rawData;
	scaledData_t scaledData;
	filteredData_t filteredData;

	//Filters congiguration

} droneGyroscope_t;

//Accelerometer data structure
typedef struct {

	//Sensor activation configuration
	bool enabled;

	//Frequency of measures following the formula freq = 1125Hz/(1+rateDiv)
	uint8_t rateDiv;

	//Data range configuration
	icm20948_accel_range_e range;

	//Data low pass filter configuration
	bool enabledDLPF;
	icm20948_accel_cutoff_freq_e cutoffFreqDLPF;

	//Data averaging configuration
	icm20948_accel_averaging_e averaging;

	//Data
	rawData_t rawData;
	scaledData_t scaledData;
	filteredData_t filteredData;

	//Filters configuration

} droneAccelerometer_t;

//Temperature data structure
typedef struct {

	//Sensor activation configuration
	bool enabled;

	//Data low pass filter configuration
	icm20948_temp_cutoff_freq_e cutoffFreqDLPF;

	//Data
	int16_t rawData;
	double scaledData;

} droneTemperature_t;

//Magnetometer data structure
typedef struct {

	//Sensor activation configuration
	bool enabled;

	//Frequency of measures
	ak09916_mag_rate_div_e rateDiv;

	//Data
	rawData_t rawData;
	scaledData_t scaledData;
	filteredData_t filteredData;

	//Filters

} droneMagnetometer_t;

//Gyroscope modes
typedef enum {
	ICM20948_GYRO_RESET_MODE = 0x0,					//< This mode prepares the gyroscope to be resetted
	ICM20948_GYRO_SLEEP_MODE = 0x1,					//< This mode turns off the gyroscope
	ICM20948_GYRO_CALIB_MODE = 0x2,					//< This mode prepares the gyroscope to be calibrated
	ICM20948_GYRO_STANDARD_MODE = 0x3,				//< This is the standard mode for the gyroscope
	ICM20948_GYRO_HIGH_ACCURACY_MODE = 0x4,			//< This the high accuracy mode for the gyroscope
	ICM20948_GYRO_DMP_TESTING_MODE = 0x5,			//< 
} icm20948_gyro_mode_e;
typedef icm20948_gyro_mode_e icm20948_gyro_mode_t;

//Accelerometer modes
typedef enum {
	ICM20948_ACCEL_RESET_MODE = 0x0,				//< This mode prepares the accelerometer to be resetted
	ICM20948_ACCEL_SLEEP_MODE = 0x1,				//< This mode turns off the accelerometer
	ICM20948_ACCEL_CALIB_MODE = 0x2,				//< This mode prepares the accelerometer to be calibrated
	ICM20948_ACCEL_STANDARD_MODE = 0x3,				//< This is the standard mode for the accelerometer
	ICM20948_ACCEL_DMP_TESTING_MODE = 0x4,			//< 
} icm20948_accel_mode_e;
typedef icm20948_accel_mode_e icm20948_accel_mode_t;

//Temperature sensor modes
typedef enum {
	ICM20948_TEMP_RESET_MODE = 0x0,					//< This mode prepares the temperature sensor to be resetted
	ICM20948_TEMP_SLEEP_MODE = 0x1,					//< This mode turns off the temperature sensor
	ICM20948_TEMP_CALIB_MODE = 0x2,					//< This mode prepares the temperature sensor to be calibrated
	ICM20948_TEMP_STANDARD_MODE = 0x3,				//< This is the standard mode for the temperature sensor
} icm20948_temp_mode_e;
typedef icm20948_temp_mode_e icm20948_temp_mode_t;

//Magnetometer modes
typedef enum {
	ICM20948_MAG_RESET_MODE = 0x0,					//< This mode prepares the magnetometer to be resetted
	ICM20948_MAG_SLEEP_MODE = 0x1,					//< This mode turns off the magnetometer
	ICM20948_MAG_CALIB_MODE = 0x2,					//< This mode prepares the magnetometer to be calibrated
	ICM20948_MAG_STANDARD_MODE = 0x3,				//< This is the standard mode for the magnetometer
} icm20948_mag_mode_e;
typedef icm20948_mag_mode_e icm20948_mag_mode_t;

class DroneICM20948{
	public:

		//Public begin functions
		bool begin();

		//Public reset functions
		bool dynamicReset();

		//Public sensor calibration
		bool calibrateSensors(double gravity);

		//Sensor data updating
		bool updateSensors(double localGroundGravity, dataToObtain_e dataToObtain, bool scaleData = true);

		//Sensors mode configuration
		bool setGyroMode(icm20948_gyro_mode_e newMode);
		bool setAccelMode(icm20948_accel_mode_e newMode);
		bool setMagMode(icm20948_mag_mode_e newMode);
		bool setTempMode(icm20948_temp_mode_e newMode);

		///--- RAW DATA OBTENTION ---///

		//Raw data getters
		rawData_t getRawGyroscopeData() {return droneGyroscope.rawData; } //Digitalized 16bit analog signal
		rawData_t getRawAccelerometerData() {return droneAccelerometer.rawData; } //Digitalized 16bit analog signal
		rawData_t getRawMagnetometerData() {return droneMagnetometer.rawData; } //Digitalized 16bit analog signal
		int16_t getRawTemperature() {return droneTemperature.rawData; } //Digitalized 16bit analog signal

		//Scaled data getters
		scaledData_t getScaledGyroscopeData() {return droneGyroscope.scaledData; } // rad/s
		scaledData_t getScaledAccelerometerData() {return droneAccelerometer.scaledData; } // m/s^2
		scaledData_t getScaledMagnetometerData() {return droneMagnetometer.scaledData; } // gauss
		double getScaledTemperature() {return droneTemperature.scaledData; } // Cº

		//Filtered data getters
		filteredData_t getFilteredGyroscopeData() {return droneGyroscope.filteredData; } // rad/s
		filteredData_t getFilteredAccelerometerData() {return droneAccelerometer.filteredData; } // m/s^2
		filteredData_t getFilteredMagnetometerData() {return droneMagnetometer.filteredData; } // gauss

		///--- DMP DATA OBTENTION ---///

		//DMP data getters
		void getQuaternionDMP();
		void getGyroscopeDMP();
		void getAccelerometerDMP();
		void getMagnetometerDMP();

		//Execution time getter
		timeUs_t getLastExecutionTime() {return lastExecutionTime; }

	private:

		//Private begin functions
		bool beginMagnetometer();
		void beginDMP();

		//Private reset functions
		bool reset();
		bool resetMagnetometer();

		//Configuring the DMP
		bool loadFirmwareDMP(const uint8_t *data_start, uint16_t size_start);
		void applyMountingMatrix();
		void setB2SMatrix(int *b2sMtx);

		//Private data interaction
		void read3AxisAveragedSensor(uint16_t regAddr, int16_t *Xread, int16_t *Yread, int16_t *Zread);
		bool checkAndProcessPacketDMP();
		void calculateOffsets(double gravity);

		//Sensor self tests
		bool gyroSelfTest();
		bool accelSelfTest();
		bool magSelfTest();

		//Sensor data scalation
		void scaleSensorsData(double gravity);

		//Move functions to DSPI, TODO when moved onto DSPI, make bool returns useful
		uint8_t transfer(uint8_t send);
		void writeICM20948Register(uint16_t regAddr, uint8_t regAddrLen, const uint8_t *writeBuff, uint8_t writeBuffLen, bool adaptAddress = true);
		void writeICM20948Register(uint16_t regAddr, uint8_t regAddrLen, uint32_t writeBuff, uint8_t writeBuffLen, uint8_t byteorder = LSBFIRST);
		void readICM20948Register(uint16_t regAddr, uint8_t regAddrLen, uint8_t *readBuff, uint8_t readBuffLen, uint8_t sendValue = 0xFF);
		void writeICM20948RegisterBits(uint16_t regAddr, uint8_t regAddrLen, uint32_t writeBuff, uint8_t writeBuffLen, uint8_t bits, uint8_t shift, uint8_t byteorder = LSBFIRST);
		uint32_t readICM20948RegisterBits(uint16_t regAddr, uint8_t regAddrLen, uint8_t readBuffLen, uint8_t bits, uint8_t shift, uint8_t byteorder = LSBFIRST, uint8_t sendValue = 0xFF);
		//void writeBytes();
		//bool writeMemoryBlock();

		//Magnetometer comunication functions
		uint8_t auxillaryRegisterTransaction(bool read, uint8_t slvAddr, uint8_t regAddr, uint8_t value = -1);
		bool writeExternalRegister(uint8_t slvAddr, uint8_t regAddr, uint8_t value);
		bool writeMagRegister(uint8_t magRegAddr, uint8_t value);
		uint8_t readExternalRegister(uint8_t slvAddr, uint8_t regAddr);
		uint8_t readMagRegister(uint8_t magRegAddr);
		bool writeMagRegisterBits(uint8_t magRegAddr, uint8_t value, uint8_t bits, uint8_t shift);
		uint8_t readMagRegisterBits(uint8_t magRegAddr, uint8_t bits, uint8_t shift);

		//DMP comunication functions
		bool writeDMPMemory(uint16_t address, uint16_t bytesDataLength, const uint8_t *data);
		bool readDMPMemory(uint16_t address, uint16_t bytesDataLength, uint8_t *data);
		/*bool writeDMPRegister(uint16_t address, uint16_t bytesDataLength, const uint8_t *data);
		bool readDMPRegister(uint16_t address, uint16_t bytesDataLength, uint8_t *data);*/

		//ICM20948 specific comunication functions
		void adaptRegAddress(uint16_t address, RegType_e regType);
		uint32_t convertTo32(uint8_t *data, uint8_t width, uint8_t byteorder = LSBFIRST); //TODO, move to utils
		void setRegBank(uint8_t bankNumber, bool mandatory = false);
		void setDMPPageBank(uint8_t bankNumber, bool mandatory = false);

		//FIFO readings functions
		/*uint8_t getFIFOCount();
		void getFIFOBytes(uint8_t *data, uint8_t length);*/

		//Data conversion functions
		uint8_t *convert16to8(int16_t x, uint8_t *big8);
		uint8_t *convert32to8(int32_t x, uint8_t *big8);

		//DMP calculation functions
		void setChipToBodyAxisQuaternion(float rotationAngle);
		void rotationToQuaternion(float Rcb[9], long Qcb[4]);
		void convertMatrixToQuatFLT(float *R, float *q);
		void convertQuatToMultFXP(const long *quat1_q30, const long *quat2_q30, long *quatProd_q30);
		long convertMultToQ30FXP(long a_q30, long b_q30);

		//Indicates if the chip can be used
		bool chipInitialized;

		//TODO, move to DSPI
		SPISettings spiSettings;

		//Sensor data structures
		droneGyroscope_t droneGyroscope;
		droneAccelerometer_t droneAccelerometer;
		droneMagnetometer_t droneMagnetometer;
		droneTemperature_t droneTemperature;

		//Actual mode of the gyroscope
		icm20948_gyro_mode_t gyroMode;

		//Gyroscope low pass filter
		lowPassFilter_t gyroLPF;

		//Actual mode of the accelerometer
		icm20948_accel_mode_t accelMode;

		//Accelerometer low pass filter
		lowPassFilter_t accelLPF;

		//Actual mode of the magnetometer
		icm20948_mag_mode_t magMode;

		//Magnetometer low pass filter
		lowPassFilter_t magLPF;

		//Actual mode of the temperature
		icm20948_temp_mode_t tempMode;

		//Self test data obtained
		uint8_t selfTestBase[6];

		//Buffer for saving data to be sent or recived
		uint8_t _buffer[4];

		//Buffer for saving the address addapted from a 16 bit address
		uint8_t _addReg[2];

		//The register bank that the sensor is selected to be in
		uint8_t regBank;

		//The DMP page bank that the sensor is selected to be in
		uint8_t pageBank;

		//Offsets calculated when the sensor is still
		int16_t gyroOffsetX, gyroOffsetY, gyroOffsetZ;
		int16_t accelOffsetX, accelOffsetY, accelOffsetZ;
		int16_t magOffsetX, magOffsetY, magOffsetZ;

		//Sacle of the data being recived by the sensor
		double accelScale;
		double gyroScale;

		//Correction of the scale between the scaled used in the calibration and in the measurements, TODO check if it is necessary
		double gyroCorrectionScale;
		double accelCorrectionScale;

		//Mounting matrix
		signed char mountingMatrix[9];

		//Data conversion
		long quatChipToBody[4];

		//Execution and internal times
		timeUs_t lastExecutionTime;
		timeUs_t internalTime;
};
extern DroneICM20948 DICM20948;
#endif