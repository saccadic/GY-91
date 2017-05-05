#include <Wire.h>
#include <EEPROM.h>
#include <I2Cdev.h>
#include "helper_3dmath.h"
#include "quaternionFilters.h"

extern "C" {
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
}

//Adress
#define MPU9250_ADDRESS				0x68  // MPU9250 address when ADO = 1
#define AK8963_ADDRESS				0x0C  
#define AK8963_CNTL					0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASAX					0x10  // Fuse ROM x-axis sensitivity adjustment value
#define MPU9250_XG_OFFSET_H			0x13  // User-defined trim values for gyroscope
#define MPU9250_XG_OFFSET_L			0x14
#define MPU9250_YG_OFFSET_H			0x15
#define MPU9250_YG_OFFSET_L			0x16
#define MPU9250_ZG_OFFSET_H			0x17
#define MPU9250_ZG_OFFSET_L			0x18
#define MPU9250_XA_OFFSET_H			0x77
#define MPU9250_XA_OFFSET_L			0x78
#define MPU9250_YA_OFFSET_H			0x7A
#define MPU9250_YA_OFFSET_L			0x7B
#define MPU9250_ZA_OFFSET_H			0x7D
#define MPU9250_ZA_OFFSET_L			0x7E

//Sensor options
#define GYRO_250DPS  250
#define GYRO_500DPS  500
#define GYRO_1000DPS 1000
#define GYRO_2000DPS 2000
#define Accel_2G 2
#define Accel_4G 4
#define Accel_8G 8
#define Accel_16G 16

enum lpf_e {
	INV_FILTER_256HZ_NOLPF2 = 0,
	INV_FILTER_188HZ,
	INV_FILTER_98HZ,
	INV_FILTER_42HZ,
	INV_FILTER_20HZ,
	INV_FILTER_10HZ,
	INV_FILTER_5HZ,
	INV_FILTER_2100HZ_NOLPF,
	NUM_FILTER
};

#define POWER_ON 1
#define POWER_OFF 2
#define MPU_RATE 1000	//MAX:1000
#define DMP_RATE 200	//MAX:200
#define MAG_RATE 100	//MAX:100
#define DMP_ON 1
#define DMP_OFF 0

//Setting options
#define SERIAL_SPEED 230400

enum {
	DMP,		//Digital Motion Processor //OK
	MADGWICK,	//MadgwickQuaternionUpdate //OK
	MAHONY,		//MahonyQuaternionUpdate
};

enum {
	IDOLE,
	RAWDATA,
	Euler_ANGLE,
	PYR_ANGLE,
	QUATERNION,
};

//##########################################
//Option
int processEngine						= MADGWICK;
int updateMode							= QUATERNION;
int CalibrationMode						= -1;
const int GyroScale						= GYRO_2000DPS;
const int AccelScale					= Accel_2G;
const int lpf_rate						= INV_FILTER_188HZ;
int filterNum							= 0.9f; //0.0f �` 1.0fs
const float magneticRes					= 10.0f * 1229.0f / 4096.0f; // scale  milliGauss
const unsigned short gyrosensitivity	= 131;  // = 131 LSB/degrees/sec
const unsigned short accelsensitivity	= 16384;// = 16384 LSB/g
//##########################################

//FIFO
int fifoResult;
long unsigned int sensor_timestamp;
short sensors;
unsigned char fifoCount = 0;

//Raw data
short gyro[3];
short accel[3];
short commpass[3];
long quat[4];

//Calibration data
struct CalibrationData {
	float destination[3]	= { 0, 0, 0 };
	float gyroBias[3]		= { 0, 0, 0 };
	float accelBias[3]		= { 0, 0, 0 };
	float magBias[3]		= { 0, 0, 0 };
};
CalibrationData CalibrationSaveData;

//Mathematics data
Quaternion rawQuaternion;
Quaternion quaternion;
Quaternion Temp_quaternion;
VectorFloat angle;
VectorFloat tempAngle;

//Quaternion filter
float GyroMeasError = PI * (40.0f / 180.0f);		// gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f / 180.0f);			// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;		// compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;		// compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float Kp = 2.0f * 5.0f;								// these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
float Ki = 0.0f;
long delt_t = 0;
float deltat = 0.0f;
long lastUpdate = 0, firstUpdate = 0;			// used to calculate integration interval
long Now = 0;									// used to calculate integration interval
float eInt[3] = { 0.0f, 0.0f, 0.0f };				// vector to hold integral error for Mahony method

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
									   //	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	//        Wire.requestFrom(address, count);  // Read bytes from slave register address
	Wire.requestFrom(address, (size_t)count);  // Read bytes from slave register address
	while (Wire.available()) {
		dest[i++] = Wire.read();
	}         // Put read results in the Rx buffer
}
unsigned short inv_row_2_scale(const signed char *row)
{
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;      // error
	return b;
}
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
	unsigned short scalar;
	/*	XYZ  010_001_000 Identity Matrix
		XZY  001_010_000
		YXZ  010_000_001
		YZX  000_010_001
		ZXY  001_000_010
		ZYX  000_001_010				*/
	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;
	return scalar;
}
void GetYawPitchRoll(VectorFloat *ypr, Quaternion *q) {
	// roll: (tilt left/right, about X axis)
	ypr->x = atan2(2.0f * (q->x * q->y + q->w * q->z), q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z);
	// pitch: (nose up/down, about Y axis)
	ypr->y = atan2(2.0f * (q->y * q->z + q->w * q->x), q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z);
	// yaw: (about Z axis)
	ypr->z = asin((2.0f * (q->w * q->y)) - (2.0f * (q->x * q->z)));

	//http://vldb.gsi.go.jp/sokuchi/geomag/menu_04/index.html

	ypr->z *= 180.0f / PI;
	ypr->y *= 180.0f / PI; //- 8.35f; // Declination at Danville, Morioka is 8 degrees 35 minutes on 2017-04-29;
	ypr->x *= 180.0f / PI;
}
void GetEulerAngle(VectorFloat *angle, Quaternion* q)
{
	double ysqr = q->y * q->y;

	// roll (x-axis rotation)
	double t0 = +2.0 * (q->w * q->x + q->y * q->z);
	double t1 = +1.0 - 2.0 * (q->x * q->x + ysqr);
	angle->x = atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q->w * q->y - q->z * q->x);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	angle->y = asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q->w * q->z + q->x * q->y);
	double t4 = +1.0 - 2.0 * (ysqr + q->z * q->z);
	angle->z = atan2(t3, t4);

	angle->x *= 180.0f / PI;
	angle->y *= 180.0f / PI;
	angle->z *= 180.0f / PI;
}
void accelgyrocalMPU9250(float * dest1, float * dest2) {
	unsigned int BufferSize = 1000;
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data

	long  gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	int update = 1;
	while (update) {
		short accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };

		mpu_get_gyro_reg(gyro, &sensor_timestamp);
		gyro_temp[0] = gyro[0];
		gyro_temp[1] = gyro[1];
		gyro_temp[2] = gyro[2];

		mpu_get_accel_reg(accel, &sensor_timestamp);
		accel_temp[0] = accel[0];
		accel_temp[1] = accel[1];
		accel_temp[2] = accel[2];

		gyro_bias[0] += (long)gyro_temp[0];
		gyro_bias[1] += (long)gyro_temp[1];
		gyro_bias[2] += (long)gyro_temp[2];

		accel_bias[0] += (long)accel_temp[0];
		accel_bias[1] += (long)accel_temp[1];
		accel_bias[2] += (long)accel_temp[2];

		Serial.print(".");

		if (Serial.available() > 0) {
			char inputchar = Serial.read();
			if (inputchar == 's')
				update = 0;
		}
		else {
			BufferSize++;
		}
	}

	Serial.println("");

	gyro_bias[0] /= (long)BufferSize;
	gyro_bias[1] /= (long)BufferSize;
	gyro_bias[2] /= (long)BufferSize;
	accel_bias[0] /= (long)BufferSize;
	accel_bias[1] /= (long)BufferSize;
	accel_bias[2] /= (long)BufferSize;

	Serial.print("gx_ave : ");	Serial.println(gyro_bias[0]);
	Serial.print("gy_ave : ");	Serial.println(gyro_bias[1]);
	Serial.print("gz_ave : ");	Serial.println(gyro_bias[2]);
	Serial.print("ax_ave : ");	Serial.println(accel_bias[0]);
	Serial.print("ay_ave : ");	Serial.println(accel_bias[1]);
	Serial.print("az_ave : ");	Serial.println(accel_bias[2]);

	if (accel_bias[2] > 0L)
		accel_bias[2] -= (long)accelsensitivity;
	else
		accel_bias[2] += (long)accelsensitivity;

	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF;		// Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

	writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_L, data[5]);

	dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
	dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
	dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

	long accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	readBytes(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (long)(((short)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (long)(((short)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (long)(((short)data[0] << 8) | data[1]);

	accel_bias_reg[0] -= (accel_bias[0] / AccelScale); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / AccelScale);
	accel_bias_reg[2] -= (accel_bias[2] / AccelScale);

	dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
	dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
	dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}
void magcalMPU9250(float * magCalibration, float * dest1)
{
	long mag_bias[3] = { 0, 0, 0 };
	short mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };

	int update = 1;

	while (update) {
		mpu_get_compass_reg(mag_temp, &sensor_timestamp);  // Read the mag data
		for (int j = 0; j < 3; j++) {
			if (mag_temp[j] > mag_max[j]) mag_max[j] = mag_temp[j];
			if (mag_temp[j] < mag_min[j]) mag_min[j] = mag_temp[j];
		}
		Serial.print("mag_max_x  : ");	Serial.println(mag_max[0]);
		Serial.print("mag_min_x  : ");	Serial.println(mag_min[0]);
		Serial.print("mag_max_y : ");	Serial.println(mag_max[1]);
		Serial.print("mag_min_y  : ");	Serial.println(mag_min[1]);
		Serial.print("mag_max_z  : ");	Serial.println(mag_max[2]);
		Serial.print("mag_min_z  : ");	Serial.println(mag_min[2]);

		if (Serial.available() > 0) {
			char inputchar = Serial.read();
			if (inputchar == 's')
				update = 0;
		}

		delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
	}

	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

	dest1[0] = (float)mag_bias[0] * magneticRes*magCalibration[0];  // save mag biases in G for main program
	dest1[1] = (float)mag_bias[1] * magneticRes*magCalibration[1];
	dest1[2] = (float)mag_bias[2] * magneticRes*magCalibration[2];
}
void GetCompassSelfTest(float * destination) {
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	delay(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	destination[0] = (float)(rawData[0] - 128) / 256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] = (float)(rawData[1] - 128) / 256. + 1.;
	destination[2] = (float)(rawData[2] - 128) / 256. + 1.;
	delay(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 1 << 4 | 1);
	delay(10);

	Serial.print("destination_x  : ");	Serial.println(destination[0]);
	Serial.print("destination_y  : ");	Serial.println(destination[1]);
	Serial.print("destination_z : ");	Serial.println(destination[2]);
}
void SaveCalibration(struct CalibrationData *calibrationData) {
	Serial.println("Save saveCalibration");
	Serial.print("EEPROM");
	byte* p = (byte*)calibrationData;
	for (int j = 0; j < sizeof(CalibrationData); j++) {
		EEPROM.write(j, *p); p++;
		Serial.print("<");
	}
}
void LoadCalibration(struct CalibrationData *calibrationData) {
	Serial.println("Load saveCalibration");
	Serial.print("EEPROM");
	byte* p2 = (byte*)calibrationData;
	for (int j = 0; j < sizeof(CalibrationData); j++) {
		byte b = EEPROM.read(j); *p2 = b; p2++;
		Serial.print(">");
	}
	Serial.println("Done !");

	uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
	long  gyro_bias[3] = { 0, 0, 0 };

	gyro_bias[0] = calibrationData->gyroBias[0] * (float)gyrosensitivity;
	gyro_bias[1] = calibrationData->gyroBias[1] * (float)gyrosensitivity;
	gyro_bias[2] = calibrationData->gyroBias[2] * (float)gyrosensitivity;

	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF;		// Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;
	writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_L, data[5]);

	Serial.print("gyro_bias\t: ");	Serial.println(CalibrationSaveData.gyroBias[0]);
	Serial.print("gyro_bias\t: ");	Serial.println(CalibrationSaveData.gyroBias[1]);
	Serial.print("gyro_bias\t: ");	Serial.println(CalibrationSaveData.gyroBias[2]);
	Serial.print("accel_bias\t: ");	Serial.println(CalibrationSaveData.accelBias[0]);
	Serial.print("accel_bias\t: ");	Serial.println(CalibrationSaveData.accelBias[1]);
	Serial.print("accel_bias\t: ");	Serial.println(CalibrationSaveData.accelBias[2]);
	Serial.print("Compass_bias\t: ");	Serial.println(CalibrationSaveData.magBias[0]);
	Serial.print("Compass_bias\t: ");	Serial.println(CalibrationSaveData.magBias[1]);
	Serial.print("Compass_bias\t: ");	Serial.println(CalibrationSaveData.magBias[2]);
}
int InitMPU9250_Debug() {

	Serial.println("-----MPU-----");

	if (mpu_init(NULL) != 0)
	{
		Serial.println("mpu_init failed!");
		return -1;
	}
	else
	{
		Serial.println("Initialize sensor");

		//Select using sensor
		if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS) != 0) {
			Serial.println("mpu_set_sensors failed!");
			return -1;
		}
		else {
			Serial.println("Activate gyro, accel and compass sensor.");
		}

		//Set Scale
		if (mpu_set_gyro_fsr(GyroScale) != 0) {
			Serial.println("Failure set gyro scale.");
			return -1;
		}
		else {
			Serial.println("Set gyro scale : " + String(GyroScale) + "DPS");
		}
		if (mpu_set_accel_fsr(AccelScale) != 0) {
			Serial.println("Failure set Accel scale.");
			return -1;
		}
		else {
			Serial.println("Set accel scale : " + String(AccelScale) + "G");
		}

		////Set Update rate
		if (mpu_set_sample_rate(MPU_RATE) != 0) {
			Serial.println("mpu_set_sample_rate() failed");
			return -1;
		}
		else {
			Serial.println("Set MPU sample rate : " + String(MPU_RATE) + "hz");
		}

		if (mpu_set_compass_sample_rate(MAG_RATE) != 0) {
			Serial.println("mpu_set_compass_sample_rate() failed");
			return -1;
		}
		else {
			Serial.println("Set compass sample rate : " + String(MAG_RATE) + "hz");
		}

		////Set Filter
		if (mpu_set_lpf(lpf_rate) != 0) {
			Serial.println("mpu_set_lpf() failed");
			return -1;
		}
		else {
			int ipfRate;

			if (lpf_rate >= 188)
				ipfRate = 188;
			else if (lpf_rate >= 98)
				ipfRate = 98;
			else if (lpf_rate >= 42)
				ipfRate = 42;
			else if (lpf_rate >= 20)
				ipfRate = 20;
			else if (lpf_rate >= 10)
				ipfRate = 10;
			else
				ipfRate = 5;

			Serial.println("Set lpf rate : " + String(ipfRate) + "hz");
		}

		//FIFO setting
		if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
			printf("Failed to initialize MPU fifo!\n");
			return -1;
		}
		else {
			Serial.println("Active fifo gyro, accel sensor.");
		}

		// verify connection
		unsigned char devStatus;
		Serial.println("Powering up MPU...");
		mpu_get_power_state(&devStatus);
		if (devStatus)
			Serial.println("> MPU6050 connection successful\n");
		else
			Serial.println("> MPU6050 connection failed %u\n");

		//DMP Setup
		Serial.println("-----DMP------");

		if (dmp_load_motion_driver_firmware() != 0) {
			Serial.println("Failure load motion driver.");
			return -1;
		}
		else {
			Serial.println("Load motion driver.");
		}

		if (mpu_set_dmp_state(DMP_ON) != 0) {
			Serial.println("Stop DMP.");
			return -1;
		}
		else {
			Serial.println("> Digital Motion Processor active.\n");
		}

		signed char gyro_orientation[9] = {
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
		};

		if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)) != 0) {
			Serial.println("dmp_set_orientation() failed");
			return -1;
		}
		else {
			Serial.println("dmp_set_orientation complete.");
		}

		if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL) != 0) {
			Serial.println("Failure DMP features.");
			return -1;
		}
		else {
			Serial.println("Configuring DMP setting.");
		}

		if (dmp_set_fifo_rate(DMP_RATE) != 0) {
			Serial.println("Failure set fifo rate.");
			return -1;
		}
		else {
			Serial.println("Set DMP rate : " + String(DMP_RATE) + "hz");
		}

		if (mpu_reset_fifo() != 0) {
			printf("Failed mpu_reset_fifo!\n");
			return -1;
		}
		else {
			Serial.println("Reset fifo");
		}

		Serial.println("-----TEST------");
		Serial.print("Checking...");
		do {
			delay(100);  //dmp will habve 4 (5-1) packets based on the fifo_rate
			fifoResult = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &fifoCount);
			Serial.print(".");
		} while (fifoResult != 0 || fifoCount < 5);

		Serial.println("\nMPU9250 Factory calibration data:");
		long factoryGyroData[3], factoryAccelData[3];
		mpu_run_self_test(factoryGyroData, factoryAccelData);
		Serial.print("gx : ");	Serial.print(factoryGyroData[0], 1);	Serial.println("%");
		Serial.print("gy : ");	Serial.print(factoryGyroData[1], 1);	Serial.println("%");
		Serial.print("gz : ");	Serial.print(factoryGyroData[2], 1);	Serial.println("%");
		Serial.print("ax : ");	Serial.print(factoryAccelData[0], 1);	Serial.println("%");
		Serial.print("ay : ");	Serial.print(factoryAccelData[1], 1);	Serial.println("%");
		Serial.print("az : ");	Serial.print(factoryAccelData[2], 1);	Serial.println("%");

		Serial.println("\n-----MPU9250-ready-!!-----\n");
	}
}
int InitMPU9250() {
	if (mpu_init(NULL) != 0)
	{
		return -1;
	}
	else
	{
		//Select using sensor
		if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS) != 0)
			return -1;

		//Set Scale
		if (mpu_set_gyro_fsr(GyroScale) != 0)
			return -1;

		if (mpu_set_accel_fsr(AccelScale) != 0)
			return -1;

		////Set Update rate
		if (mpu_set_sample_rate(MPU_RATE) != 0)
			return -1;

		if (mpu_set_compass_sample_rate(MAG_RATE) != 0)
			return -1;

		////Set Filter
		if (mpu_set_lpf(lpf_rate) != 0)
			return -1;

		//FIFO setting
		if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0)
			return -1;


		if (dmp_load_motion_driver_firmware() != 0)
			return -1;

		if (mpu_set_dmp_state(DMP_ON) != 0)
			return -1;


		signed char gyro_orientation[9] = {
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
		};

		if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)) != 0)
			return -1;

		if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL) != 0)
			return -1;

		if (dmp_set_fifo_rate(DMP_RATE) != 0)
			return -1;

		if (mpu_reset_fifo() != 0)
			return -1;
	}
}

void setup() {
	// put your setup code here, to run once:
	Wire.begin();
	Serial.begin(SERIAL_SPEED);
	Serial.println("\nInitializing MPU...");
	if (InitMPU9250()) {
		Serial.println("mpu init !");
		LoadCalibration(&CalibrationSaveData);
	}
}

void loop() {
	if (CalibrationMode == 0)
	{
		accelgyrocalMPU9250(CalibrationSaveData.gyroBias, CalibrationSaveData.accelBias);
		GetCompassSelfTest(CalibrationSaveData.destination);
		magcalMPU9250(CalibrationSaveData.destination, CalibrationSaveData.magBias);

		Serial.print("gyro_bias    : ");	Serial.println(CalibrationSaveData.gyroBias[0]);
		Serial.print("gyro_bias    : ");	Serial.println(CalibrationSaveData.gyroBias[1]);
		Serial.print("gyro_bias    : ");	Serial.println(CalibrationSaveData.gyroBias[2]);
		Serial.print("accel_bias   : ");	Serial.println(CalibrationSaveData.accelBias[0]);
		Serial.print("accel_bias   : ");	Serial.println(CalibrationSaveData.accelBias[1]);
		Serial.print("accel_bias   : ");	Serial.println(CalibrationSaveData.accelBias[2]);
		Serial.print("Compass_bias : ");	Serial.println(CalibrationSaveData.magBias[0]);
		Serial.print("Compass_bias : ");	Serial.println(CalibrationSaveData.magBias[1]);
		Serial.print("Compass_bias : ");	Serial.println(CalibrationSaveData.magBias[2]);

		CalibrationMode = -1;
	}
	else
	{
		//Command
		if (Serial.available() > 0) {
			char inputchar = Serial.read();
			switch (inputchar) {
			case 'o':
				tempAngle = angle;
				break;
			case 'c':
				CalibrationMode = 0;
				break;
			case 's':
				SaveCalibration(&CalibrationSaveData);
				break;
			}
		}

		//Update data
		switch (processEngine) {
		case DMP:
		{
			do { dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &fifoCount); } while (fifoCount > 1);

			rawQuaternion = Quaternion(
				(float)(quat[0] / 1073741824.f), //long to float
				(float)(quat[1] / 1073741824.f),
				(float)(quat[2] / 1073741824.f),
				(float)(quat[3] / 1073741824.f)
			);
		}
		break;
		case MADGWICK:
		{
			float ax, ay, az; //�����x
			float gx, gy, gz; //�p���x
			float mx, my, mz; //�n���C

			Now = micros();
			deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
			lastUpdate = Now;

			mpu_get_gyro_reg(gyro, &sensor_timestamp);
			gx = (float)gyro[0] * (float)GyroScale / 32768.0f;
			gy = (float)gyro[1] * (float)GyroScale / 32768.0f;
			gz = (float)gyro[2] * (float)GyroScale / 32768.0f;

			mpu_get_accel_reg(accel, &sensor_timestamp);
			ax = ((float)accel[0] * (float)AccelScale - CalibrationSaveData.accelBias[0]);
			ay = ((float)accel[1] * (float)AccelScale - CalibrationSaveData.accelBias[1]);
			az = ((float)accel[2] * (float)AccelScale - CalibrationSaveData.accelBias[2]);

			mpu_get_compass_reg(commpass, &sensor_timestamp);
			mx = (float)commpass[0] * magneticRes * CalibrationSaveData.destination[0] - CalibrationSaveData.magBias[0];
			my = (float)commpass[1] * magneticRes * CalibrationSaveData.destination[1] - CalibrationSaveData.magBias[1];
			mz = (float)commpass[2] * magneticRes * CalibrationSaveData.destination[2] - CalibrationSaveData.magBias[2];

			MadgwickQuaternionUpdate(
				&rawQuaternion,
				beta,
				zeta,
				deltat,
				ax,
				ay,
				az,
				gx * PI / 180.0f,
				gy * PI / 180.0f,
				gz * PI / 180.0f,
				my,
				mx,
				-mz
			);
		}
		break;
		case MAHONY:
		{
			float ax, ay, az; //�����x
			float gx, gy, gz; //�p���x
			float mx, my, mz; //�n���C

			Now = micros();
			deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
			lastUpdate = Now;

			mpu_get_gyro_reg(gyro, &sensor_timestamp);
			gx = (float)gyro[0] * (float)GyroScale / 32768.0f;
			gy = (float)gyro[1] * (float)GyroScale / 32768.0f;
			gz = (float)gyro[2] * (float)GyroScale / 32768.0f;

			mpu_get_accel_reg(accel, &sensor_timestamp);
			ax = ((float)accel[0] * (float)AccelScale - CalibrationSaveData.accelBias[0]) / 32768.0f;
			ay = ((float)accel[1] * (float)AccelScale - CalibrationSaveData.accelBias[1]) / 32768.0f;
			az = ((float)accel[2] * (float)AccelScale - CalibrationSaveData.accelBias[2]) / 32768.0f;

			mpu_get_compass_reg(commpass, &sensor_timestamp);
			mx = (float)commpass[0] * magneticRes * CalibrationSaveData.destination[0] - CalibrationSaveData.magBias[0];
			my = (float)commpass[1] * magneticRes * CalibrationSaveData.destination[1] - CalibrationSaveData.magBias[1];
			mz = (float)commpass[2] * magneticRes * CalibrationSaveData.destination[2] - CalibrationSaveData.magBias[2];

			//-ax, ay, az, gx*pi/180.0f, -gy*pi/180.0f, -gz*pi/180.0f,  my,  -mx, mz
			MahonyQuaternionUpdate(
				&rawQuaternion,
				eInt,
				Ki,
				Kp,
				deltat,
				ax,
				ay,
				az,
				gx * PI / 180.0f,
				gy * PI / 180.0f,
				gz * PI / 180.0f,
				my,
				mx,
				-mz
			);
		}
		break;
		}

		//Filter RC�t�B���^
		if (filterNum > 1.0f) { filterNum = 1.0f; }
		if (filterNum < 0.0f) { filterNum = 0.0f; }
		quaternion.x = filterNum * Temp_quaternion.x - (1.0f - filterNum) * rawQuaternion.x;
		quaternion.y = filterNum * Temp_quaternion.y - (1.0f - filterNum) * rawQuaternion.y;
		quaternion.z = filterNum * Temp_quaternion.z - (1.0f - filterNum) * rawQuaternion.z;
		quaternion.w = filterNum * Temp_quaternion.w - (1.0f - filterNum) * rawQuaternion.w;
		Temp_quaternion = quaternion;

		//Output
		switch (updateMode) {
		case RAWDATA:
		{
			float ax, ay, az; //�����x
			float gx, gy, gz; //�p���x
			float mx, my, mz; //�n���C

			mpu_get_gyro_reg(gyro, &sensor_timestamp);
			gx = gyro[0] * (float)GyroScale / 32768.0f;
			gy = gyro[1] * (float)GyroScale / 32768.0f;
			gz = gyro[2] * (float)GyroScale / 32768.0f;

			mpu_get_accel_reg(accel, &sensor_timestamp);
			ax = accel[0] * (float)AccelScale / 32768.0f;
			ay = accel[1] * (float)AccelScale / 32768.0f;
			az = accel[2] * (float)AccelScale / 32768.0f;

			mpu_get_compass_reg(commpass, &sensor_timestamp);
			mx = commpass[0] * 10.0f * 1229.0f / 4096.0f + 18.0f;
			my = commpass[1] * 10.0f * 1229.0f / 4096.0f + 70.0f;
			mz = commpass[2] * 10.0f * 1229.0f / 4096.0f + 270.0f;

			String outputBuffer;
			outputBuffer += "Gyro : ";
			outputBuffer += String(gx) + "," + String(gy) + "," + String(gz);
			outputBuffer += ", Accel : ";
			outputBuffer += String(ax) + "," + String(ay) + "," + String(az);
			outputBuffer += ", Compass : ";
			outputBuffer += String(mx) + "," + String(my) + "," + String(mz);
			outputBuffer += "\n";

			Serial.write(outputBuffer.c_str(), outputBuffer.length());
		}
		break;
		case Euler_ANGLE:
		{
			GetEulerAngle(&angle, &quaternion);

			String outputBuffer =
				//String(sensor_timestamp)	  + ',' +
				String(angle.x - tempAngle.x) + ',' +
				String(angle.y - tempAngle.y) + ',' +
				String(angle.z - tempAngle.z) + '\n';

			Serial.write(outputBuffer.c_str(), outputBuffer.length());
		}
		break;
		case PYR_ANGLE:
		{
			GetYawPitchRoll(&angle, &quaternion);

			String outputBuffer =
				//String(sensor_timestamp) + ',' +
				String(angle.x - tempAngle.x) + ',' +  //Roll
				String(angle.y - tempAngle.y) + ',' +  //Pitch
				String(angle.z - tempAngle.z) + '\n';  //Yaw

			Serial.write(outputBuffer.c_str(), outputBuffer.length());
		}
		break;
		case QUATERNION:
		{
			String outputBuffer =
				String(quaternion.w * 100000.f) + ',' +  //W
				String(quaternion.x * 100000.f) + ',' +  //X
				String(quaternion.y * 100000.f) + ',' +  //Y
				String(quaternion.z * 100000.f) + '\n';  //Z

			Serial.write(outputBuffer.c_str(), outputBuffer.length());
		}
		break;
		}
	}
}

