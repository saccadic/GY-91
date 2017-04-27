#include <Wire.h>
#include <I2Cdev.h>
#include "helper_3dmath.h"
#include "quaternionFilters.h"

extern "C" {
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
}

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
#define MPU_RATE 1000
#define DMP_RATE 200
#define MAG_RATE 100
#define DMP_ON 1
#define DMP_OFF 0

//Setting options
#define SERIAL_SPEED 230400

//Option
const int GyroScale = GYRO_2000DPS;
const int AccelScale = Accel_2G;
const int lpf_rate = INV_FILTER_188HZ;

//Update mode
enum {
	DMP,		//Digital Motion Processor
	MADGWICK,	//MadgwickQuaternionUpdate
	MAHONY,		//MahonyQuaternionUpdate
};
int processEngine = DMP;

enum {
	IDOLE,
	RAWDATA,
	Euler_ANGLE,
	PYR_ANGLE,
	QUATERNION,
};
int updateMode = QUATERNION;

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

//Mathematics data
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
uint32_t delt_t = 0;
float deltat = 0.0f;
uint32_t lastUpdate = 0, firstUpdate = 0;			// used to calculate integration interval
uint32_t Now = 0;									// used to calculate integration interval
float eInt[3] = { 0.0f, 0.0f, 0.0f };				// vector to hold integral error for Mahony method

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

float RadToDeg(float x) { return x * (180 / PI); }

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

uint8_t GetGravity(VectorFloat *v, Quaternion *q) {
	v->x = 2 * (q->x*q->z - q->w*q->y);
	v->y = 2 * (q->w*q->x + q->y*q->z);
	v->z = q->w*q->w - q->x*q->x - q->y*q->y + q->z*q->z;
	return 0;
}

uint8_t GetYawPitchRoll(VectorFloat *ypr, Quaternion *q, VectorFloat *gravity) {
	// yaw: (about Z axis)
	ypr->z = atan2(2 * q->x*q->y - 2 * q->w*q->z, 2 * q->w*q->w + 2 * q->x*q->x - 1);
	// pitch: (nose up/down, about Y axis)
	ypr->y = atan(gravity->x / sqrt(gravity->y*gravity->y + gravity->z*gravity->z));
	// roll: (tilt left/right, about X axis)
	ypr->x = atan(gravity->y / sqrt(gravity->x*gravity->x + gravity->z*gravity->z));
	return 0;
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
}

int InitMPU9250() {

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
		Serial.println("\nMPU9250 ready !!\n");
	}
}

void setup() {
	// put your setup code here, to run once:
	Wire.begin();
	Serial.begin(SERIAL_SPEED);
	Serial.println("\nInitializing MPU...");
	InitMPU9250();
}

void loop() {
	//Command
	if (Serial.available() > 0) {
		char inputchar = Serial.read();
		switch (inputchar) {
		case 'o':
			tempAngle = angle;
			break;
		}
	}

	//Update data
	//TODO dalay

	if (processEngine != RAWDATA) {
		switch (processEngine) {
		case DMP:
		{
			do { dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &fifoCount); } while (fifoCount > 1);

			Quaternion rawQuaternion = Quaternion(
				(float)(quat[0] / 1073741824.f), //long to float
				(float)(quat[1] / 1073741824.f),
				(float)(quat[2] / 1073741824.f),
				(float)(quat[3] / 1073741824.f)
			);

			//Filter
			float a = 9.0f;
			quaternion.w = a * rawQuaternion.w - (1.0f - a) * Temp_quaternion.w;
			quaternion.x = a * rawQuaternion.x - (1.0f - a) * Temp_quaternion.x;
			quaternion.y = a * rawQuaternion.y - (1.0f - a) * Temp_quaternion.y;
			quaternion.z = a * rawQuaternion.z - (1.0f - a) * Temp_quaternion.z;
			Temp_quaternion = rawQuaternion;
		}
		break;
		case MADGWICK:
		{

			Now = micros();
			deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
			lastUpdate = Now;

			mpu_get_gyro_reg(gyro, &sensor_timestamp);
			mpu_get_accel_reg(accel, &sensor_timestamp);
			mpu_get_compass_reg(commpass, &sensor_timestamp);

			MadgwickQuaternionUpdate(&quaternion, beta, zeta, deltat, accel[0], accel[1], accel[2], gyro[0] * PI / 180.0f, gyro[1] * PI / 180.0f, gyro[2] * PI / 180.0f, commpass[1], commpass[0], commpass[2]);

		}
		break;
		case MAHONY:
		{

			Now = micros();
			deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
			lastUpdate = Now;

			mpu_get_gyro_reg(gyro, &sensor_timestamp);
			mpu_get_accel_reg(accel, &sensor_timestamp);
			mpu_get_compass_reg(commpass, &sensor_timestamp);

			MahonyQuaternionUpdate(&quaternion, eInt, Ki, Kp, deltat, accel[0], accel[1], accel[2], gyro[0] * PI / 180.0f, gyro[1] * PI / 180.0f, gyro[2] * PI / 180.0f, commpass[1], commpass[0], commpass[2]);
		}
		break;
		}
	}

	//Output
	//if (Serial.available() > 0) {
	switch (updateMode) {
	case RAWDATA:
	{
		mpu_get_gyro_reg(gyro, &sensor_timestamp);
		mpu_get_accel_reg(accel, &sensor_timestamp);
		mpu_get_compass_reg(commpass, &sensor_timestamp);

		String outputBuffer;
		outputBuffer += "Gyro : ";
		outputBuffer += String(gyro[0]) + "," + String(gyro[1]) + "," + String(gyro[2]);
		outputBuffer += ", Accel : ";
		outputBuffer += String(accel[0]) + "," + String(accel[1]) + "," + String(accel[2]);
		outputBuffer += ", Compass : ";
		outputBuffer += String(commpass[0]) + "," + String(commpass[1]) + "," + String(commpass[2]);
		outputBuffer += "\n";

		Serial.write(outputBuffer.c_str(), outputBuffer.length());
	}
	break;
	case Euler_ANGLE:
	{
		GetEulerAngle(&angle, &quaternion);

		//Serial.print(sensor_timestamp);
		//Serial.print(",");
		Serial.print(RadToDeg(angle.x - tempAngle.x));
		Serial.print(",");
		Serial.print(RadToDeg(angle.y - tempAngle.y));
		Serial.print(",");
		Serial.println(RadToDeg(angle.z - tempAngle.z));
	}
	break;
	case PYR_ANGLE:
	{
		VectorFloat gravity, calcYPR;
		GetGravity(&gravity, &quaternion);
		GetYawPitchRoll(&angle, &quaternion, &gravity);
		calcYPR.x = RadToDeg(angle.x - tempAngle.x);
		calcYPR.y = RadToDeg(angle.y - tempAngle.y);
		calcYPR.z = RadToDeg(angle.z - tempAngle.z);
		calcYPR.x = wrap_180(calcYPR.x);
		calcYPR.y *= -1.0;

		//Serial.print(sensor_timestamp);
		//Serial.print(",");
		Serial.print(RadToDeg(angle.x - tempAngle.x));
		Serial.print(",");
		Serial.print(RadToDeg(angle.y - tempAngle.y));
		Serial.print(",");
		Serial.println(RadToDeg(angle.z - tempAngle.z));
	}
	break;
	case QUATERNION:
	{
		String outputBuffer =
			String(quaternion.x * 100000.f) + ',' +  // Convert the value to an ASCII string.
			String(quaternion.y * 100000.f) + ',' +
			String(quaternion.z * 100000.f) + ',' +
			String(quaternion.w * 100000.f) + '\n';  // Add the new line character.;

		Serial.write(outputBuffer.c_str(), outputBuffer.length());
	}
	break;
	}
	//}
}

