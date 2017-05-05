#include <Wire.h>

#include "esp32_tools.h"
#include "helper_3dmath.h"
#include "quaternionFilters.h"

extern "C" {
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
}

//Sensor options
#define POWER_ON 1
#define POWER_OFF 2
#define MPU_RATE 1000	//MAX:1000
#define DMP_RATE 200	//MAX:200
#define MAG_RATE 100	//MAX:100
#define DMP_ON 1
#define DMP_OFF 0
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

//Tools
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

void GetEulerAngle(VectorFloat *angle, Quaternion* q) {
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

//##########################################
//Option
const int GyroScale = GYRO_2000DPS;
const int AccelScale = Accel_2G;
const int lpf_rate = INV_FILTER_188HZ;

//##########################################
//Buffer
int fifoResult;
long unsigned int sensor_timestamp;
short sensors;
unsigned char fifoCount = 0;
short gyro[3];
short accel[3];
short commpass[3];
long quat[4];
//Mathematics data
Quaternion rawQuaternion;
Quaternion quaternion;
Quaternion Temp_quaternion;
VectorFloat angle;
VectorFloat tempAngle;
//##########################################


//Set i2C
#define SDA 25
#define SCL 26

//Setting options
#define SERIAL_SPEED 250000

void setup() {
	Serial.begin(SERIAL_SPEED);
	Wire.begin(SDA, SCL);

	Serial.println("\n-----MPU-----");

	if (mpu_init(NULL) != 0)
	{
		Serial.println("mpu_init failed!");
		return;
	}
	else {
		Serial.println("Initialize sensor");

		//Select using sensor
		if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS) != 0) {
			Serial.println("mpu_set_sensors failed!");
			return;
		}
		else {
			Serial.println("Activate gyro, accel and compass sensor.");
		}

		//Set Scale
		if (mpu_set_gyro_fsr(GyroScale) != 0) {
			Serial.println("Failure set gyro scale.");
			return;
		}
		else {
			Serial.println("Set gyro scale : " + String(GyroScale) + "DPS");
		}
		if (mpu_set_accel_fsr(AccelScale) != 0) {
			Serial.println("Failure set Accel scale.");
			return;
		}
		else {
			Serial.println("Set accel scale : " + String(AccelScale) + "G");
		}

		////Set Update rate
		if (mpu_set_sample_rate(MPU_RATE) != 0) {
			Serial.println("mpu_set_sample_rate() failed");
			return;
		}
		else {
			Serial.println("Set MPU sample rate : " + String(MPU_RATE) + "hz");
		}

		if (mpu_set_compass_sample_rate(MAG_RATE) != 0) {
			Serial.println("mpu_set_compass_sample_rate() failed");
			return;
		}
		else {
			Serial.println("Set compass sample rate : " + String(MAG_RATE) + "hz");
		}

		////Set Filter
		if (mpu_set_lpf(lpf_rate) != 0) {
			Serial.println("mpu_set_lpf() failed");
			return;
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
			Serial.println("Failed to initialize MPU fifo!\n");
			return;
		}
		else {
			Serial.println("Active fifo gyro, accel sensor.");
		}

		// verify connection
		unsigned char devStatus;
		Serial.println("Powering up MPU...");
		mpu_get_power_state(&devStatus);
		if (devStatus)
			Serial.println("> MPU6500 connection successful\n");
		else
			Serial.println("> MPU6500 connection failed %u\n");

		//DMP Setup
		Serial.println("-----DMP------");

		if (dmp_load_motion_driver_firmware() != 0) {
			Serial.println("Failure load motion driver.");
			return;
		}
		else {
			Serial.println("Load motion driver.");
		}

		if (mpu_set_dmp_state(DMP_ON) != 0) {
			Serial.println("Stop DMP.");
			return;
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
			return;
		}
		else {
			Serial.println("dmp_set_orientation complete.");
		}

		if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL) != 0) {
			Serial.println("Failure DMP features.");
			return;
		}
		else {
			Serial.println("Configuring DMP setting.");
		}

		if (dmp_set_fifo_rate(DMP_RATE) != 0) {
			Serial.println("Failure set fifo rate.");
			return;
		}
		else {
			Serial.println("Set DMP rate : " + String(DMP_RATE) + "hz");
		}

		if (mpu_reset_fifo() != 0) {
			Serial.println("Failed mpu_reset_fifo!\n");
			return;
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

void loop() {
	//float ax, ay, az; //�����x
	//float gx, gy, gz; //�p���x
	//float mx, my, mz; //�n���C

	//mpu_get_gyro_reg(gyro, &sensor_timestamp);
	//gx = gyro[0] * (float)GyroScale / 32768.0f;
	//gy = gyro[1] * (float)GyroScale / 32768.0f;
	//gz = gyro[2] * (float)GyroScale / 32768.0f;

	//mpu_get_accel_reg(accel, &sensor_timestamp);
	//ax = accel[0] * (float)AccelScale / 32768.0f;
	//ay = accel[1] * (float)AccelScale / 32768.0f;
	//az = accel[2] * (float)AccelScale / 32768.0f;

	//mpu_get_compass_reg(commpass, &sensor_timestamp);
	//mx = commpass[0] * 10.0f * 1229.0f / 4096.0f + 18.0f;
	//my = commpass[1] * 10.0f * 1229.0f / 4096.0f + 70.0f;
	//mz = commpass[2] * 10.0f * 1229.0f / 4096.0f + 270.0f;

	//String outputBuffer;
	//outputBuffer += String(sensor_timestamp) + ',';
	//outputBuffer += "Gyro : ";
	//outputBuffer += String(gx) + "," + String(gy) + "," + String(gz);
	//outputBuffer += ", Accel : ";
	//outputBuffer += String(ax) + "," + String(ay) + "," + String(az);
	//outputBuffer += ", Compass : ";
	//outputBuffer += String(mx) + "," + String(my) + "," + String(mz);
	//Serial.println(outputBuffer);

	do {
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &fifoCount);
	} while (fifoCount > 1);


	rawQuaternion = Quaternion(
		(float)(quat[0] / 1073741824.f), //long to float
		(float)(quat[1] / 1073741824.f),
		(float)(quat[2] / 1073741824.f),
		(float)(quat[3] / 1073741824.f)
	);

	GetEulerAngle(&angle, &rawQuaternion);

	//Serial.print(angle.x);
	//Serial.print(",");
	//Serial.print(angle.y);
	//Serial.print(",");
	//Serial.print(angle.z);
	//Serial.print("\n");

	String outputBuffer =
		//String(sensor_timestamp) + ',' +
		String(angle.x - tempAngle.x) + ',' +
		String(angle.y - tempAngle.y) + ',' +
		String(angle.z - tempAngle.z);

	//String outputBuffer =
	//	String(rawQuaternion.w) + ',' +  //W
	//	String(rawQuaternion.x) + ',' +  //X
	//	String(rawQuaternion.y) + ',' +  //Y
	//	String(rawQuaternion.z) + '\n';  //Z

	Serial.println(outputBuffer);
}
