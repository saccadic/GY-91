#define EMPL_TARGET_ATMEGA328

#include <Wire.h>
#include <I2Cdev.h>
#include "helper_3dmath.h"

extern "C" {
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
}

#define GYRO_250DPS  250
#define GYRO_500DPS  500
#define GYRO_1000DPS 1000
#define GYRO_2000DPS 2000
#define Accel_2G 2
#define Accel_4G 4
#define Accel_8G 8
#define Accel_16G 16

/* Filter configurations. */
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

Quaternion q;

#include "BMP280.h"
BMP280  bmp;
#define BMP_OVERSAMPLING 4
#define P0 1013.25

short sensors;
short gyro[3], accel[3], realAccel[3];
unsigned char more = 0;
long quat[4];
long sensor_timestamp;

float gravity[3];

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
  /*
     XYZ  010_001_000 Identity Matrix
     XZY  001_010_000
     YXZ  010_000_001
     YZX  000_010_001
     ZXY  001_000_010
     ZYX  000_001_010
  */
  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;
  return scalar;
}

uint8_t dmpGetGravity(float *g, Quaternion *q) {
  g[0] = 2 * (q -> x * q -> z - q -> w * q -> y);
  g[1] = 2 * (q -> w * q -> x + q -> y * q -> z);
  g[2] = q -> w * q -> w - q -> x * q -> x - q -> y * q -> y + q -> z * q -> z;
  return 0;
}

uint8_t dmpGetLinearAccel(short *realAccel, short *Accel, float *gravity) {
  // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
  realAccel[0] = Accel[0] - gravity[0] * 8192;
  realAccel[1] = Accel[1] - gravity[1] * 8192;
  realAccel[2] = Accel[2] - gravity[2] * 8192;
  return 0;
}

double temperature;
double pressure;
double atm;
double altitude;

void updateBMP_DATA()
{
  double T, P;
  char result = bmp.startMeasurment();

  if (result != 0) {
    delay(result);
    result = bmp.getTemperatureAndPressure(T, P);
    if (result != 0)
    {
      temperature = T;
      pressure    = P;
      atm         = P / P0;
      altitude    = bmp.altitude(P, P0);
    }
    else {
      Serial.println("Error.");
    }
  }
  else {
    Serial.println("Error.");
  }
}

bool mpuSetup() {
  if (mpu_init(NULL) != 0 ) {
    Serial.print("mpu_init failed!");
    return false;
  } else {
    //MPU setting
    {
      Serial.println("****************************");
      Serial.println("****************************");
      Serial.println("Start Init mpu.");

      if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS) != 0) {
        Serial.println("mpu_set_sensors failed!");
        return false;
      } else {
        Serial.println("mpu_set_sensors complete.");
      }

      if (mpu_set_sample_rate(MPU_RATE) != 0) {
        Serial.println("mpu_set_sample_rate() failed");
        return false;
      } else {
        Serial.println("mpu_set_sample_rate complete.");
      }

      if (mpu_set_compass_sample_rate(MAG_RATE) != 0) {
        Serial.println("mpu_set_compass_sample_rate() failed");
        return false;
      } else {
        Serial.println("mpu_set_compass_sample_rate complete.");
      }

      if (mpu_set_lpf(INV_FILTER_188HZ) != 0) {
        Serial.println("mpu_set_lpf() failed");
        return false;
      } else {
        Serial.println("mpu_set_lpf complete.");
      }

      if (mpu_set_gyro_fsr(GYRO_2000DPS) != 0 ) {
        Serial.println("Failure set gyro scale.");
        return false;
      } else {
        Serial.println("mpu_set_gyro_fsr complete.");
      }

      if (mpu_set_accel_fsr(Accel_2G) != 0 ) {
        Serial.println("Failure set Accel scale.");
        return false;
      } else {
        Serial.println("mpu_set_Accel_fsr complete.");
      }

      if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
        Serial.println("Disable FIFO.");
        return false;
      } else {
        Serial.println("mpu_configure_fifo complete.");
      }
    }

    //Dmp setting
    {
      Serial.println("****************************");
      Serial.println("****************************");
      Serial.println("Start init DMP.");
      if ( dmp_load_motion_driver_firmware() != 0 ) {
        Serial.println("Failure load motion driver >> ");
        return false;
      } else {
        Serial.println("Load motion driver.");
      }

      signed char gyro_orientation[9] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
      };

      if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)) != 0) {
        Serial.println("dmp_set_orientation() failed");
        return false;
      } else {
        Serial.println("dmp_set_orientation complete.");
      }

      if (mpu_set_dmp_state(DMP_ON) != 0 ) {
        Serial.println("Stop DMP.");
        return false;
      } else {
        Serial.println("mpu_set_dmp_state complete.");
      }

      if ( dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL) != 0 ) {
        Serial.println("Disable DMP features.");
        return false;
      } else {
        Serial.println("dmp_enable_feature complete.");
      }

      if (dmp_set_fifo_rate(DMP_RATE) != 0) {
        Serial.println("\ndmp_set_fifo_rate() failed\n");
        return false;
      } else {
        Serial.println("dmp_set_fifo_rate complete.");
      }

      if (mpu_reset_fifo() != 0) {
        printf("Failed to reset fifo!\n");
        return -1;
      }

      printf("Checking... ");
      bool r;
      do {
        delay(1000 / DMP_RATE); //dmp will habve 4 (5-1) packets based on the fifo_rate
        r = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
      } while (r != 0 || more < 5); //packtets!!!
      printf("Done.\n");

    }

    //BMP setting
    {
      Serial.println("****************************");
      Serial.println("****************************");
      Serial.println("Start init BMP280.");
      if (bmp.begin() == 0) {
        Serial.println("Error bmp begin()");
        return false;
      } else {
        Serial.println("bmp begin complete.");
      }

      if (bmp.setOversampling(BMP_OVERSAMPLING) == 0) {
        Serial.println("Error bmp setOversampling()");
        return false;
      } else {
        Serial.println("bmp setOversampling complete.");
      }
    }

    return true;
  }
}

bool completeInit = false;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();

  //  Serial.begin(38400);
  Serial.begin(230400);
  //while (!Serial);

  Serial.println();
  Serial.println(F("Initializing MPU..."));

  if (mpuSetup()) {
    Serial.println("Completed all setting !!");
    completeInit = true;
  } else {
    Serial.println("Failure setting !!");
    mpuSetup();
  }
}

void loop() {
  if (completeInit) {
    //DMP update
    do {
      int success = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

      if ((success == 0)) {
        q = Quaternion( (float)(quat[0]) / 1073741824.f,
                        (float)(quat[1]) / 1073741824.f,
                        (float)(quat[2]) / 1073741824.f,
                        (float)(quat[3]) / 1073741824.f);
      }
    } while (more > 1);

    // Send Quaternion
//    if (Serial.available() > 0) {
//      String outputBuffer = String(q.x * 100000) + ',' +  // Convert the value to an ASCII string.
//                            String(q.y * 100000) + ',' +
//                            String(q.z * 100000) + ',' +
//                            String(q.w * 100000) + ',' +
//                            String(gyro[0] * 100000) + ',' +
//                            String(gyro[1] * 100000) + ',' +
//                            String(gyro[2] * 100000) + ',' +
//                            String(a[0] * 100000) + ',' +
//                            String(a[1] * 100000) + ',' +
//                            String(a[2] * 100000) + '\n';     // Add the new line character.;
//
//      Serial.write(outputBuffer.c_str(), outputBuffer.length());
//    }


    //Send LinearAccel
    dmpGetGravity(gravity, &q);
    dmpGetLinearAccel(realAccel, accel, gravity);

    String outputBuffer = String(realAccel[0] ) + ',' +
                          String(realAccel[1] ) + ',' +
                          String(realAccel[2] ) + '\n';     // Add the new line character.;

    Serial.write(outputBuffer.c_str(), outputBuffer.length());
  }
}
