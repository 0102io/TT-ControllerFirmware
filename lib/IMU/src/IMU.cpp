#include "IMU.h"
#include "utils.h"
#include "configuration.h"

#define I2C_ADDRESS 0x6A

IMU::IMU() : sox() 
{
  Wire1.begin(SDA1, SCL1);
}

/*
Set options for accelerometer + gyro ranges and sampling frequencies.
*/
void IMU::setupIMU()
{
  DPRINTLN("Setting up IMU");

  if (!sox.begin_I2C(I2C_ADDRESS, &Wire1)) {
    delay(100);
    if (!sox.begin_I2C(I2C_ADDRESS, &Wire1)) {
      DPRINTLN("Couldn't connect to IMU");
    }
  }

  DPRINTLN("LSM6DSOX connected");

  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G); // used in adafruit library example
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  // sox.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  // sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS); // used in adafruit library example
  // sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  // sox.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  // sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  // sox.setAccelDataRate(LSM6DS_RATE_26_HZ);
  // sox.setAccelDataRate(LSM6DS_RATE_26_HZ);
  // sox.setAccelDataRate(LSM6DS_RATE_52_HZ);
  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  // sox.setAccelDataRate(LSM6DS_RATE_208_HZ);
  // sox.setAccelDataRate(LSM6DS_RATE_416_HZ);
  // sox.setAccelDataRate(LSM6DS_RATE_833_HZ);
  // sox.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  // sox.setAccelDataRate(LSM6DS_RATE_3_33K_HZ);
  // sox.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_26_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_26_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_52_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_208_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_416_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_833_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_3_33K_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
}

void IMU::poll()
{
  // from adafruit library example: https://github.com/adafruit/Adafruit_LSM6DS/blob/f6ec7a1af1ba93beff41dfe0acf9f61d556e2d09/examples/adafruit_lsm6dsox_test/adafruit_lsm6dsox_test.ino
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  // multiply data by 1k to preserve 3 decimal places and convert into ints
  float2int32[0] = (int32_t)(accel.acceleration.x * 1000);
  float2int32[1] = (int32_t)(accel.acceleration.y * 1000);
  float2int32[2] = (int32_t)(accel.acceleration.z * 1000);
  float2int32[3] = (int32_t)(gyro.gyro.x * 1000);
  float2int32[4] = (int32_t)(gyro.gyro.y * 1000);
  float2int32[5] = (int32_t)(gyro.gyro.z * 1000);
  tempInt = temp.temperature * 10; // keep one decimal place
  updateBoardTempLevel(tempInt/10); // update the temperature state so we can attenuate the tap if we're running hot

  // break data into bytes
  dataAsBytes[0] = (float2int32[0] >> 24) & 0xFF;
  dataAsBytes[1] = (float2int32[0] >> 16) & 0xFF;
  dataAsBytes[2] = (float2int32[0] >> 8) & 0xFF;
  dataAsBytes[3] = float2int32[0] & 0xFF;
  dataAsBytes[4] = (float2int32[1] >> 24) & 0xFF;
  dataAsBytes[5] = (float2int32[1] >> 16) & 0xFF;
  dataAsBytes[6] = (float2int32[1] >> 8) & 0xFF;
  dataAsBytes[7] = float2int32[1] & 0xFF;
  dataAsBytes[8] = (float2int32[2] >> 24) & 0xFF;
  dataAsBytes[9] = (float2int32[2] >> 16) & 0xFF;
  dataAsBytes[10] = (float2int32[2] >> 8) & 0xFF;
  dataAsBytes[11] = float2int32[2] & 0xFF;

  dataAsBytes[12] = (float2int32[3] >> 24) & 0xFF;
  dataAsBytes[13] = (float2int32[3] >> 16) & 0xFF;
  dataAsBytes[14] = (float2int32[3] >> 8) & 0xFF;
  dataAsBytes[15] = float2int32[3] & 0xFF;
  dataAsBytes[16] = (float2int32[4] >> 24) & 0xFF;
  dataAsBytes[17] = (float2int32[4] >> 16) & 0xFF;
  dataAsBytes[18] = (float2int32[4] >> 8) & 0xFF;
  dataAsBytes[19] = float2int32[4] & 0xFF;
  dataAsBytes[20] = (float2int32[5] >> 24) & 0xFF;
  dataAsBytes[21] = (float2int32[5] >> 16) & 0xFF;
  dataAsBytes[22] = (float2int32[5] >> 8) & 0xFF;
  dataAsBytes[23] = float2int32[5] & 0xFF;
  dataAsBytes[24] = (tempInt >> 8) & 0xFF;
  dataAsBytes[25] = tempInt & 0xFF;

  dataPtr = dataAsBytes;

  return;
}