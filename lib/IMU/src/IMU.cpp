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
  imuData.accelX = (int32_t)(accel.acceleration.x * 1000);
  imuData.accelY = (int32_t)(accel.acceleration.y * 1000);
  imuData.accelZ = (int32_t)(accel.acceleration.z * 1000);
  imuData.gyroX = (int32_t)(gyro.gyro.x * 1000);
  imuData.gyroY = (int32_t)(gyro.gyro.y * 1000);
  imuData.gyroZ = (int32_t)(gyro.gyro.z * 1000);
  imuData.temperature = (uint16_t)(temp.temperature * 10); // keep one decimal place
  updateBoardTempLevel(imuData.temperature/10); // update the temperature state so we can attenuate the tap if we're running hot

  return;
}