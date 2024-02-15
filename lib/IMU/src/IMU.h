#ifndef IMU_H
#define IMU_H

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

struct ImuData {
    uint32_t accelX;
    uint32_t accelY;
    uint32_t accelZ;
    uint32_t gyroX;
    uint32_t gyroY;
    uint32_t gyroZ;
    uint16_t temperature;
};

/*
Wrapper class for the Adafruit_LSM6DSOX inertial measurement unit (IMU).
*/
class IMU
{
public:
    IMU();
    void setupIMU();
    void poll();
    ImuData imuData;

private:
    Adafruit_LSM6DSOX sox;
};

#endif // IMU_H
