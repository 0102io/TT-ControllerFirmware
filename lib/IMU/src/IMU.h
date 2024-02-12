#ifndef IMU_H
#define IMU_H

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

/*
Wrapper class for the Adafruit_LSM6DSOX inertial measurement unit (IMU).
*/
class IMU
{
public:
    IMU();
    void setupIMU();
    void poll();
    uint8_t* dataPtr;
    uint16_t tempInt;

private:
    Adafruit_LSM6DSOX sox;
    int32_t float2int32[6];
    uint8_t dataAsBytes[26];
};

#endif // IMU_H
