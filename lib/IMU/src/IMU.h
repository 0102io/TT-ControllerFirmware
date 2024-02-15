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
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    uint16_t temperature;
    uint8_t* accelXptr;
    uint8_t* accelYptr;
    uint8_t* accelZptr;
    uint8_t* gyroXptr;
    uint8_t* gyroYptr;
    uint8_t* gyroZptr;

private:
    Adafruit_LSM6DSOX sox;
};

#endif // IMU_H
