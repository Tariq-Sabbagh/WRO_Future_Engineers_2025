#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU {
public:
    IMU();
    bool setup();
    void update();
    float getHeading();

private:
    Adafruit_BNO055 _bno;
    sensors_event_t _event;
};

#endif // IMU_H