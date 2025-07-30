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
    void calibrate();
    float getHeading();
    void update();

private:
    Adafruit_BNO055 _bno;
    float _heading_offset;
    float _current_heading;
    sensors_event_t event;
};

#endif // IMU_H