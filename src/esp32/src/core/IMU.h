#pragma once

#include <Adafruit_BNO055.h>
#include <Wire.h>

class IMU {
public:
    IMU();
    bool setup();
    void update();         // Reads new sensor data
    float getHeading();    // Returns heading in -180 to 180
    float getHeadingRotating();  // -90 0 90 180 270 360 450...
    void reset();          // Sets current heading as "zero"

private:
    Adafruit_BNO055 _bno;
    float _heading = 0.0f; // Stores current heading only
    float _offset = 0.0f;

    float _prevRawHeading = 0.0f;
    int _rotationCount = 0;

};