#include "IMU.h"

IMU::IMU() : _bno(Adafruit_BNO055(55, 0x28, &Wire)) {}

bool IMU::setup() {
    if (!_bno.begin()) {
        Serial.println("ERROR: No BNO055 detected. Check wiring or I2C address.");
        return false;
    }
    _bno.setExtCrystalUse(true);
    return true;
}

void IMU::update() {
    _bno.getEvent(&_event);
}

float IMU::getHeading() {
    // return _heading;
    return _heading;
}

float IMU::getHeadingRotating() {
    return _rotationCount * 360.0f + _prevRawHeading;
}

void IMU::reset() {
    sensors_event_t event;
    _bno.getEvent(&event);

    // Store current heading as offset
    float rawHeading = event.orientation.x;
    if (rawHeading > 180.0f) {
        rawHeading -= 360.0f;
    }

    _offset = rawHeading;
}
