#include "IMU.h"

IMU::IMU() : _bno(Adafruit_BNO055(55, 0x28, &Wire)) {}

bool IMU::setup() {
    if (!_bno.begin()) {
        Serial.println("ERROR: No BNO055 detected. Check wiring or I2C address.");
        return false;
    }
    _bno.setExtCrystalUse(true);
    reset();
    return true;
}

void IMU::update() {
    sensors_event_t event;
    _bno.getEvent(&event);

    // Convert heading from [0, 360) to [-180, 180)
    float rawHeading = event.orientation.x;
    if (rawHeading > 180.0f) {
        rawHeading -= 360.0f;
    }

    // Apply offset (reset)
    _heading = rawHeading - _offset;

    // Normalize result to -180..180 again after offset
    if (_heading > 180.0f) _heading -= 360.0f;
    if (_heading < -180.0f) _heading += 360.0f;
}

float IMU::getHeading() {
    return _heading;
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
