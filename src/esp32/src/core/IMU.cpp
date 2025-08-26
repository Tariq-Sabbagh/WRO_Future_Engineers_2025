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

    // Raw heading in [0, 360)
    float rawHeading = event.orientation.x;

    // Handle wrap-around to maintain rotation count
    float delta = rawHeading - _prevRawHeading;

    // Correct for wraparound
    if (delta > 180.0f) {
        _rotationCount--;
    } else if (delta < -180.0f) {
        _rotationCount++;
    }

    _prevRawHeading = rawHeading;

    // Convert to [-180, 180)
    float normalizedHeading = rawHeading;
    if (normalizedHeading > 180.0f) {
        normalizedHeading -= 360.0f;
    }

    // Apply offset
    _heading = normalizedHeading - _offset;

    // Normalize result again to -180..180
    if (_heading > 180.0f) _heading -= 360.0f;
    if (_heading < -180.0f) _heading += 360.0f;
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