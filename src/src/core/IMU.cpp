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
    return _event.orientation.x; // Return the raw absolute heading
}