#include "IMU.h"
#include "../config.h"

IMU::IMU() : _bno(Adafruit_BNO055(55, 0x28, &Wire)) {
    _heading_offset = 0;
    _current_heading = 0;
}

bool IMU::setup() {
    if (!_bno.begin()) {
        Serial.println("ERROR: No BNO055 detected. Check wiring or I2C address.");
        return false;
    }
    _bno.setExtCrystalUse(true);
    delay(100); // Allow sensor to settle
    return true;
}

void IMU::calibrate() {
    // Set the current orientation as the new "zero"
    update();
    _heading_offset = _current_heading;
}

void IMU::update() {
    sensors_event_t event;
    _bno.getEvent(&event);
    _current_heading = event.orientation.x; // Using x for yaw
}

float IMU::getHeading() {
    // Calculate the heading relative to the calibrated offset
    float relative_heading = _current_heading - _heading_offset;

    // Normalize the angle to be between -180 and 180 degrees
    // This simplifies error calculations and prevents issues when crossing 360/0
    if (relative_heading > 180) {
        relative_heading -= 360;
    } else if (relative_heading < -180) {
        relative_heading += 360;
    }
    return relative_heading;
}

