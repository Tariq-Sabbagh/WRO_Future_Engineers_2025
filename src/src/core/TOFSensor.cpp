#include "TOFSensor.h"

TOFSensor::TOFSensor(uint8_t xshutPin, uint8_t i2cAddress, uint16_t defaultDistance)
    : _xshutPin(xshutPin), _i2cAddress(i2cAddress), _defaultDistance(defaultDistance) {
}

bool TOFSensor::begin() {
    pinMode(_xshutPin, OUTPUT);

    // Reset the sensor
    digitalWrite(_xshutPin, LOW);
    delay(10);
    digitalWrite(_xshutPin, HIGH);
    delay(10);

    // Initialize the sensor with custom I2C address
    if (!_lox.begin(_i2cAddress)) {
        Serial.println(F("Failed to initialize VL53L0X sensor"));
        return false;
    }
    return true;
}

uint16_t TOFSensor::readDistance() {
    _lox.rangingTest(&_measure, false);

    if (_measure.RangeStatus != 4) {
        return _measure.RangeMilliMeter;
    } else {
        return _defaultDistance; // return max or default value if out of range
    }
}

bool TOFSensor::isOutOfRange() {
    return (_measure.RangeStatus == 4);
}
