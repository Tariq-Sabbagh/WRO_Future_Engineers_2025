#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#define TOF_FILTER_SIZE 5
class TOFSensor {
public:
    TOFSensor(uint8_t xshutPin, uint8_t i2cAddress = 0x30, uint16_t defaultDistance = 8190);

    bool begin();
    uint16_t readDistance(); // in mm
    bool isOutOfRange();

private:
    uint8_t _xshutPin;
    uint8_t _i2cAddress;
    uint16_t _defaultDistance;

    uint16_t _distanceBuffer[TOF_FILTER_SIZE];
    uint8_t _bufferIndex = 0;
    bool _bufferFilled = false;

    Adafruit_VL53L0X _lox;
    VL53L0X_RangingMeasurementData_t _measure;
};

#endif
