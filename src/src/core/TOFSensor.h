#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#define TOF_FILTER_SIZE 2
class TOFSensor {
public:
    TOFSensor(uint8_t xshutPin, uint8_t i2cAddress = 0x30, uint16_t defaultDistance = 8000);

    bool begin();
    void update(); // in mm
    bool isOutOfRange();
    uint16_t getDistance();

private:
    uint8_t _xshutPin;
    uint8_t _i2cAddress;
    uint16_t _defaultDistance;
    
    uint16_t _distanceBuffer[TOF_FILTER_SIZE];
    uint16_t _avgDistance = 0;
    uint8_t _bufferIndex = 0;
    bool _bufferFilled = false;

    Adafruit_VL53L0X _lox;
    VL53L0X_RangingMeasurementData_t _measure;
};

#endif
