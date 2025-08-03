#include "TOFSensor.h"

TOFSensor::TOFSensor(uint8_t xshutPin, uint8_t i2cAddress, uint16_t defaultDistance)
    : _xshutPin(xshutPin), _i2cAddress(i2cAddress), _defaultDistance(defaultDistance)
{
}

bool TOFSensor::begin()
{
    pinMode(_xshutPin, OUTPUT);

    // Reset the sensor
    digitalWrite(_xshutPin, LOW);
    delay(10);
    digitalWrite(_xshutPin, HIGH);
    delay(10);

    // Initialize the sensor with custom I2C address
    if (!_lox.begin(_i2cAddress))
    {
        Serial.println(F("Failed to initialize VL53L0X sensor"));
        return false;
    }
    return true;
}

void TOFSensor::update()
{
}
uint16_t TOFSensor::getDistance(){
    
    _lox.rangingTest(&_measure, false);

    uint16_t rawDistance;
    if (_measure.RangeStatus != 4)
    {
        rawDistance = _measure.RangeMilliMeter;
    }
    else
    {
        rawDistance = _defaultDistance;
    }
    if (rawDistance < 10)
        rawDistance = 8000;
    // Add to circular buffer
    _distanceBuffer[_bufferIndex++] = rawDistance;
    if (_bufferIndex >= TOF_FILTER_SIZE)
    {
        _bufferIndex = 0;
        _bufferFilled = true;
    }

    // Compute average
    uint32_t sum = 0;
    uint8_t count = _bufferFilled ? TOF_FILTER_SIZE : _bufferIndex;
    for (uint8_t i = 0; i < count; i++)
    {
        sum += _distanceBuffer[i];
    }

    _avgDistance = sum / count;
    return  _avgDistance;
}
bool TOFSensor::isOutOfRange()
{
    return (_measure.RangeStatus == 4);
}
