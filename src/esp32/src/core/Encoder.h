#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <AS5600.h>  // Rob Tillaart's AS5600 library
#include <Wire.h>
#include "config.h"

class Encoder {
public:
    Encoder();
    bool begin();
    void update();               // Call frequently to track motion
    void reset();                // Reset accumulated distance
    float getDistanceCm() const;

private:
    AS5600 _as5600;
    int _lastAngle;
    float _totalAngle;
    float _distanceCm;
};

#endif
