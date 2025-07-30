#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include <NewPing.h>
#include "config.h"

class DistanceSensors {
public:
    DistanceSensors(int front_pin, int left_pin, int right_pin);
    float getFrontCm();
    float getLeftCm();
    float getRightCm();

private:
    NewPing _sonar_front;
    NewPing _sonar_left;
    NewPing _sonar_right;
};

#endif // DISTANCE_SENSORS_H