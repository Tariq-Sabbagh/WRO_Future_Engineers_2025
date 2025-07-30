#include "DistanceSensors.h"

DistanceSensors::DistanceSensors(int front_pin, int left_pin, int right_pin)
    : _sonar_front(front_pin, front_pin, ULTRASONIC_MAX_DISTANCE_CM),
      _sonar_left(left_pin, left_pin, ULTRASONIC_MAX_DISTANCE_CM),
      _sonar_right(right_pin, right_pin, ULTRASONIC_MAX_DISTANCE_CM) {
    // Constructor body is empty, initialization is done in the initializer list
}

float DistanceSensors::getFrontCm() {
    return _sonar_front.ping_cm();
}

float DistanceSensors::getLeftCm() {
    return _sonar_left.ping_cm();
}

float DistanceSensors::getRightCm() {
    return _sonar_right.ping_cm();
}