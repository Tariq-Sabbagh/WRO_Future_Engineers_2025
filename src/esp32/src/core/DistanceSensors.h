#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include <Arduino.h>
#include <NewPing.h>

// Adjust as needed
#define ULTRASONIC_MAX_DISTANCE_CM 400
#define MEDIAN_SAMPLES 5

class DistanceSensors {
public:
    DistanceSensors(int front_pin, int left_pin, int right_pin);

    float getFrontCm();
    float getLeftCm();
    float getRightCm();
    float readReliable(NewPing& sensor, int threshold);

private:
    NewPing _sonar_front;
    NewPing _sonar_left;
    NewPing _sonar_right;

    float _kalman_front = 0;
    float _kalman_left = 0;
    float _kalman_right = 0;

    float _error_front = 1;
    float _error_left = 1;
    float _error_right = 1;

    const float _kalmanQ = 0.05;
    const float _kalmanR = 5.0;

    float median(NewPing& sensor, int samples = MEDIAN_SAMPLES);
    float kalman(float measurement, float& estimate, float& error);
};

#endif
