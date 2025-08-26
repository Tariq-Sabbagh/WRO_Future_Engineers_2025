#include "DistanceSensors.h"

DistanceSensors::DistanceSensors(int front_pin, int left_pin, int right_pin)
    : _sonar_front(front_pin, front_pin, ULTRASONIC_MAX_DISTANCE_CM),
      _sonar_left(left_pin, left_pin, ULTRASONIC_MAX_DISTANCE_CM),
      _sonar_right(right_pin, right_pin, ULTRASONIC_MAX_DISTANCE_CM) {
    // Initialization done via initializer list
}

float DistanceSensors::getFrontCm() {
    float med = median(_sonar_front);
    return kalman(med, _kalman_front, _error_front);
}

float DistanceSensors::getLeftCm() {
    float med = median(_sonar_left);
    return kalman(med, _kalman_left, _error_left);
}

float DistanceSensors::getRightCm() {
    float med = median(_sonar_right );
    return kalman(med, _kalman_right, _error_right);
}



float DistanceSensors::median(NewPing& sensor, int samples) {
    float values[samples];
    for (int i = 0; i < samples; i++) {
        values[i] = sensor.ping_cm();
        delay(5);
    }

    // Insertion sort
    for (int i = 0; i < samples - 1; i++) {
        for (int j = i + 1; j < samples; j++) {
            if (values[j] < values[i]) {
                float temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }

    return (samples % 2 == 0) ?
           (values[samples / 2] + values[samples / 2 - 1]) / 2.0 :
            values[samples / 2];
}

float DistanceSensors::kalman(float measurement, float& estimate, float& error) {
    if (measurement <= 0) return estimate; // reject bad measurements

    float kalmanGain = error / (error + _kalmanR);
    estimate = estimate + kalmanGain * (measurement - estimate);
    error = (1 - kalmanGain) * error + fabs(estimate - measurement) * _kalmanQ;
    return estimate;
}
