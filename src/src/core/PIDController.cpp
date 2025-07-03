#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController() {
    kp = ki = kd = 0.0;
    prevError = 0.0;
    integral = 0.0;
    outputMin = 0.0;
    outputMax = 255.0; // default for PWM
    lastTime = millis();
}

void PIDController::setup(float p, float i, float d) {
    setTunings(p, i, d);
    reset();
}

void PIDController::setTunings(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

void PIDController::setOutputLimits(float min, float max) {
    outputMin = min;
    outputMax = max;
}

void PIDController::reset() {
    prevError = 0.0;
    integral = 0.0;
    lastTime = millis();
}

float PIDController::compute(float setpoint, float input) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;

    float error = setpoint - input;
    integral += error * dt;
    float derivative = (dt > 0) ? (error - prevError) / dt : 0.0;

    float output = kp * error + ki * integral + kd * derivative;

    if (output > outputMax) output = outputMax;
    else if (output < outputMin) output = outputMin;

    prevError = error;
    lastTime = now;

    return output;
}
