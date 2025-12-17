#pragma once

class PIDController {
public:
    PIDController();
    void setup(float kp = 0.0, float ki = 0.0, float kd = 0.0);
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void reset();
    float compute(float setpoint, float input);
    float getError();

private:
    float kp, ki, kd;
    float prevError;
    float integral;
    float outputMin, outputMax;
    unsigned long lastTime;
};
