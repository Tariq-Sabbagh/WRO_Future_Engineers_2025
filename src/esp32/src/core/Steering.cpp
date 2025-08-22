#include "Steering.h"
#include "../config.h"
#include <Arduino.h>

Steering::Steering(int servo_pin) {
    _servo_pin = servo_pin;
}

void Steering::setup() {
    ESP32PWM::allocateTimer(0); // Allocate a timer for the servo library
    _servo.setPeriodHertz(50);
    _servo.attach(_servo_pin, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    center();
}

void Steering::setAngle(int angle) {
    int constrainedAngle = map(angle,-90,90,SERVO_CENTER_ANGLE-80,SERVO_CENTER_ANGLE+80);
    _servo.write(constrainedAngle);
    // Serial.println("constrainedAngle");
    // Serial.println(_servo_pin);
    
}

void Steering::center() {
    _servo.write(SERVO_CENTER_ANGLE);
}
