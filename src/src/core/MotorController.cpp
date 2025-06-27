#include "MotorController.h"
#include "../config.h"

MotorController::MotorController(int dir1_pin, int dir2_pin, int speed_pin) {
    _dir1_pin = dir1_pin;
    _dir2_pin = dir2_pin;
    _speed_pin = speed_pin;
}

void MotorController::setup() {
    Serial.println("Setting up Motors....");
    pinMode(_dir1_pin, OUTPUT);
    pinMode(_dir2_pin, OUTPUT);
    pinMode(_speed_pin,OUTPUT);
    stop(); // Ensure motors are off at start
}

void MotorController::forward(int speed) {
    digitalWrite(_dir1_pin, LOW);
    digitalWrite(_dir2_pin, HIGH);
    analogWrite(_speed_pin, FORWARD_SPEED);
}

void MotorController::backward(int speed) {
    digitalWrite(_dir1_pin, HIGH);
    digitalWrite(_dir2_pin, LOW);
    analogWrite(_speed_pin, FORWARD_SPEED);
}

void MotorController::stop() {
    digitalWrite(_dir1_pin, LOW);
    digitalWrite(_dir2_pin, LOW);
    analogWrite(_speed_pin, 0);
}
