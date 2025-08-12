#ifndef OUTPARKING_H
#define OUTPARKING_H

#include <Arduino.h>
#include "TOFSensor.h"
#include "config.h"
#include "MotorController.h"
#include "Steering.h"
#include "IMU.h"
#include "core/PIDController.h"
#include "Encoder.h"
#include "SerialCommunicator.h"
#include "Button.h"
#include <Wire.h>
#include "Timer.h"
#include "core/DistanceSensors.h"
#include "core/TOFSensor.h"

class OutParking {
public:
    OutParking();    // Constructor بدون معاملات
    void begin();
    void Do();
    void attachHardware(MotorController* motors,
                        Steering* servo,
                        Button* button,
                        Encoder* encoder,
                        IMU* imu,
                        TOFSensor* backSensor,
                        DistanceSensors* ultra) 
    {
        _motors = motors;
        _servo = servo;
        _button = button;
        _encoder = encoder;
        _imu = imu;
        _backSensor = backSensor;
        _ultra = ultra;
    }

private:
     int _servo_angle_parking = 90;
    int _pid_target_parking = -100;

    MotorController* _motors;
    Steering* _servo;
    IMU* _imu;
    Encoder* _encoder;
    Button* _button;
    DistanceSensors* _ultra;
    TOFSensor* _backSensor;
    PIDController _pid;
};

#endif // OUTPARKING_H
