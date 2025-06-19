#ifndef CAR_H
#define CAR_H

#include "config.h"
#include "MotorController.h"
#include "Steering.h"
#include "DistanceSensors.h"
#include "IMU.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class Car {
public:
    Car();
    void setup();
    void loop();

private:
    // Car Components
    MotorController _motors;
    Steering _steering;
    DistanceSensors _distSensors;
    IMU _imu;

    // Navigation State
    float _targetHeading;
    int _turnCounter;
    unsigned long _lastTurnTime;
    
    // Private Methods
    void navigate();
    void checkForTurns();
    void stopAll();
};

#endif // CAR_H
