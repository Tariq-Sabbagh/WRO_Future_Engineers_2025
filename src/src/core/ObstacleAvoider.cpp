#include "ObstacleAvoider.h"

ObstacleAvoider::ObstacleAvoider() : _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
                                     _steering(SERVO_PIN),
                                     _imu(),
                                     _pid(),
                                     _encoder(),
                                     _button(BUTTON_PIN),
                                     timer(),
                                     _comm() // Initialize the communicator
{
    _currentState = IDLE;
}
float distance, angle;
void ObstacleAvoider::setup() {
    Wire.begin();
    _motors.setup();
    _steering.setup();
    _button.setup();
    _encoder.begin();
    while (!Serial)
    ; // Wait for serial monitor to open (remove for production)
    Serial.println("ESP32 Ready");
    
    if (!_imu.setup()) {
        Serial.println("FATAL: IMU failed to initialize.");
        _stopAndHalt();
    }
    _button.waitForPress();
    _pid.setup(3.5, 0, 0);
    _pid.setOutputLimits(-90, 90);
    
    Serial.println("Obstacle Avoider Initialized. Waiting for commands...");
}

void ObstacleAvoider::loop()
{
    switch (_currentState) {
        case FORWARD:
        _goForward();
        break;
        
        case AVOIDING:
        _avoidObs();
        break;
        
        case STOP:
        _stopAndHalt();
        break;
        
        case IDLE:
        _execute();
        break;
    }
}

void ObstacleAvoider::_avoidObs()
{  
    float currentHeading, correction;
    
    float currentDistance = _encoder.getDistanceCm();
    if (abs(currentDistance) <= distance)
    {
        // Serial.print("Received command -> Distance: ");
        // Serial.println(distance);
        // Serial.println(abs(currentDistance));
        _encoder.update();
        currentDistance = _encoder.getDistanceCm();
        _imu.update();
        currentHeading = _imu.getHeading();
        // Serial.print("Received command -> gyro: ");
        // Serial.println(currentHeading);
        correction = _pid.compute(angle, currentHeading);
        _steering.setAngle(-correction);
        _motors.forward(FORWARD_SPEED - 75);
        

    }
    else if (abs( _imu.getHeading()) >= 5)
        {
            // Serial.println("reset car_______________________________________________________");
            _imu.update();
            currentHeading = _imu.getHeading();
            correction = _pid.compute(0, currentHeading);
            _steering.setAngle(-correction);
        }
    else 
    {
        _currentState = IDLE;
    }

}

void ObstacleAvoider::_execute()
{
    _motors.stop();
    // timer.wait(500);
     if (_comm.getManeuverCommand(distance, angle))
        {
            Serial.print("Received command -> Distance: ");
            Serial.print(distance);
            Serial.print(" cm, Angle: ");
            Serial.print(angle);
            Serial.println(" degrees.");
            _encoder.reset();
            _encoder.update();
            _imu.update();
            // _currentState = AVOIDING;
        }
    else
    {
        _currentState = FORWARD;
    }

}



void ObstacleAvoider::_goForward()
{
    float currentHeading, correction;
    _imu.update();
    currentHeading = _imu.getHeading();
    correction = _pid.compute(0, currentHeading);
    _steering.setAngle(-correction);
    // _motors.forward(FORWARD_SPEED - 75);
    
        
}


void ObstacleAvoider::_stopAndHalt()
{
    _motors.stop();
    _steering.center();
    Serial.println("Execution Halted.");
}
