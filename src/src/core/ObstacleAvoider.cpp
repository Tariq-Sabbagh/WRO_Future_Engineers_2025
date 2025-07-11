#include "ObstacleAvoider.h"

ObstacleAvoider::ObstacleAvoider() : _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
                                     _steering(SERVO_PIN),
                                     _imu(),
                                     _pid(),
                                     _encoder(),
                                     _button(BUTTON_PIN),
                                     _timer(),
                                     _comm() // Initialize the communicator
{
    _currentState = FORWARD;
}
float distance, angle, _forwardTarget = 0;
void ObstacleAvoider::setup()
{
    Wire.begin();
    _motors.setup();
    _steering.setup();
    _button.setup();
    _encoder.begin();
    while (!Serial)
    ; // Wait for serial monitor to open (remove for production)
    Serial.println("ESP32 Ready");
    _button.waitForPress();

    if (!_imu.setup())
    {
        Serial.println("FATAL: IMU failed to initialize.");
        _stopAndHalt();
    }
    _pid.setup(3.5, 0, 0);
    _pid.setOutputLimits(-90, 90);

    _comm.clearSerialBuffer();
    Serial.println("Obstacle Avoider Initialized. Waiting for commands...");
}

void ObstacleAvoider::loop()
{
    _encoder.update();
    _imu.update();
    _comm.update();

    if (_comm.getTurn() != 0.f)
    {
        _forwardTarget += _comm.getTurn();
        _comm.resetTurn();
    }

    switch (_currentState)
    {
    case AVOIDING:
        _avoidObstacle();
        break;

    case IDLE:
        _stopUntilTimer();
        break;

    case FORWARD:
        _goForward();
        break;
    }
}
void ObstacleAvoider::_stopUntilTimer()
{
    _motors.stop();
    if (_timer.isFinished())
        _currentState = FORWARD;
}
void ObstacleAvoider::_avoidObstacle()
{
    float correction = 0;
    float currentHeading = _imu.getHeading();
    float currentDistance = _encoder.getDistanceCm();
    if (abs(currentDistance) <= distance)
    {
        correction = _pid.compute(angle, currentHeading);
        _steering.setAngle(-correction);
        _motors.forward(FORWARD_SPEED - 30);
    }
    else if (abs(currentHeading) >= 5)
    {
        // Serial.println("reset car_______________________________________________________");
        correction = _pid.compute(0, currentHeading);
        _steering.setAngle(-correction);
    }
    else
    {
        _comm.resetManeuverValues();
        // _timer.start(500);
        _currentState = FORWARD;
    }
}

void ObstacleAvoider::_goForward()
{
    if (_comm.getManeuverValues(distance, angle))
    {
        Serial.print("Received command -> Distance: ");
        Serial.print(distance);
        Serial.print(" cm, Angle: ");
        Serial.print(angle);
        Serial.println(" degrees.");

        _encoder.reset();
        _imu.reset();
        _currentState = AVOIDING;
    }
    else
    {
        float correction = _pid.compute(_forwardTarget, _imu.getHeadingRotating());
        _steering.setAngle(-correction);
        _motors.forward(FORWARD_SPEED - 30);
    }
}

void ObstacleAvoider::_stopAndHalt()
{
    _motors.stop();
    _steering.center();
    Serial.println("Execution Halted.");
    while (true)
        ;
}
