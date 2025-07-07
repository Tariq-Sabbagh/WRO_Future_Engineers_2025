#include "ObstacleAvoider.h"

ObstacleAvoider::ObstacleAvoider() : _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
                                     _steering(SERVO_PIN),
                                     _imu(),
                                     _pid(),
                                     _encoder(),
                                     _button(BUTTON_PIN),
                                     _comm() // Initialize the communicator
{
    _currentState = IDLE;
}

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

    if (!_imu.setup())
    {
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
    // Only listen for commands when idle.
    if (_currentState == IDLE)
    {
        float distance, angle;
        // Check for a new command. This is non-blocking.
        if (_comm.getManeuverCommand(distance, angle))
        {
            Serial.print("Received command -> Distance: ");
            Serial.print(distance);
            Serial.print(" cm, Angle: ");
            Serial.print(angle);
            Serial.println(" degrees.");
        }
        _encoder.reset();
        _imu.update();
        _encoder.update();
        float currentHeading, correction;
        float currentDistance = _encoder.getDistanceCm();
        while (abs(currentDistance) <= distance)
        {
            _encoder.update();
            currentDistance = _encoder.getDistanceCm();
            _imu.update();
            currentHeading = _imu.getHeading();
            correction = _pid.compute(angle, currentHeading);
            _steering.setAngle(-correction);
            // Serial.println(currentHeading);
            _motors.forward(FORWARD_SPEED - 75);
        }
        while (abs(currentHeading) >= 5)
        {
            _imu.update();
            currentHeading = _imu.getHeading();
            correction = _pid.compute(0, currentHeading);
            _steering.setAngle(-correction);
        }
        _motors.stop();
        
    }
}

void ObstacleAvoider::_executeManeuver(float distance, float angle)
{
    _currentState = AVOIDING;
    Serial.println("State: AVOIDING");

    Serial.println("Step 1: Turning...");
    _turn(angle);

    Serial.println("Step 2: Driving forward...");
    _driveDistance(distance);

    Serial.println("Step 3: Turning back to zero...");
    _turn(0);

    _motors.stop();
    _steering.center();
    Serial.println("Maneuver complete. State: IDLE");
    _currentState = IDLE;
}

void ObstacleAvoider::_turn(float targetAngle)
{
    _imu.update();
    while (true)
    {
        _imu.update();
        float currentHeading = _imu.getHeading();
        if (abs(currentHeading - targetAngle) < 2.0)
        {
            break;
        }
        float correction = _pid.compute(targetAngle, currentHeading);
        _steering.setAngle(-correction);
        _motors.forward(FORWARD_SPEED - 75);
    }
    _motors.stop();
    delay(250);
}

void ObstacleAvoider::_driveDistance(float targetDistance)
{
    _encoder.reset();
    _imu.update();
    while (true)
    {
        _imu.update();
        float currentDistance = _encoder.getDistanceCm();
        if (currentDistance >= targetDistance)
        {
            break;
        }
        float correction = _pid.compute(0, _imu.getHeading());
        _steering.setAngle(-correction);
        _motors.forward(FORWARD_SPEED);
    }
    _motors.stop();
    delay(250);
}

void ObstacleAvoider::_stopAndHalt()
{
    _motors.stop();
    _steering.center();
    Serial.println("Execution Halted.");
    while (true)
        ;
}
