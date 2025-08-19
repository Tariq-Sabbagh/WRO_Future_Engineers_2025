#include "Car.h"

Car::Car() : _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
             _steering(SERVO_PIN),
             _distSensors(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
             _imu(),
             _button(BUTTON_PIN),
             _pid(),
             _encoder()
{
    // direction = "straight";
}

void Car::setup()
{
  
    _motors.setup();
    _steering.setup();
    _button.setup();
    _encoder.begin();

    _pid.setup(STEERING_KP, 0, 0);
    _pid.setOutputLimits(-90, 90);

    if (!_imu.setup())
    {
        Serial.println("IMU failed.");
        _stopAndHalt();
    }

    Serial.println("All components initialized.");
    Serial.println("Press the button to start...");

    _button.waitForPress();
    Serial.println("Button pressed! Calibrating...");

    _imu.update();
    _previousTurnMillis = millis();

    Serial.println("Calibration complete. Starting course...");
}

void Car::loop()
{
    _runCourse(); // Start the main course logic
    _stopAndHalt();
    // Not used in this structure
}

// ---------- COURSE LOGIC ----------

void Car::_runCourse()
{
    for (int lab = 0; lab < _totalLabs; lab++)
    {
        Serial.print("Starting lab ");
        Serial.println(lab + 1);
        _runLab();
    }
}

void Car::_runLab()
{
    for (int segment = 0; segment < 1; segment++)
    {
        Serial.print("Starting segment ");
        Serial.println(segment + 1);
        _goUntilTurn();
        _decideAndTurn();
    }
       
}

void Car::_goUntilTurn()
{
    _imu.reset();
    while (true)
    {
         _imu.update();
        _moveStraight();
        if (_empty_on_left() or _empty_on_right())
        {
            // Serial.println("Wall detected. Stopping.");
            break;
        }
    }
}

void Car::_decideAndTurn()
{

    if (_empty_on_left())
        _turn(90);
    else if (_empty_on_right())
        _turn(-90);
}

void Car::_turn(float angle)
{

    while (true)
    {
        _imu.update();
        float currentHeading = _imu.getHeading();

        if (abs(currentHeading - angle) < 0.1f)
            break;

        float correction = _pid.compute(angle, currentHeading);
        _steering.setAngle(-correction);
        _motors.forward(FORWARD_SPEED - 50);
    }
}

// ---------- MOVEMENT & CONTROL ----------

void Car::_moveStraight()
{
    float currentHeading = _imu.getHeading();
    error = currentHeading - 0;
    Serial.println("Error: ");
    Serial.println(error);

    float correction = _pid.compute(0, currentHeading);

    _steering.setAngle(-correction);
    _motors.forward(FORWARD_SPEED);
}

// ---------- TURNING ----------

bool Car::_empty_on_right()
{
    float frontDist = _distSensors.getFrontCm();
    float rightDist = _distSensors.getRightCm();
    return (abs(_imu.getHeading()) < 15) and (frontDist > 1 && frontDist <= TURN_TRIGGER_DISTANCE_CM) and
           (rightDist > 1 && rightDist <= TURN_CLEARANCE_DISTANCE_CM);
}

bool Car::_empty_on_left()
{
    float frontDist = _distSensors.getFrontCm();
    float leftDist = _distSensors.getLeftCm();
    return (abs(_imu.getHeading()) < 15) and (frontDist > 1 && frontDist <= TURN_TRIGGER_DISTANCE_CM) and
           (leftDist > 1 && leftDist <= TURN_CLEARANCE_DISTANCE_CM);
}

// ---------- SAFETY ----------

void Car::_stopAndHalt()
{
    _motors.stop();
    _steering.center();
    Serial.println("Execution halted after completing route.");
    while (true); // Safety halt
}
