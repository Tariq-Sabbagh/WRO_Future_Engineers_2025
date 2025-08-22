#include "Car.h"

Car::Car() :
    _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
    _steering(SERVO_PIN),
    _distSensors(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
    _imu(),
    _button(BUTTON_PIN),
    _pid()  // Initialize PID controller
{
    // Initialize state variables
    _offsetGyro = 0.0f;
    _turnCounter = 0;
    _previousTurnMillis = 0;
    error = 0.0f;
    angle = 0.0f;
    direction = "straight";
}

void Car::setup() {
    // Setup all hardware components
    _motors.setup();
    _steering.setup();
    _button.setup();
    
    // Configure PID controller
    _pid.setup(STEERING_KP, 0, 0);
    _pid.setOutputLimits(-90,90);
    
    if (!_imu.setup()) {
        // If IMU fails, halt everything.
        _stopAndHalt();
    }

    Serial.println("All components initialized.");
    Serial.println("Press the button to start calibration and run...");

    _button.waitForPress(); // Wait for the start button
    Serial.println("Button pressed! Calibrating...");
    // Timer::wait(1500);
    
    _imu.update(); // Get initial reading
    _offsetGyro = _imu.getHeading(); // Set initial offset
    _previousTurnMillis = millis(); // Initialize turn timer

    Serial.println("Calibration complete. Starting main loop.");
}

void Car::loop() {
    _imu.update(); // Always get the latest sensor data first
    
    _checkForTurns();   // Check for turn conditions
    _moveStraight();    // Main movement with PID steering

    // Check for end condition
    if (_turnCounter >= 12) {
        if (millis() - _previousTurnMillis >= 1000) {
            _stopAndHalt();
        }
    }
}

void Car::_moveStraight() {
    // Normalize gyro offset
    if (_offsetGyro >= 360) _offsetGyro = 0;
    else if (_offsetGyro < 0) _offsetGyro += 360;

    // Calculate current heading relative to offset
    float currentHeading = _imu.getHeading() - _offsetGyro;

    // Normalize heading to [-180, 180] range
    if (currentHeading > 180) currentHeading -= 360;
    else if (currentHeading < -180) currentHeading += 360;

    // Store error for turn decision logic
    error = currentHeading;

    Serial.println("Error: ");
    Serial.println(error);

    
    // Use PID controller for precise steering
    float steeringCorrection = _pid.compute(0, currentHeading);

    Serial.println("Correction: ");
    Serial.println(steeringCorrection);
    _steering.setAngle(-steeringCorrection);
    
    // Maintain forward motion
    _motors.forward(FORWARD_SPEED);
}

void Car::_checkForTurns() {
    // Check turn cooldown and max turns
    unsigned long currentMillis = millis();
    if (currentMillis - _previousTurnMillis < TURN_COOLDOWN_MS || _turnCounter >= 12) {
        return;
    }

    // Only consider turns when reasonably aligned
    if (abs(error) < 15) {
        // First turn decision logic
        if (_turnCounter < 1) {
            _turnLeft();
            // _turnRight();
        } 
        // Subsequent turns based on pattern
        else if (direction == "right") {
            _turnRight();
        } 
        else if (direction == "left") {
            _turnLeft();
        }
    }
}

void Car::_turnRight() {
    float frontDist = _distSensors.getFrontCm();
    if (frontDist > 1 && frontDist <= TURN_TRIGGER_DISTANCE_CM) {
        float rightDist = _distSensors.getRightCm();
        if (rightDist >= TURN_CLEARANCE_DISTANCE_CM || rightDist == 0) {
            direction = "right";
            Serial.println(">>> Turning RIGHT");
            _previousTurnMillis = millis();
            _turnCounter++;
            _offsetGyro += 90.0f;
            _pid.reset();  
        }
    }
}

void Car::_turnLeft() {
    float frontDist = _distSensors.getFrontCm();
    if (frontDist > 1 && frontDist <= TURN_TRIGGER_DISTANCE_CM) {
        float leftDist = _distSensors.getLeftCm();
        if (leftDist >= TURN_CLEARANCE_DISTANCE_CM || leftDist == 0) {
            direction = "left";
            Serial.println(">>> Turning LEFT");
            _previousTurnMillis = millis();
            _turnCounter++;
            _offsetGyro -= 90.0f;
            _pid.reset();  // Reset PID after turn
        }
    }
}

void Car::_stopAndHalt() {
    _motors.stop();
    _steering.center();
    Serial.println("Execution halted after completing route.");
    while (true); // Safety halt
}