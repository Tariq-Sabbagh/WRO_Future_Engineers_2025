#include "Car.h"

Car::Car() :
    _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
    _steering(SERVO_PIN),
    _distSensors(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
    _imu(),
    _button(BUTTON_PIN)
{
    // Initialize state variables
    _offsetGyro = 0.0f;
    _turnCounter = 0;
    _previousTurnMillis = 0;
}

void Car::setup() {
    // Setup all hardware components
    _motors.setup();
    _steering.setup();
    _button.setup();
    
    if (!_imu.setup()) {
        // If IMU fails, halt everything.
        _stopAndHalt();
    }

    Serial.println("All components initialized.");
    Serial.println("Press the button to start calibration and run...");

    _button.waitForPress(); // Wait for the start button
    Serial.println("Button pressed! Calibrating...");
    
    _imu.update(); // Get initial reading
    _offsetGyro = _imu.getHeading(); // Set initial offset
    _previousTurnMillis = millis(); // Initialize turn timer

    Serial.println("Calibration complete. Starting main loop.");
}

void Car::loop() {
    _imu.update(); // Always get the latest sensor data first
    
    _moveStraight();

    // Check for end condition
    if (_turnCounter >= 12) {
        // Your original code had a 1-second delay check here.
        if (millis() - _previousTurnMillis >= 1000) {
             _stopAndHalt();
        }
    }
}

void Car::_moveStraight() {
    // This method is a direct translation of your original moveStraight()
    
    if (_offsetGyro >= 360.0f) {
        _offsetGyro -= 360.0f;
    }
     if (_offsetGyro < 0.0f) {
        _offsetGyro += 360.0f;
    }

    float currentHeading = _imu.getHeading();
    float offsetGyroNow = currentHeading - _offsetGyro;

    if (offsetGyroNow > 180.0f) {
        offsetGyroNow -= 360.0f;
    } else if (offsetGyroNow < -180.0f) {
        offsetGyroNow += 360.0f;
    }

    float error = offsetGyroNow; // TargetOffset was 0 in your code
    float angle = error * STEERING_KP;

    _steering.setAngle(angle);
    _motors.forward(FORWARD_SPEED);
    _checkForTurns(); // This was 'turn()' in your code
}

void Car::_checkForTurns() {
    // Combined turn_Right and turn_left logic
    unsigned long currentMillis = millis();
    if (currentMillis - _previousTurnMillis < TURN_COOLDOWN_MS || _turnCounter > 11) {
        return;
    }

    // Check if we are driving relatively straight before allowing a turn
    float error = (imu.getHeading() - _offsetGyro);
    if(abs(error) < 15) {
        float frontDist = _distSensors.getFrontCm();
        if (frontDist > 1 && frontDist <= TURN_TRIGGER_DISTANCE_CM) {
            float rightDist = _distSensors.getRightCm();
            if (rightDist >= TURN_CLEARANCE_DISTANCE_CM || rightDist == 0) {
                _turnRight();
                return; // Prioritize right turn and exit
            }

            float leftDist = _distSensors.getLeftCm();
            if (leftDist >= TURN_CLEARANCE_DISTANCE_CM || leftDist == 0) {
                _turnLeft();
                return; // Turn left
            }
        }
    }
}

void Car::_turnRight() {
    Serial.println(">>> Turning RIGHT");
    _previousTurnMillis = millis();
    _turnCounter++;
    _offsetGyro += 90.0f;
}

void Car::_turnLeft() {
    Serial.println(">>> Turning LEFT");
    _previousTurnMillis = millis();
    _turnCounter++;
    _offsetGyro -= 90.0f;
}

void Car::_stopAndHalt() {
    _motors.stop();
    _steering.center();
    Serial.println("Execution halted.");
    while (true); // Loop forever
}
