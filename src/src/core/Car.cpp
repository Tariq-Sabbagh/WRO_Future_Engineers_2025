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
    error = 0.0f;
}

void Car::setup() {
    // Setup all hardware components
    _motors.setup();
    _steering.setup();
    _button.setup();
    _pid.setup(STEERING_KP, 0, 0); // Example values for kp, ki, kd
    _pid.setOutputLimits(45, 135);
    
    if (!_imu.setup()) {
        // If IMU fails, halt everything.
        _stopAndHalt();
    }

    Serial.println("All components initialized.");
    Serial.println("Press the button to start calibration and run...");

    _button.waitForPress(); // Wait for the start button
    Serial.println("Button pressed! Calibrating...");
    Timer::wait(1500);
    // delay(1500);
    
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
    // _checkForTurns(); 
    
    float currentHeading = _imu.getHeading();
    float error = currentHeading ;
    Serial.print(error);
    Serial.print(" ");
    Serial.print(currentHeading);
    Serial.print(" ");
    Serial.println(_offsetGyro);


    
    // angle = 0;
    // _steering.setAngle(angle);
    _motors.forward(FORWARD_SPEED);
}

void Car::_checkForTurns() {
    // Combined turn_Right and turn_left logic
    unsigned long currentMillis = millis();
    if (currentMillis - _previousTurnMillis < TURN_COOLDOWN_MS || _turnCounter > 11) {
        return;
    }

    Serial.print(" left");Serial.print(_distSensors.getLeftCm());
    Serial.print(" right");Serial.print(_distSensors.getRightCm());
    Serial.print(" front");Serial.println(_distSensors.getFrontCm());
    
    if(abs(error) < 15) {
        if(_turnCounter < 1)
        {
            // _turnLeft();
            _turnRight();
        }

        else
        {
            if(direction=="right")
            {
                _turnRight();
            }
            else if(direction=="left")
            {
                _turnLeft();
            }
        }
       

           
        }
    }


void Car::_turnRight() {
     float frontDist = _distSensors.getFrontCm();
        if (frontDist > 1 && frontDist <= TURN_TRIGGER_DISTANCE_CM) {
            float rightDist = _distSensors.getRightCm();
            if (rightDist >= TURN_CLEARANCE_DISTANCE_CM || rightDist == 0) {
                direction ="right";
                Serial.println(">>> Turning RIGHT");
                _previousTurnMillis = millis();
                _turnCounter++;
                _offsetGyro += 90.0f;
            }
        }
}

void Car::_turnLeft() {
     float leftDist = _distSensors.getLeftCm();
            if (leftDist >= TURN_CLEARANCE_DISTANCE_CM || leftDist == 0) {
                direction ="left";
                Serial.println(">>> Turning LEFT");
                _previousTurnMillis = millis();
                _turnCounter++;
                _offsetGyro -= 90.0f;
              
            }
}

void Car::_stopAndHalt() {
    _motors.stop();
    _steering.center();
    Serial.println("Execution halted.");
    while (true); // Loop forever
}
