#include "Car.h"
#include "../config.h"


float offsetGyro=0;
int offsetGyroNow;
sensors_event_t event;

Car::Car() :
    _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
    _steering(SERVO_PIN),
    _distSensors(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
    _imu()
{
    // Initialize navigation variables
    _targetHeading = 0.0;
    _turnCounter = 0;
    _lastTurnTime = 0;
    
}

void Car::setup() {
    // Setup all hardware components
   
    if (!_imu.setup()) {
        // If IMU fails, halt everything.
        stopAll();
        while(true);
    }
    _steering.setup();

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.println("All components initialized.");
    Serial.println("Press the button to start calibration and run...");

    // Wait for the start button to be pressed
    while(digitalRead(BUTTON_PIN) == HIGH) {
        delay(10);
    }
    Serial.println("Button pressed! Calibrating...");
    delay(500); // Debounce and wait

    // Calibrate sensors
    _imu.calibrate();
    offsetGyro=_imu.getHeading();
    _motors.setup();
    _targetHeading = 0.0;
    _lastTurnTime = millis();
    Serial.println("Calibration complete. Starting main loop.");
}

void Car::loop() {
    // The main execution block
    _imu.update(); // Always get the latest sensor data first
    
    navigate();

   

    // Check for end condition (e.g., 12 turns)
    if (_turnCounter >= 12) {
        Serial.println("Run complete (12 turns). Stopping.");
        stopAll();
        while(true); // Halt execution
    }
}

void Car::navigate() {
    // 1. Check for necessary turns
   if (offsetGyro == 360)
    {
        offsetGyro = 0;
    }
    offsetGyroNow = event.orientation.x - offsetGyro;
    if (offsetGyroNow > 180)
    {
        offsetGyroNow -= 360;
    }
    
    if (offsetGyroNow > -180)
    {
        offsetGyroNow += 360;
    }

    // 2. Implement PID controller for staying straight
    float currentError = offsetGyroNow - _targetHeading;

    float steeringAdjustment = currentError * STEERING_KP;

    Serial.println(event.orientation.x);

    
    _steering.setAngle(steeringAdjustment);

    // Serial.println(currentError);

    // Serial.print("Wall ahead! F: "); Serial.print(_distSensors.getFrontCm());
    // Serial.print(" R: "); Serial.print(_distSensors.getRightCm());
    // Serial.print(" L: "); Serial.println(_distSensors.getLeftCm());


    
    // 3. Move forward
    // _motors.forward(FORWARD_SPEED);

    if (currentError < abs(15)){
    checkForTurns();
}
}

void Car::checkForTurns() {
    // Don't check for another turn if we just made one
    if (millis() - _lastTurnTime < TURN_COOLDOWN_MS) {
        return;
    }

    float frontDist = _distSensors.getFrontCm();

    // Only consider turning if the front sensor reads a valid, close distance
    if (frontDist > 1 && frontDist <= TURN_DECISION_DISTANCE_CM) {
        float rightDist = _distSensors.getRightCm();
        float leftDist = _distSensors.getLeftCm();

    
        
        Serial.print("Wall ahead! F: "); Serial.print(frontDist);
        Serial.print(" R: "); Serial.print(rightDist);
        Serial.print(" L: "); Serial.println(leftDist);

        // Prefer turning right if there's space
        if (rightDist == 0 || rightDist > TURN_CLEARANCE_DISTANCE_CM) {
            Serial.println(">>> Turning RIGHT");
            offsetGyro += 90.0;
            _turnCounter++;
            _lastTurnTime = millis();
        } 
        // Otherwise, turn left if there's space
        else if (leftDist == 0 || leftDist > TURN_CLEARANCE_DISTANCE_CM) {
            Serial.println(">>> Turning LEFT");
            offsetGyro -= 90.0;
            _turnCounter++;
            _lastTurnTime = millis();
        }
    }
}

void Car::stopAll() {
    _motors.stop();
    _steering.center();
}