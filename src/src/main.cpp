#include "core/Car.h"
#include <Wire.h> 

#include "core/ObstacleAvoider.h"
// #include "core/Garage.h"

#ifdef RUN_TESTS
  #include "test/Tests.h"
#endif

// Create our main car object
// Car myCar;


MotorController motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN);
Steering servo(SERVO_PIN);
Button button(BUTTON_PIN);
Encoder encoder;
IMU imu;
TOFSensor backSensor(SHT_LOX, 0x20);
Ultrasonic ultra(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT);
ObstacleAvoider robot;
OutParking outParking;

//==============================================================================
// ARDUINO SETUP
//==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Modular Car Initializing ---");
  // Wire.begin();
   Wire.begin(); 

    
    motors.setup();
    servo.setup();
    button.setup();
    encoder.begin();
    while (!imu.setup()) {
        Serial.println("FATAL: IMU failed to initialize.");
    }

    if (!backSensor.begin()) {
        Serial.println("TOF Sensor failed to init!");
        while (1);
    }

  #ifdef RUN_TESTS
    Serial.println("!!! RUNNING IN TEST MODE !!!");
    runHardwareTests(); // This function will loop forever, running tests.
  #else
    Serial.println("--- Running in Normal Operation Mode ---");
    // myCar.setup();
    // garage.begin();
    robot.attachHardware(&motors, &servo, &button, &encoder, &imu, &backSensor , &ultra);
    outParking.attachHardware(&motors, &servo, &encoder, &imu, &ultra);

    
    robot.setup();
    outParking.Do();

    
    Serial.println("finish parking");

    #endif
  }
  
  //==============================================================================
  // ARDUINO LOOP
  //==============================================================================
  void loop() {
    #ifdef RUN_TESTS
    // In test mode, all logic is in setup(), so the loop does nothing.
    #else
    // In normal mode, we run the car's main logic loop.
    // myCar.loop();
    robot.loop();
  #endif
}