#pragma once

#include "config.h"
#include "core/MotorController.h"
#include "core/Steering.h"
#include "core/DistanceSensors.h"
#include "core/IMU.h"
#include <Wire.h> 
#include "Adafruit_VL53L0X.h"
#include "core/Timer.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Servo servo;
// Instantiate objects specifically for testing
MotorController testMotors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN);
Steering testSteering(SERVO_PIN);
DistanceSensors testDistSensors(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT);
IMU testImu;

/**
 * @brief A simple blocking wait function, an alternative to delay().
 * Prints a message to the serial monitor.
 * @param duration_ms The time to wait in milliseconds.
 * @param message An optional message to print.
 */
void wait(unsigned long duration_ms, const char* message = "") {
    if (strlen(message) > 0) {
        Serial.print("  > Waiting ");
        Serial.print(duration_ms);
        Serial.print("ms (");
        Serial.print(message);
        Serial.println(")...");
    }
    delay(duration_ms);
}


void wait_for_button(const char* message) {
    Serial.println("----------------------------------------");
    Serial.print("Press button to start test: ");
    Serial.println(message);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    while(digitalRead(BUTTON_PIN) == HIGH) {
        delay(10);
    }
    Serial.println("...starting test.");
    wait(500, "Debouncing"); // Use our new wait function
}

void test_TOF()
{
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);
}

void test_wire()
{
  byte error, address;
  int nDevices = 0;

  delay(5000);

  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0x7f; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if (error != 2) {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  }
}

void test_motors() {
    wait_for_button("Motor Test");
    testMotors.setup();

    Serial.println("Moving FORWARD...");
    testMotors.forward(FORWARD_SPEED);
    wait(2000, "Forward Motion");

    Serial.println("STOPPING...");
    testMotors.stop();
    wait(1000, "Brake");

    Serial.println("Moving BACKWARD...");
    testMotors.backward(FORWARD_SPEED);
    wait(2000, "Backward Motion");

    testMotors.stop();
    Serial.println("Motor test complete.");
}

void test_steering() {
    wait_for_button("Steering Test");
    testSteering.setup();

    Serial.println("Turning LEFT...");
    testSteering.setAngle(SERVO_CENTER_ANGLE - 45);
    wait(2000, "Full Left");

    Serial.println("CENTERING...");
    testSteering.center();
    wait(2000, "Center");

    Serial.println("Turning RIGHT...");
    testSteering.setAngle(SERVO_CENTER_ANGLE + 45);
    wait(2000, "Full Right");
    
    testSteering.center();
    Serial.println("Steering test complete.");
}

void test_distance_sensors() {
    wait_for_button("Distance Sensor Test (10 seconds)");
    Serial.println("Move objects in front of sensors. Readings are in CM.");
    Serial.println("Time\tFront\tLeft\tRight");

    unsigned long startTime = millis();
    while(millis() - startTime < 100000) {
        float front = testDistSensors.getFrontCm();
        float left = testDistSensors.getLeftCm();
        float right = testDistSensors.getRightCm();

        Serial.print(millis() / 1000.0); Serial.print("s\t");
        Serial.print(front); Serial.print("\t");
        Serial.print(left); Serial.print("\t");
        Serial.println(right);
        wait(500); // Poll every half second
    }
    Serial.println("Distance sensor test complete.");
}

void test_imu() {
    wait_for_button("IMU Heading Test (10 seconds)");
    if (!testImu.setup()) {
        Serial.println("IMU failed to initialize. Test aborted.");
        return;
    }
    
    Serial.println("Calibrating IMU... Keep it flat and still.");
    wait(1000, "Sensor Settling");
    testImu.getHeading();
    
    Serial.println("Rotate the car. Heading should change relative to start.");
    Serial.println("Time\tHeading");
    
    unsigned long startTime = millis();
    while(millis() - startTime < 10000) {
        testImu.update();
        float heading = testImu.getHeading();
        
        Serial.print(millis() / 1000.0); Serial.print("s\t");
        Serial.println(heading);
        wait(250); // Poll every quarter second
    }
    Serial.println("IMU test complete.");
}


void runHardwareTests() {
    Serial.println("\n===== STARTING HARDWARE DIAGNOSTIC SUITE =====");
    
    // test_motors();
    // test_steering();
    // test_distance_sensors();
    test_imu();
    // test_wire();
    // test_TOF();
    
    Serial.println("\n===== ALL TESTS COMPLETE =====");
    Serial.println("Reset device to run again.");
    while(true) {
      // Loop forever
    }
}

