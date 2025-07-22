#pragma once

#include "config.h"
#include "core/MotorController.h"
#include "core/Steering.h"
#include "core/DistanceSensors.h"
#include "core/IMU.h"
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "core/Timer.h"
#include "core/PIDController.h"
#include <AS5600.h>
#include "core/Button.h"
#include "core/Car.h"
#include "core/Encoder.h"
#include "core/TOFSensor.h"




Servo servo;
Button button(BUTTON_PIN);
// Instantiate objects specifically for testing
MotorController testMotors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN);
Steering testSteering(SERVO_PIN);
DistanceSensors testDistSensors(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT);
IMU testImu;
Encoder encoder;
TOFSensor frontSensor(SHT_LOX, 0x20);
/**
 * @brief A simple blocking wait function, an alternative to delay().
 * Prints a message to the serial monitor.
 * @param duration_ms The time to wait in milliseconds.
 * @param message An optional message to print.
 */
void wait(unsigned long duration_ms, const char *message = "")
{
  if (strlen(message) > 0)
  {
    Serial.print("  > Waiting ");
    Serial.print(duration_ms);
    Serial.print("ms (");
    Serial.print(message);
    Serial.println(")...");
  }
  delay(duration_ms);
}

void test_TOF()
{
  button.waitForPress("TOF Test (10 seconds)");
  
  while (!Serial) delay(1);

    Serial.println("Starting TOF Sensor...");

    if (!frontSensor.begin()) {
        Serial.println("Sensor failed to init!");
        while (1);
    }

    Serial.println("TOF sensor initialized.");
    Timer timer;
    timer.start(100000);
    while(!timer.isFinished()){
    test_imu();
    uint16_t dist = frontSensor.readDistance();
    Serial.print(millis() / 1000.0);
    Serial.print("s\t");
    Serial.print("Distance: ");
    Serial.print(dist);
    Serial.println(" mm");}
}
void test_wire()
{
  button.waitForPress("wire Test (10 seconds)");
  byte error, address;
  int nDevices = 0;

  delay(1000);

  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    }
    else if (error != 2)
    {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found");
  }
}

void test_motors()
{
  button.waitForPress("Motor Test");
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

void test_encoder()
{
  button.waitForPress("encoder Test (10 seconds)");
  Wire.begin();

  if (!encoder.begin()) {
    Serial.println("Encoder not detected!");
    while (1);
  }

  unsigned long startTime = millis(); 
  while (millis() - startTime < 10000)
  {
    testMotors.forward(FORWARD_SPEED);
    encoder.update();
    Serial.print("Distance (cm): ");
    Serial.println(encoder.getDistanceCm());

    delay(100); // adjust as needed
  }

  testMotors.stop();
}

void test_steering()
{
  button.waitForPress("Steering Test");
  testSteering.setup();

  Serial.println("Turning LEFT...");
  testSteering.setAngle(-90);
  wait(2000, "Full Left");

  Serial.println("CENTERING...");
  testSteering.center();
  wait(2000, "Center");

  Serial.println("Turning RIGHT...");
  testSteering.setAngle(90);
  wait(2000, "Full Right");

  testSteering.center();
  Serial.println("Steering test complete.");
}

void test_distance_sensors()
{
  button.waitForPress("Distance Sensor Test (10 seconds)");
  Serial.println("Move objects in front of sensors. Readings are in CM.");
  Serial.println("Time\tFront\tLeft\tRight");

  unsigned long startTime = millis();
  while (millis() - startTime < 10000)
  {
    float front = testDistSensors.getFrontCm();
    float left = testDistSensors.getLeftCm();
    float right = testDistSensors.getRightCm();

    Serial.print(millis() / 1000.0);
    Serial.print("s\t");
    Serial.print(front);
    Serial.print("\t");
    Serial.print(left);
    Serial.print("\t");
    Serial.println(right);
    // wait(500); // Poll every half second
  }
  Serial.println("Distance sensor test complete.");
}

void test_imu()
{
  button.waitForPress("IMU Heading Test (10 seconds)");
  if (!testImu.setup())
  {
    Serial.println("IMU failed to initialize. Test aborted.");
    return;
  }

  Serial.println("Calibrating IMU... Keep it flat and still.");
  wait(1000, "Sensor Settling");
  testImu.getHeading();

  Serial.println("Rotate the car. Heading should change relative to start.");
  Serial.println("Time\tHeading");

  Timer timer;
  timer.start(10000);
  while (!timer.isFinished())
  {
    testImu.update();
    float heading = testImu.getHeading();

    Serial.print(millis() / 1000.0);
    Serial.print("s\t");
    Serial.println(heading);
    wait(250); // Poll every quarter second
  }

  button.waitForPress("Reset IMU");
  testImu.reset();

  timer.start(4000);
  while (!timer.isFinished())
  {
    testImu.update();
    float heading = testImu.getHeading();

    Serial.print(millis() / 1000.0);
    Serial.print("s\t");
    Serial.println(heading);
    wait(250); // Poll every quarter second
  }

  Serial.println("IMU test complete.");
}

void test_pid_controller()
{

  PIDController pid;
  pid.setup(0.9, 0, 0);
  pid.setOutputLimits(-45, 45);
  unsigned long startTime = millis();
  while (millis() - startTime < 10000)
  {
    float correction = pid.compute(0, 50);
    Serial.println(correction);
    correction = pid.compute(0, 0);
    Serial.println(correction);
    correction = pid.compute(0, -50);
    Serial.println(correction);
  }
}

void test_turn()
{
  Car myCar;
  myCar.setup();
  button.waitForPress("Turn 90");
  myCar._turn(90);
  
}

void runHardwareTests()
{

  Serial.println("\n===== STARTING HARDWARE DIAGNOSTIC SUITE =====");

  // test_motors();
  // test_steering();
  // test_distance_sensors();
  // test_wire();
  // test_imu();
  // test_encoder();
  test_TOF();
  // test_turn();

  Serial.println("\n===== ALL TESTS COMPLETE =====");
  Serial.println("Reset device to run again.");
  while (true)
  {
    // Loop forever
  }
}
