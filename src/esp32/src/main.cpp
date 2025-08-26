#include <Wire.h> 

#include "core/ObstacleAvoider.h"
#include "core/OpenChallenge.h"

#ifdef RUN_TESTS
  #include "test/Tests.h"
#endif

// Objects
OpenChallenge openChallenge;
ObstacleAvoider robot;

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Modular Car Initializing ---");

  #ifdef RUN_TESTS
    Serial.println("!!! RUNNING IN TEST MODE !!!");
    runHardwareTests(); // loops forever
  #elif defined(OBSTACLE)
    Serial.println("--- Running in ObstacleAvoider Mode ---");
    robot.setup();
  #else
    Serial.println("--- Running in OpenChallenge Mode ---");
    openChallenge.setup();
  #endif

  Serial.println("Setup finished.");
}

void loop() {
  #ifdef RUN_TESTS
    // Nothing to do, tests already running in setup()
  #elif defined(OBSTACLE)
    robot.loop();
  #else
    openChallenge.loop();
  #endif
}
