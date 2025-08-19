#include "core/Car.h"
#include <Wire.h> 

// #define RUN_TESTS

#ifdef RUN_TESTS
  #include "test/Tests.h"
#endif

// Create our main car object
Car myCar;

//==============================================================================
// ARDUINO SETUP
//==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Modular Car Initializing ---");
  Wire.begin();

  #ifdef RUN_TESTS
    Serial.println("!!! RUNNING IN TEST MODE !!!");
    runHardwareTests(); // This function will loop forever, running tests.
  #else
    Serial.println("--- Running in Normal Operation Mode ---");
    myCar.setup();
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
    myCar.loop();
  #endif
}