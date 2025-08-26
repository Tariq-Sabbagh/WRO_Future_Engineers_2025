// #include "core/Car.h"
#include <Wire.h> 

#include "core/ObstacleAvoider.h"
#include "core/OpenChallenge.h"
// #include "core/Garage.h"

#ifdef RUN_TESTS
  #include "test/Tests.h"
#endif

// Create our main car object
// Car myCar;
OpenChallenge openchallenge;



ObstacleAvoider robot;
// OutParking outParking;

//==============================================================================
// ARDUINO SETUP
//==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Modular Car Initializing ---");
 
  #ifdef RUN_TESTS
    Serial.println("!!! RUNNING IN TEST MODE !!!");
    runHardwareTests(); // This function will loop forever, running tests.
  #else
    Serial.println("--- Running in Normal Operation Mode ---");
    // myCar.setup();
    openchallenge.setup();
    // garage.begin();
    // robot.attachHardware(&motors, &servo, &button, &encoder, &imu, &backSensor , &ultra);
    // outParking.attachHardware(&motors, &servo, &encoder, &imu, &ultra);

    
    // robot.setup();
    

    
    Serial.println("setup finish");

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
    // robot.loop();
    openchallenge.loop();
  #endif
}