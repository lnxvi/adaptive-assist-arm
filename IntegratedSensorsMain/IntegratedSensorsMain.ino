#include <Arduino.h>
#include "Accelerometer/Accelerometer.h"
#include "Myoware/Myoware.h"
#include "ForceSensors/ForceSensors.h"
//#include "PlottingCoeffs.h"  
#include "Accelerometer/Accelerometer.cpp"
#include "Accelerometer/AccelerometerInternal.cpp"
#include "Myoware/Myoware.cpp"
#include "Myoware/MyowareInternal.cpp"
#include "ForceSensors/ForceSensors.cpp"
#include "ForceSensors/ForceSensorsInternal.cpp"

// Instantiate modules
Accel::Module accel;
Myo::Module myo;
Force::Module force;

void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  // set up all sensors
  //accel.setup();
  // myo.setup();       
  force.setup();  

  // calibrate all sensors


}

void calibrate(){
  
}

void loop() {
  //accel.runTestStep();

  // myo.runTestStep();
  force.runTestStep();
}
