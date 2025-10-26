#include "MotorAssist.h"

namespace MotorInternal {
  void motor_setup();
  void motor_loop();
}

namespace MotorTorque {

void Module::setup() {
  MotorInternal::motor_setup();
}

void Module::runTestStep() {
  MotorInternal::motor_loop();
}

} 
