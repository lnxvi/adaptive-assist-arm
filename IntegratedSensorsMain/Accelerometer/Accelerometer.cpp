#include "Accelerometer.h"

namespace AccelInternal {
  void accel_setup();
  void accel_loop();
}

namespace Accel {
  void Module::setup() {
    AccelInternal::accel_setup();
  }
  void Module::runTestStep() {
    AccelInternal::accel_loop();
  }
}
