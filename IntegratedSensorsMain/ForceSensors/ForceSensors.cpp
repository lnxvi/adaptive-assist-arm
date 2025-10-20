#include "ForceSensors.h"

namespace ForceInternal {
  void force_setup();
  void force_loop();
}

namespace Force {
  void Module::setup() {
    ForceInternal::force_setup();
  }
  void Module::runTestStep() {
    ForceInternal::force_loop();
  }
} 
