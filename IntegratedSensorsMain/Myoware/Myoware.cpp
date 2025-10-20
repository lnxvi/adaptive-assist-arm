#include "Myoware.h"

namespace MyoInternal {
  void myo_setup();
  void myo_loop();
}

namespace Myo {
  void Module::setup() {
    MyoInternal::myo_setup();
  }
  void Module::runTestStep() {
    MyoInternal::myo_loop();
  }
} 
