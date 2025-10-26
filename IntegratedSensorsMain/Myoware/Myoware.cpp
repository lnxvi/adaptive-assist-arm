#include "Myoware.h"

namespace MyoInternal {
  void myo_setup();
  void myo_loop();
  bool  myo_is_active();
  bool  myo_is_lift();
  float myo_get_smooth();
}

namespace Myo {
  void Module::setup() {
    MyoInternal::myo_setup();
  }

  void Module::runTestStep() {
    MyoInternal::myo_loop();
  }

  bool Module::isActive() const { 
    return MyoInternal::myo_is_active(); 
  }

  bool Module::isLiftActive() const {
    return MyoInternal::myo_is_lift();
  }

  float Module::getEmgSmooth() const { 
    return MyoInternal::myo_get_smooth();
  }

} 
