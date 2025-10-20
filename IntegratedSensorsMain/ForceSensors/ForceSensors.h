#pragma once
#include <Arduino.h>

namespace Force {
  class Module {
  public:
    void setup();
    void runTestStep();
  };
} 
