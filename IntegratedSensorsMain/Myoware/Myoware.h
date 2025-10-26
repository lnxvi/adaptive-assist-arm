#pragma once
#include <Arduino.h>

namespace Myo {
  class Module {
  public:
    void setup();
    void runTestStep();
    bool  isActive() const;      
    bool  isLiftActive() const;  
    float getEmgSmooth() const;
  };
} 
