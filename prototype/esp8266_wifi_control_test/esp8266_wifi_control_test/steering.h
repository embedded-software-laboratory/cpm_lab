#pragma once
#include <Arduino.h>

class Steering {
public:
  void setup();

  // throttle: 0 = max right, 127 = center, 255 = max left
  void set_command(uint8_t steering);
};

