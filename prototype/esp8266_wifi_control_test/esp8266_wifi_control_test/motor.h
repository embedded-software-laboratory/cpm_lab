#pragma once
#include <Arduino.h>

class Motor {
public:
  void setup();

  // throttle: 0 = max reverse, 127 = standstill, 255 = max forward
  void set_command(uint8_t throttle);
};

