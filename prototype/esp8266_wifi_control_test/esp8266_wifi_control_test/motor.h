#pragma once
#include <Arduino.h>

class Motor {
  int speed = 0;
public:
  void setup();

  // throttle: 0 = max reverse, 127 = standstill, 255 = max forward
  void set_command(uint8_t throttle);
  void set_speed(int spd);
};

