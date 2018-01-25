#pragma once
#include <Arduino.h>

class SpeedSensor {
public:
  void setup();
  int get_speed();
};

