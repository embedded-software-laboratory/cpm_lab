#pragma once
#include <Arduino.h>
#include <Servo.h> 

class Steering {
  Servo myservo;
  
public:
  void setup();

  // throttle: 0 = max right, 127 = center, 255 = max left
  void set_command(uint8_t steering);
};

