#include "motor.h"

void Motor::setup() {
  
}

void Motor::set_command(uint8_t throttle) {
  Serial.printf("throttle: %u; ", throttle);
}

