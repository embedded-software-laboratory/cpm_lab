#include "steering.h"

void Steering::setup() {
  
}

void Steering::set_command(uint8_t steering) {
  Serial.printf("steering: %u; ", steering);  
}

