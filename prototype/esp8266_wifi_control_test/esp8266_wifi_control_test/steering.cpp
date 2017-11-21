#include "steering.h"

void Steering::setup() {
  myservo.attach(2); 
}

void Steering::set_command(uint8_t steering) {
  myservo.write(map(steering,0,255,125,57));
  //Serial.printf("steering: %u; ", steering);  
}

