#include "motor.h"

void Motor::setup() {
  analogWriteFreq(50);
  pinMode(LED_BUILTIN, OUTPUT);
}

void Motor::set_command(uint8_t throttle) {

  if(throttle == 255)  analogWrite(LED_BUILTIN, 85);
  else if(throttle == 0)  analogWrite(LED_BUILTIN, 50);
  else analogWrite(LED_BUILTIN, 105);
  //Serial.printf("throttle: %u; ", throttle);
}

