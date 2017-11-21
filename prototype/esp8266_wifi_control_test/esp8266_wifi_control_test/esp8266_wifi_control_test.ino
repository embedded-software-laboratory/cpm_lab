
#include "wifi.h"

Wifi wifi;
uint8_t led_state = 0;




void setup()
{
  Serial.begin(115200);
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);

  wifi.setup();
}


void get_command(uint8_t &steering, uint8_t &throttle) {
  static uint8_t latest_steering = 127;
  static uint8_t latest_throttle = 127;
  static long unsigned int timer_ms = 0;

  if(wifi.receive_command(latest_steering, latest_throttle)) {
    timer_ms = millis();
  }

  if(timer_ms + 150 < millis()) { // command is old, use safe default values
    latest_steering = 127;
    latest_throttle = 127;
  }
  
  steering = latest_steering;
  throttle = latest_throttle;
}




void loop()
{
  uint8_t steering = 0;
  uint8_t throttle = 0;
  get_command(steering, throttle);
  Serial.printf("%u, %u\n", steering, throttle);
  delay(5);
}


