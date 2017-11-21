
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


void loop()
{
  uint8_t steering = 0;
  uint8_t throttle = 0;
  if(wifi.receive_command(steering, throttle)) {    
      Serial.printf("%u, %u\n", steering, throttle);
      led_state = ~led_state;
      digitalWrite(LED_BUILTIN, led_state);
  }
}


