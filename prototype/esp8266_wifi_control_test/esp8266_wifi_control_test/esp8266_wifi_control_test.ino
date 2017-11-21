
#include "wifi.h"
#include "motor.h"
#include "steering.h"


Wifi wifi;
Motor motor;
Steering steering;


void setup()
{
  Serial.begin(115200);
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);

  wifi.setup();
  motor.setup();
  steering.setup();
}


void loop()
{
  uint8_t steering_value = 0;
  uint8_t throttle_value = 0;
  wifi.get_command(steering_value, throttle_value);
  motor.set_command(throttle_value);
  steering.set_command(steering_value);
  Serial.write("\n");
  delay(5);
}


