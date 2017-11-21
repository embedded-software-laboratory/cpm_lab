#pragma once

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>



#define HOSTNAME_SZ 50

class Wifi {
  
  char my_hostname[HOSTNAME_SZ];
  WiFiUDP Udp;

public:
  
  void setup();
  bool receive_command(uint8_t &steering, uint8_t &throttle);
  void get_command(uint8_t &steering, uint8_t &throttle);
};

