

#include "wifi.h"

#define LOCAL_UDP_PORT 4210

void Wifi::setup() {
  
  // Wifi connection
  const char* ssid = "Buggy2";
  const char* password = "Lucky484";
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  // UDP setup
  Udp.begin(LOCAL_UDP_PORT);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), LOCAL_UDP_PORT);
  


  // mDNS setup  
  int chip_id = ESP.getChipId();
  snprintf(my_hostname, HOSTNAME_SZ-1, "esp8266_%i", chip_id);
  Serial.printf("Hostname: %s\n ", my_hostname);
  
  if (!MDNS.begin(my_hostname)) 
    Serial.println("mDNS responder started");
  else 
    Serial.println("Error setting up MDNS responder!");
}



bool Wifi::receive_command(uint8_t &steering, uint8_t &throttle) {
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    char incomingPacket[4];
    int len = Udp.read(incomingPacket, 3);
    if (len == 2)
    {
      steering = incomingPacket[0];
      throttle = incomingPacket[1];
      return true;
    }
  }
  return false;
}





