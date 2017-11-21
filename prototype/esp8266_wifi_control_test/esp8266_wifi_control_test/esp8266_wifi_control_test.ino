

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>

const char* ssid = "Buggy2";
const char* password = "Lucky484";

char mDNS_name[50];

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
char replyPacekt[] = "Hi there! Got the message :-)";  // a reply string to send back
uint8_t led_state = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println();

  int chip_id = ESP.getChipId();
  snprintf(mDNS_name, 49, "esp8266_%i", chip_id);
  Serial.printf("mDNS_name %s\n ", mDNS_name);

  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  pinMode(LED_BUILTIN, OUTPUT);

  if (!MDNS.begin(mDNS_name)) {             // Start the mDNS responder for esp8266.local
    Serial.println("Error setting up MDNS responder!");
  }
  else Serial.println("mDNS responder started");
}


void loop()
{
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      Serial.printf("%u, %u\n", incomingPacket[0], incomingPacket[1]);

      // send back a reply, to the IP address and port we got the packet from
      /*Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(replyPacekt);
        Udp.endPacket();*/
      led_state = ~led_state;
      digitalWrite(LED_BUILTIN, led_state);
    }
  }
}
