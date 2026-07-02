#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>

const char* ssid = "Divyansh123";
const char* password = "sgth2314";

IPAddress local_IP(10, 90, 17, 34);
IPAddress gateway(10, 90, 17, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiUDP udp;
const unsigned int UDP_PORT = 5005;

SoftwareSerial bridge(D6, D5);

void setup() {
  Serial.begin(115200);
  bridge.begin(9600);

  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  udp.begin(UDP_PORT);
}

void loop() {
  int packetSize = udp.parsePacket();

  if (packetSize > 0) {
    char cmd = (char)udp.read();

    while (udp.available()) {
      udp.read();
    }

    if (cmd == 'W' || cmd == 'A' || cmd == 'S' || cmd == 'D' || cmd == 'X') {
      bridge.write(cmd);
    }
  }
}