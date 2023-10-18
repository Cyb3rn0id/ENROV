// use this code for retrieving the ESP32 MAC Address
// open terminal to 115200bps and copy the result
#include <WiFi.h>
 
void setup()
  {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  }
 
void loop()
  {
  Serial.println(WiFi.macAddress());
  delay(1000);
  }