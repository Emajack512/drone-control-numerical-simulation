#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"

void setup() {
  Serial.begin(115200);
  delay(300);

  WiFi.mode(WIFI_MODE_STA);      // Inicializa Wi-Fi
  delay(50);                     // Pequeña espera

  uint8_t mac_sta[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac_sta);

  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac_sta[0], mac_sta[1], mac_sta[2], mac_sta[3], mac_sta[4], mac_sta[5]);

  Serial.print("WiFi STA MAC: ");
  Serial.println(buf);
}

void loop() {}
