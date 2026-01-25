// funktioniert nicht mit zigbee und wifi gleichzeitig

#ifndef WIFI_AP_SERVER_H
#define WIFI_AP_SERVER_H

#include <WiFi.h>

bool WiFisetup(const char *ssid, const char *password);
void WiFiloop();
void WiFishutdown();

#endif // WIFI_AP_SERVER_H
