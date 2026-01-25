// funktioniert nicht mit zigbee und wifi gleichzeitig

#include "WiFi_AP_Server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

NetworkServer server(80);

bool WiFisetup(const char *ssid, const char *password) {
  Serial.println("\n=== WiFi AP Setup ===");

  // Ensure WiFi is off first
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_OFF);
  delay(100);

  // Set to AP mode
  if (!WiFi.mode(WIFI_AP)) {
    Serial.println("Failed to set WiFi to AP mode");
    return false;
  }

  delay(200);

  Serial.print("Creating WiFi Access Point: ");
  Serial.println(ssid);

  // Minimal softAP configuration - just SSID and password
  bool result = WiFi.softAP(ssid, password);

  Serial.print("softAP result: ");
  Serial.println(result);

  if (!result) {
    Serial.println("Failed to create Access Point");
    return false;
  }

  delay(2000); // Give AP significant time to stabilize

  Serial.println("Access Point created successfully!");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("AP SSID: ");
  Serial.println(ssid);

  server.begin();

  return true;
}

void WiFiloop() {
  static NetworkClient activeClient;
  static bool clientActive = false;
  static String currentLine;

  if (!clientActive) {
    NetworkClient newClient = server.accept();
    if (newClient) {
      activeClient = newClient;
      clientActive = true;
      currentLine = "";
      Serial.println("New Client.");
    } else {
      return;
    }
  }

  if (!activeClient.connected()) {
    activeClient.stop();
    clientActive = false;
    currentLine = "";
    Serial.println("Client Disconnected.");
    return;
  }

  while (activeClient.available()) {
    char c = activeClient.read();
    Serial.write(c);

    if (c == '\n') {
      if (currentLine.length() == 0) {
        activeClient.println("HTTP/1.1 200 OK");
        activeClient.println("Content-type:text/html");
        activeClient.println();

        activeClient.print(
            "Click <a href=\"/H\">here</a> to turn the LED on pin 5 on.<br>");
        activeClient.print(
            "Click <a href=\"/L\">here</a> to turn the LED on pin 5 off.<br>");

        activeClient.println();
        activeClient.stop();
        clientActive = false;
        currentLine = "";
        Serial.println("Client Disconnected.");
        break;
      } else {
        currentLine = "";
      }
    } else if (c != '\r') {
      currentLine += c;
    }

    if (currentLine.endsWith("GET /H")) {
    }
    if (currentLine.endsWith("GET /L")) {
    }
  }
}

void WiFishutdown() {
  server.close();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
}