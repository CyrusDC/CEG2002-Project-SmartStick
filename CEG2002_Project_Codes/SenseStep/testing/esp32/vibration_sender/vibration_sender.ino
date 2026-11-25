// XIAO ESP32-S3, Arduino-ESP32 3.x, USB CDC On Boot enabled.
// Trigger with v over USB Serial Monitor (115200 baud) or MSP430 UART (9600 8N1).
// MSP430 P3.3/TX -> XIAO D7/RX, and GND -> GND. Leave D6/TX disconnected.
// ESP-NOW broadcast is intended for a single test pair on channel 1.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

constexpr uint8_t WIFI_CHANNEL = 1;
constexpr unsigned long MSP430_BAUD = 9600;
constexpr uint8_t BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint8_t VIBRATE_COMMAND[] = {'S', 'S', 'V', 1};

void checkResult(esp_err_t result, const char *operation) {
  if (result == ESP_OK) return;

  Serial.printf("%s failed: %s\n", operation, esp_err_to_name(result));
  while (true) delay(1000);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(MSP430_BAUD, SERIAL_8N1, D7, D6);  // RX, TX

  if (!WiFi.mode(WIFI_STA)) checkResult(ESP_FAIL, "Start Wi-Fi station mode");
  checkResult(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE), "Set channel");
  checkResult(esp_wifi_set_ps(WIFI_PS_NONE), "Disable Wi-Fi power saving");
  checkResult(esp_now_init(), "Start ESP-NOW");

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  checkResult(esp_now_add_peer(&peer), "Add broadcast peer");

  Serial.println("Sender ready. Send v over USB or press S1 on the MSP430 test board.");
}

void handleTrigger(Stream &port, const char *source) {
  while (port.available() > 0) {
    const int input = port.read();
    if (input != 'v' && input != 'V') continue;

    const esp_err_t result = esp_now_send(BROADCAST_MAC, VIBRATE_COMMAND,
                                          sizeof(VIBRATE_COMMAND));
    if (result == ESP_OK) {
      Serial.printf("%s: Vibration command queued for broadcast.\n", source);
    } else {
      Serial.printf("%s: Could not send vibration command: %s\n", source,
                    esp_err_to_name(result));
    }
  }
}

void loop() {
  handleTrigger(Serial, "USB");
  handleTrigger(Serial1, "MSP430 UART");
  delay(1);
}
