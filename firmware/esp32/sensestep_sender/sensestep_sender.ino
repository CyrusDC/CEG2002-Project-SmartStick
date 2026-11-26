// Author: Alfred
// XIAO ESP32-S3 cane sender. Tested with Arduino-ESP32 3.3.12.
// MSP430 P3.3/TX -> D7/RX, GND -> GND. USB Serial Monitor: 115200 baud.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <atomic>
#include <cstring>

// Enter the Wi-Fi MAC printed by each wrist ESP. A zero address skips that wrist.
constexpr uint8_t WRIST_MACS[2][6] = {
    {0, 0, 0, 0, 0, 0},  // Left
    {0, 0, 0, 0, 0, 0},  // Right
};
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint8_t UART_START = 0xA5;
constexpr uint32_t MAX_PENDING_AGE_MS = 200;
const char *const SIDE_NAMES[] = {"LEFT", "RIGHT"};

struct WristUpdate {
  uint16_t distanceMm;
  uint32_t receivedAt;
  bool waiting;
};

WristUpdate updates[2] = {};
bool peerReady[2] = {};
uint8_t nextPeer = 0;
std::atomic<bool> sending{false};

void checkResult(esp_err_t result, const char *operation) {
  if (result == ESP_OK) return;
  Serial.printf("%s failed: %s\n", operation, esp_err_to_name(result));
  while (true) delay(1000);
}

void onSent(const esp_now_send_info_t *, esp_now_send_status_t) {
  sending.store(false);
}

void addWrist(uint8_t side) {
  constexpr uint8_t ZERO_MAC[6] = {};
  const uint8_t *mac = WRIST_MACS[side];
  if (memcmp(mac, ZERO_MAC, sizeof(ZERO_MAC)) == 0 || (mac[0] & 1)) {
    Serial.printf("Set a valid %s MAC in WRIST_MACS.\n", SIDE_NAMES[side]);
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, sizeof(peer.peer_addr));
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  const esp_err_t result = esp_now_add_peer(&peer);
  peerReady[side] = result == ESP_OK;
  if (!peerReady[side]) {
    Serial.printf("Could not add %s wrist: %s\n", SIDE_NAMES[side], esp_err_to_name(result));
  }
}

void readMsp430() {
  static uint8_t payload[4];
  static size_t used = sizeof(payload);  // Wait for the first marker.

  while (Serial1.available()) {
    const uint8_t byte = static_cast<uint8_t>(Serial1.read());
    if (byte == UART_START) {
      used = 0;
      continue;
    }
    if (used == sizeof(payload)) continue;
    if (byte & 0x80) {
      used = sizeof(payload);
      continue;
    }
    payload[used++] = byte;
    if (used != sizeof(payload)) continue;

    const uint8_t side = payload[0];
    const uint16_t distanceMm = payload[1] | (uint16_t(payload[2]) << 7);
    const uint8_t checksum = side ^ payload[1] ^ payload[2];
    if (side > 1 || payload[3] != checksum || distanceMm > 4000 ||
        (distanceMm != 0 && distanceMm < 20)) continue;

    if (peerReady[side]) updates[side] = {distanceMm, millis(), true};
    if (Serial) Serial.printf("%s %u mm%s\n", SIDE_NAMES[side], distanceMm,
                              peerReady[side] ? "" : " (wrist MAC missing or invalid)");
  }
}

void sendNextWrist() {
  if (sending.load()) return;

  for (uint8_t i = 0; i < 2; ++i) {
    const uint8_t side = (nextPeer + i) % 2;
    WristUpdate &update = updates[side];
    if (!update.waiting) continue;
    update.waiting = false;
    if (millis() - update.receivedAt >= MAX_PENDING_AGE_MS) continue;

    // ESP-NOW payload: ASCII SSD, version 1, little-endian millimeters.
    const uint8_t packet[] = {'S', 'S', 'D', 1, uint8_t(update.distanceMm),
                              uint8_t(update.distanceMm >> 8)};
    nextPeer = 1 - side;
    sending.store(true);
    const esp_err_t result = esp_now_send(WRIST_MACS[side], packet, sizeof(packet));
    if (result != ESP_OK) {
      sending.store(false);
      if (Serial) Serial.printf("%s send error: %s\n", SIDE_NAMES[side], esp_err_to_name(result));
    }
    return;
  }
}

void setup() {
  Serial.begin(115200);
  const uint32_t startedAt = millis();
  while (!Serial && millis() - startedAt < 2000) delay(10);
  Serial1.begin(9600, SERIAL_8N1, D7, D6);

  if (!WiFi.mode(WIFI_STA)) checkResult(ESP_FAIL, "Start Wi-Fi station mode");
  checkResult(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE), "Set channel");
  checkResult(esp_wifi_set_ps(WIFI_PS_NONE), "Disable Wi-Fi power saving");
  checkResult(esp_now_init(), "Start ESP-NOW");
  checkResult(esp_now_register_send_cb(onSent), "Register send callback");
  addWrist(0);
  addWrist(1);
  Serial.println("Cane sender ready. Waiting for MSP430 distance frames on D7.");
}

void loop() {
  readMsp430();
  sendNextWrist();
  delay(1);
}
