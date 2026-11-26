/* SenseStep haptic demo sender: two XIAO ESP32-S3 boards.
 * Holds CLEAR, FAR, NEAR and URGENT sample distances for four seconds each.
 * Arduino-ESP32 3.x; USB CDC On Boot enabled; Serial Monitor 115200.
 * No MAC address editing or router required. Both sketches use broadcast.
 * Intended for one test pair. Traffic is unencrypted and not paired.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_arduino_version.h>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstring>

#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "Install esp32 by Espressif Systems version 3.x in Boards Manager."
#endif

constexpr uint8_t NODE_ID = 1; // 1 = cane, 2 = wrist
const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint8_t WIFI_CHANNEL = 1; // Must match on both boards
constexpr uint32_t SEND_INTERVAL_MS = 100;
constexpr uint32_t ACK_TIMEOUT_MS = 200;
constexpr uint32_t STALE_TIMEOUT_MS = 500;
constexpr uint32_t MAGIC = 0x53535450;
constexpr uint8_t DATA_PACKET = 1;
constexpr uint8_t ACK_PACKET = 2;
static_assert(NODE_ID == 1 || NODE_ID == 2, "NODE_ID must be 1 or 2");

// Keep the 20-byte radio layout in sync across both ESP32s.
struct __attribute__((packed)) Packet {
  uint32_t magic;
  uint32_t sequence;
  uint32_t session; // Random sender boot ID, echoed by the receiver
  uint32_t senderMs;
  uint16_t distanceMm;
  uint8_t version;
  uint8_t type;
};
static_assert(sizeof(Packet) == 20, "Packet layout changed");

QueueHandle_t receiveQueue = nullptr;
uint32_t senderSession = 0;
uint32_t nextSequence = 0;
uint32_t pendingSequence = 0;
uint32_t pendingSentAt = 0;
uint32_t lastSentAt = 0;
uint32_t lastReceivedAt = 0;
uint32_t sentCount = 0;
uint32_t ackCount = 0;
uint32_t timeoutCount = 0;
bool awaitingAck = false;
bool linkStale = false;
uint8_t previousDemoStep = 255;
uint32_t demoStartedAt = 0;

void stopWithError(const char *message) {
  Serial.println(message);
  while (true) delay(1000);
}

void checkResult(esp_err_t result, const char *operation) {
  if (result != ESP_OK) {
    Serial.printf("%s failed: %s\n", operation, esp_err_to_name(result));
    stopWithError("Check settings and reset the board.");
  }
}

// Queue packets for processing outside the Wi-Fi task.
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data || len != sizeof(Packet)) return;
  Packet packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != MAGIC || packet.version != 1) return;
  if (packet.type != DATA_PACKET && packet.type != ACK_PACKET) return;
  // A full queue drops the packet; the sender will observe an ACK timeout.
  xQueueSend(receiveQueue, &packet, 0);
}

esp_err_t sendPacket(uint8_t type, uint32_t sequence,
                     uint16_t distanceMm, uint32_t senderMs, uint32_t session) {
  Packet packet = {MAGIC, sequence, session, senderMs, distanceMm, 1, type};
  return esp_now_send(BROADCAST_MAC, reinterpret_cast<const uint8_t *>(&packet),
                      sizeof(packet));
}

void setup() {
  Serial.begin(115200);
  const uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 4000) delay(10);

  if (!WiFi.mode(WIFI_STA)) stopWithError("Cannot start Wi-Fi station mode.");
  checkResult(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE),
              "Set channel");
  checkResult(esp_wifi_set_ps(WIFI_PS_NONE), "Disable Wi-Fi power saving");
  Serial.printf("\nSenseStep node %u (%s)\n", NODE_ID,
                NODE_ID == 1 ? "cane / sender" : "wrist / receiver");
  Serial.printf("My STA MAC: %s\n", WiFi.macAddress().c_str());
  Serial.printf("Channel: %u\n", WIFI_CHANNEL);

  senderSession = esp_random();

  receiveQueue = xQueueCreate(8, sizeof(Packet));
  if (!receiveQueue) stopWithError("Cannot allocate receive queue.");
  checkResult(esp_now_init(), "ESP-NOW init");
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false; // Bench test only; add encryption for the final system.
  checkResult(esp_now_add_peer(&peer), "Add peer");
  checkResult(esp_now_register_recv_cb(onReceive), "Register receive callback");
  lastSentAt = millis();
  lastReceivedAt = millis();
  demoStartedAt = millis();
  Serial.println("Ready. Data interval 100 ms; wrist stale timeout 500 ms.");
}

void loop() {
  Packet packet;
  while (xQueueReceive(receiveQueue, &packet, 0) == pdTRUE) {
    const uint32_t now = millis();
    if (NODE_ID == 2 && packet.type == DATA_PACKET) {
      lastReceivedAt = now;
      if (linkStale) {
        Serial.println("LINK RESTORED");
        linkStale = false;
      }
      // Echo the sequence and sender timestamp so node 1 can match the ACK.
      const esp_err_t result = sendPacket(ACK_PACKET, packet.sequence,
                                          packet.distanceMm, packet.senderMs, packet.session);
      Serial.printf("RX seq=%lu distance=%u mm; ACK %s\n",
                    (unsigned long)packet.sequence, packet.distanceMm,
                    result == ESP_OK ? "queued" : esp_err_to_name(result));
    } else if (NODE_ID == 1 && packet.type == ACK_PACKET && awaitingAck &&
               packet.sequence == pendingSequence && packet.senderMs == pendingSentAt &&
               packet.session == senderSession) {
      awaitingAck = false;
      ++ackCount;
      Serial.printf("ACK seq=%lu round-trip=%lu ms; acknowledged=%lu sent=%lu timeouts=%lu\n",
                    (unsigned long)packet.sequence,
                    (unsigned long)(now - pendingSentAt),
                    (unsigned long)ackCount, (unsigned long)sentCount,
                    (unsigned long)timeoutCount);
    }
  }

  const uint32_t now = millis();
  if (NODE_ID == 1) {
    if (awaitingAck && now - pendingSentAt >= ACK_TIMEOUT_MS) {
      awaitingAck = false;
      ++timeoutCount;
      Serial.printf("ACK TIMEOUT seq=%lu\n", (unsigned long)pendingSequence);
    }
    if (!awaitingAck && now - lastSentAt >= SEND_INTERVAL_MS) {
      lastSentAt = now;
      pendingSequence = ++nextSequence;
      pendingSentAt = now;
      // Simulated distance readings.
      const uint8_t demoStep = ((now - demoStartedAt) / 4000u) % 4u;
      const uint16_t demoDistances[] = {2000, 1100, 700, 300};
      const char *demoNames[] = {"CLEAR", "FAR", "NEAR", "URGENT"};
      const uint16_t distanceMm = demoDistances[demoStep];
      if (demoStep != previousDemoStep) {
        previousDemoStep = demoStep;
        Serial.printf("DEMO: %s, %u mm\n", demoNames[demoStep], distanceMm);
      }
      const esp_err_t result = sendPacket(DATA_PACKET, pendingSequence, distanceMm, now, senderSession);
      if (result == ESP_OK) {
        ++sentCount;
        awaitingAck = true;
      } else {
        Serial.printf("Send queue error: %s\n", esp_err_to_name(result));
      }
    }
  } else if (!linkStale && now - lastReceivedAt >= STALE_TIMEOUT_MS) {
    linkStale = true;
    Serial.println("LINK STALE: no valid cane data for 500 ms (fault state)");
  }
  delay(1);
}
