/* SenseStep wrist haptic receiver, XIAO ESP32-S3, Arduino-ESP32 3.x.
 * Compatible with board_a_sender and board_a_haptic_demo (20-byte packets).
 * Keep haptic_logic.h and logic_checks.h in this sketch folder.
 * D0/GPIO1 -> motor module IN only through a compatible input / buffer.
 * Motor VCC -> USB 5 V; motor GND -> XIAO GND. No DRV2605L needed.
 * This is a one-pair, unencrypted bench demo with simulated distance data.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_arduino_version.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstring>
#include "haptic_logic.h"
#include "logic_checks.h"

#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "Use esp32 by Espressif Systems version 3.x."
#endif

using SenseStepHaptics::State;
constexpr uint8_t MOTOR_PIN = D0;
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint32_t MAGIC = 0x53535450;
constexpr uint8_t DATA_PACKET = 1;
constexpr uint8_t ACK_PACKET = 2;
const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct __attribute__((packed)) Packet {
  uint32_t magic;
  uint32_t sequence;
  uint32_t session;
  uint32_t senderMs;
  uint16_t distanceMm;
  uint8_t version;
  uint8_t type;
};
static_assert(sizeof(Packet) == 20, "Must match the first-flash sender packet");

struct ReceivedEvent {
  Packet packet;
  uint8_t source[6];
  uint32_t arrivedAt;
};

QueueHandle_t receiveQueue = nullptr;
State currentState = State::Waiting;
uint32_t patternStartedAt = 0;
uint32_t linkStartedAt = 0;
uint32_t lastFreshPacketAt = 0;
uint32_t lastSequence = 0;
uint32_t activeSession = 0;
uint32_t retiredSession = 0;
uint8_t sourceMac[6] = {};
bool haveReading = false;
bool haveRetiredSession = false;
bool motorIsOn = false;

// Explicit declarations keep Arduino's generated prototypes after these types.
const char *stateName(State state);
void setState(State state, uint32_t now);
void sendAck(const Packet &received);

void stopWithError(const char *message) {
  digitalWrite(MOTOR_PIN, LOW);
  Serial.println(message);
  while (true) delay(1000);
}

void checkResult(esp_err_t result, const char *operation) {
  if (result != ESP_OK) {
    Serial.printf("%s failed: %s\n", operation, esp_err_to_name(result));
    stopWithError("Check settings and reset. Motor is OFF.");
  }
}

const char *stateName(State state) {
  switch (state) {
    case State::Clear: return "CLEAR";
    case State::Far: return "FAR";
    case State::Near: return "NEAR";
    case State::Urgent: return "URGENT";
    case State::LinkFault: return "LINK FAULT";
    default: return "WAITING";
  }
}

void setState(State state, uint32_t now) {
  if (state == currentState) return;
  currentState = state;
  patternStartedAt = now;
  Serial.printf("STATE: %s\n", stateName(state));
}

// Only copy here: the Wi-Fi task must not wait for a motor pulse to finish.
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data || len != sizeof(Packet)) return;
  ReceivedEvent event = {};
  memcpy(&event.packet, data, sizeof(Packet));
  if (event.packet.magic != MAGIC || event.packet.version != 1 ||
      event.packet.type != DATA_PACKET || event.packet.distanceMm == 0) return;
  memcpy(event.source, info->src_addr, sizeof(event.source));
  event.arrivedAt = millis();
  xQueueSend(receiveQueue, &event, 0);
}

void sendAck(const Packet &received) {
  Packet reply = received;
  reply.type = ACK_PACKET;
  const esp_err_t result = esp_now_send(BROADCAST_MAC,
      reinterpret_cast<const uint8_t *>(&reply), sizeof(reply));
  if (result != ESP_OK) Serial.printf("ACK queue error: %s\n", esp_err_to_name(result));
}

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  Serial.begin(115200);
  const uint32_t serialStartedAt = millis();
  while (!Serial && millis() - serialStartedAt < 3000) delay(10);
  if (!WiFi.mode(WIFI_STA)) stopWithError("Cannot start Wi-Fi; motor OFF.");
  checkResult(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE), "Channel");
  checkResult(esp_wifi_set_ps(WIFI_PS_NONE), "Disable Wi-Fi power saving");
  receiveQueue = xQueueCreate(8, sizeof(ReceivedEvent));
  if (!receiveQueue) stopWithError("Cannot allocate receive queue; motor OFF.");
  checkResult(esp_now_init(), "ESP-NOW init");
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  checkResult(esp_now_add_peer(&peer), "Add broadcast peer");
  checkResult(esp_now_register_recv_cb(onReceive), "Receive callback");
  linkStartedAt = millis();
  patternStartedAt = linkStartedAt;
  Serial.printf("Wrist haptics ready. STA MAC: %s; channel: %u; motor: D0/GPIO1\n",
                WiFi.macAddress().c_str(), WIFI_CHANNEL);
  Serial.println("First accepted sender is selected until reset. No data for 500 ms = fault double pulse.");
}

void loop() {
  ReceivedEvent event;
  while (xQueueReceive(receiveQueue, &event, 0) == pdTRUE) {
    const uint32_t now = millis();
    const Packet &packet = event.packet;
    if (SenseStepHaptics::linkStale(now, event.arrivedAt)) continue;
    if (haveReading && memcmp(event.source, sourceMac, sizeof(sourceMac)) != 0) continue;

    if (haveReading && packet.session == activeSession) {
      if (!SenseStepHaptics::newerSequence(packet.sequence, lastSequence)) {
        // A repeated packet can be acknowledged without extending freshness.
        if (packet.sequence == lastSequence) sendAck(packet);
        continue;
      }
    } else if (haveReading) {
      // Permit a sender restart, while rejecting the last retired boot session.
      // This is a bench convenience, not authenticated replay protection.
      if (haveRetiredSession && packet.session == retiredSession) continue;
      retiredSession = activeSession;
      haveRetiredSession = true;
    } else {
      memcpy(sourceMac, event.source, sizeof(sourceMac));
      Serial.println("Selected the first sender. Reset wrist to select a different board.");
    }

    if (currentState == State::LinkFault) Serial.println("LINK RESTORED");
    haveReading = true;
    activeSession = packet.session;
    lastSequence = packet.sequence;
    lastFreshPacketAt = event.arrivedAt;
    setState(SenseStepHaptics::classifyDistance(packet.distanceMm, currentState), now);
    sendAck(packet);
    Serial.printf("RX seq=%lu distance=%u mm; %s\n", (unsigned long)packet.sequence,
                  packet.distanceMm, stateName(currentState));
  }

  const uint32_t now = millis();
  const uint32_t lastActivity = haveReading ? lastFreshPacketAt : linkStartedAt;
  if (SenseStepHaptics::linkStale(now, lastActivity)) {
    if (currentState != State::LinkFault) Serial.println("LINK STALE: no fresh data for 500 ms");
    setState(State::LinkFault, now);
  }
  const bool shouldRun = SenseStepHaptics::motorOn(currentState, now - patternStartedAt);
  if (shouldRun != motorIsOn) {
    motorIsOn = shouldRun;
    digitalWrite(MOTOR_PIN, motorIsOn ? HIGH : LOW);
  }
  delay(1);
}
