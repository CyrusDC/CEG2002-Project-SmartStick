// Upload this sketch to both XIAO ESP32-S3 wrist boards.
// D10 -> motor module IN. Open Serial Monitor at 115200 and send m for its MAC.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstring>

namespace SenseStepHaptics {

enum class State : uint8_t { Waiting, Clear, Far, Near, Urgent };
constexpr uint32_t LINK_TIMEOUT_MS = 500;
constexpr uint16_t URGENT_MM = 450;
constexpr uint16_t NEAR_MM = 850;
constexpr uint16_t FAR_MM = 1300;
constexpr uint16_t RELEASE_MARGIN_MM = 75;
constexpr uint32_t FAR_PULSE_MS = 400;
constexpr uint32_t FAR_PERIOD_MS = 1500;

// Values are bench examples, not validated obstacle-warning thresholds.
// Escalate immediately; require extra clearance when reducing a warning.
constexpr State classifyDistance(uint16_t distanceMm, State previous) {
  const State candidate = distanceMm <= URGENT_MM ? State::Urgent
                        : distanceMm <= NEAR_MM ? State::Near
                        : distanceMm <= FAR_MM ? State::Far : State::Clear;
  if (previous == State::Urgent && distanceMm <= URGENT_MM + RELEASE_MARGIN_MM)
    return State::Urgent;
  if (candidate == State::Urgent) return candidate;
  if (previous == State::Near && distanceMm <= NEAR_MM + RELEASE_MARGIN_MM)
    return State::Near;
  if (candidate == State::Near) return candidate;
  if (previous == State::Far && distanceMm <= FAR_MM + RELEASE_MARGIN_MM)
    return State::Far;
  return candidate;
}

constexpr bool motorOn(State state, uint32_t elapsedMs) {
  if (state == State::Far) return elapsedMs % FAR_PERIOD_MS < FAR_PULSE_MS;
  if (state == State::Near) return elapsedMs % 600 < 180;
  if (state == State::Urgent) return elapsedMs % 350 < 200;
  return false;
}

constexpr bool linkStale(uint32_t now, uint32_t lastFreshPacketAt) {
  return uint32_t(now - lastFreshPacketAt) >= LINK_TIMEOUT_MS;
}

} // namespace SenseStepHaptics


using SenseStepHaptics::State;
constexpr uint8_t MOTOR_PIN = D10;
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint8_t PACKET_HEADER[] = {'S', 'S', 'D', 1};

struct Reading {
  uint32_t receivedAt;
  uint16_t distanceMm;
};

QueueHandle_t receiveQueue = nullptr;
State state = State::Waiting;
uint32_t stateStartedAt = 0;
uint32_t lastReadingAt = 0;
bool motorOn = false;

void checkResult(esp_err_t result, const char *operation) {
  if (result == ESP_OK) return;
  digitalWrite(MOTOR_PIN, LOW);
  Serial.printf("%s failed: %s\n", operation, esp_err_to_name(result));
  while (true) delay(1000);
}

void onReceive(const esp_now_recv_info_t *, const uint8_t *data, int length) {
  if (!data || length != sizeof(PACKET_HEADER) + 2 ||
      memcmp(data, PACKET_HEADER, sizeof(PACKET_HEADER)) != 0) return;

  const uint16_t distanceMm = data[4] | (uint16_t(data[5]) << 8);
  if (distanceMm > 4000 || (distanceMm != 0 && distanceMm < 20)) return;

  const Reading reading = {millis(), distanceMm};
  xQueueOverwrite(receiveQueue, &reading);
}

void printMac() {
  Serial.printf("Wrist STA MAC: %s\n", WiFi.macAddress().c_str());
}

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  Serial.begin(115200);
  const uint32_t startedAt = millis();
  while (!Serial && millis() - startedAt < 2000) delay(10);

  receiveQueue = xQueueCreate(1, sizeof(Reading));
  if (!receiveQueue) checkResult(ESP_ERR_NO_MEM, "Create receive queue");
  if (!WiFi.mode(WIFI_STA)) checkResult(ESP_FAIL, "Start Wi-Fi station mode");
  checkResult(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE), "Set channel");
  checkResult(esp_wifi_set_ps(WIFI_PS_NONE), "Disable Wi-Fi power saving");
  checkResult(esp_now_init(), "Start ESP-NOW");
  checkResult(esp_now_register_recv_cb(onReceive), "Register receive callback");
  printMac();
  Serial.println("Wrist ready. Send m to print its MAC again.");
}

void loop() {
  Reading reading;
  const bool received = xQueueReceive(receiveQueue, &reading, 0) == pdTRUE;
  const uint32_t now = millis();
  State nextState = state;

  if (received && !SenseStepHaptics::linkStale(now, reading.receivedAt)) {
    // Fresh packets resume from a clear state after a lost link.
    const State previous = SenseStepHaptics::linkStale(now, lastReadingAt) ? State::Waiting : state;
    lastReadingAt = reading.receivedAt;
    nextState = reading.distanceMm == 0 ? State::Clear
                                        : SenseStepHaptics::classifyDistance(reading.distanceMm, previous);
  } else if (SenseStepHaptics::linkStale(now, lastReadingAt)) {
    nextState = State::Waiting;
  }

  if (nextState != state) {
    state = nextState;
    stateStartedAt = now;
  }
  const bool shouldVibrate = SenseStepHaptics::motorOn(state, now - stateStartedAt);
  if (shouldVibrate != motorOn) {
    motorOn = shouldVibrate;
    digitalWrite(MOTOR_PIN, motorOn ? HIGH : LOW);
  }

  if (Serial.available() && Serial.read() == 'm') printMac();
  delay(1);
}
