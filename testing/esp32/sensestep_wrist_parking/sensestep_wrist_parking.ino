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

constexpr uint32_t LINK_TIMEOUT_MS = 500;
constexpr uint16_t MAX_ALERT_MM = 1300;
constexpr uint16_t MAX_RELEASE_MM = 1375;
constexpr uint16_t STEADY_MM = 150;
constexpr uint16_t STEADY_RELEASE_MM = 175;
constexpr uint32_t SHORT_PULSE_MS = 125;
constexpr uint32_t LONG_PULSE_MS = 400;
constexpr uint32_t SHORT_GAP_MS = 125;
constexpr uint32_t LONG_GAP_MS = 1100;

class Pattern {
 public:
  bool update(uint16_t distanceMm, uint32_t now) {
    const uint16_t alertLimit = mode == Mode::Silent ? MAX_ALERT_MM : MAX_RELEASE_MM;
    const uint16_t steadyLimit = mode == Mode::Steady ? STEADY_RELEASE_MM : STEADY_MM;
    if (distanceMm == 0 || distanceMm > alertLimit) {
      mode = Mode::Silent;
      return false;
    }
    if (distanceMm <= steadyLimit) {
      mode = Mode::Steady;
      return true;
    }

    const uint16_t boundedMm = distanceMm > MAX_ALERT_MM ? MAX_ALERT_MM : distanceMm;
    const uint32_t offset = boundedMm - STEADY_MM;
    const uint32_t span = MAX_ALERT_MM - STEADY_MM;
    const uint32_t gapMs = SHORT_GAP_MS + offset * (LONG_GAP_MS - SHORT_GAP_MS) / span;

    if (mode == Mode::Pulse && pulsing) {
      if (now - phaseStartedAt >= pulseMs) {
        phaseStartedAt = now;
        pulsing = false;
      }
      return pulsing;
    }

    // Update the gap as the target moves. Keep each active pulse intact.
    if (mode != Mode::Pulse || now - phaseStartedAt >= gapMs) {
      mode = Mode::Pulse;
      phaseStartedAt = now;
      pulseMs = SHORT_PULSE_MS + offset * (LONG_PULSE_MS - SHORT_PULSE_MS) / span;
      pulsing = true;
    }
    return pulsing;
  }

 private:
  enum class Mode : uint8_t { Silent, Pulse, Steady };
  Mode mode = Mode::Silent;
  bool pulsing = false;
  uint32_t phaseStartedAt = 0;
  uint32_t pulseMs = 0;
};

}  // namespace SenseStepHaptics

constexpr uint8_t MOTOR_PIN = D10;
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint8_t PACKET_HEADER[] = {'S', 'S', 'D', 1};

struct Reading {
  uint32_t receivedAt;
  uint16_t distanceMm;
};

QueueHandle_t receiveQueue = nullptr;
SenseStepHaptics::Pattern vibration;
Reading latestReading = {};
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
  if (received && now - reading.receivedAt < SenseStepHaptics::LINK_TIMEOUT_MS) {
    latestReading = reading;
  }

  if (now - latestReading.receivedAt >= SenseStepHaptics::LINK_TIMEOUT_MS) {
    latestReading.distanceMm = 0;
  }

  const bool shouldVibrate = vibration.update(latestReading.distanceMm, now);
  if (shouldVibrate != motorOn) {
    motorOn = shouldVibrate;
    digitalWrite(MOTOR_PIN, motorOn ? HIGH : LOW);
  }

  if (Serial.available() && Serial.read() == 'm') printMac();
  delay(1);
}
