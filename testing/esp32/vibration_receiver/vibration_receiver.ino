// XIAO ESP32-S3, Arduino-ESP32 3.x, USB CDC On Boot enabled for logs.
// D10 (GPIO9) controls the active-high vibration motor driver input.
// Power the motor module through its VCC and GND connections.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstring>

constexpr uint8_t MOTOR_PIN = D10;
constexpr uint32_t PULSE_MS = 500;
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint8_t VIBRATE_COMMAND[] = {'S', 'S', 'V', 1};

SemaphoreHandle_t vibrationRequested = nullptr;
bool motorOn = false;
uint32_t pulseStartedAt = 0;

void checkResult(esp_err_t result, const char *operation) {
  if (result == ESP_OK) return;

  Serial.printf("%s failed: %s\n", operation, esp_err_to_name(result));
  while (true) delay(1000);
}

void onReceive(const esp_now_recv_info_t *, const uint8_t *data, int length) {
  if (!data || length != sizeof(VIBRATE_COMMAND)) return;
  if (memcmp(data, VIBRATE_COMMAND, sizeof(VIBRATE_COMMAND)) != 0) return;

  // The Wi-Fi callback only signals the Arduino loop.
  xSemaphoreGive(vibrationRequested);
}

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  Serial.begin(115200);

  vibrationRequested = xSemaphoreCreateBinary();
  if (!vibrationRequested) checkResult(ESP_ERR_NO_MEM, "Create vibration signal");
  if (!WiFi.mode(WIFI_STA)) checkResult(ESP_FAIL, "Start Wi-Fi station mode");
  checkResult(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE), "Set channel");
  checkResult(esp_wifi_set_ps(WIFI_PS_NONE), "Disable Wi-Fi power saving");
  checkResult(esp_now_init(), "Start ESP-NOW");
  checkResult(esp_now_register_recv_cb(onReceive), "Register receive callback");

  Serial.println("Receiver ready. Motor off. Waiting for an ESP-NOW command.");
}

void loop() {
  if (xSemaphoreTake(vibrationRequested, 0) == pdTRUE) {
    // A new command during a pulse restarts the 500 ms timer.
    pulseStartedAt = millis();
    digitalWrite(MOTOR_PIN, HIGH);
    motorOn = true;
    Serial.println("Vibration command received. D10 ON for 500 ms.");
  }

  if (motorOn && millis() - pulseStartedAt >= PULSE_MS) {
    digitalWrite(MOTOR_PIN, LOW);
    motorOn = false;
    Serial.println("D10 OFF.");
  }
  delay(1);
}
