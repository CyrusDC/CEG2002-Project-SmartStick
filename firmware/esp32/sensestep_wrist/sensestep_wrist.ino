/*
 * SenseStep wrist receiver for the XIAO ESP32-S3.
 * Author: Alfred (A).
 * Version control: see this sketch's Git history.
 * Upload this sketch to each configured wrist board.
 * setup() holds the motor off on D10 and creates a one-entry receive queue.
 * The receiver starts ESP-NOW on Wi-Fi channel 1 in station mode.
 * onReceive() checks the packet header, length and distance range.
 * The Wi-Fi callback hands the latest valid reading to the Arduino loop.
 * The loop uses distance bands and hysteresis to choose a vibration rate.
 * A zero reading or stale link stops vibration.
 * Send m over USB serial at 115200 baud to display the station MAC.
 * haptic_logic.h defines the pulse rules and logic_checks.h checks them.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstring>
#include "haptic_logic.h"
#include "logic_checks.h"

using SenseStepHaptics::State;
constexpr uint8_t MOTOR_PIN = D10;  /* Select D10 for the vibration motor driver (A). */
constexpr uint8_t WIFI_CHANNEL = 1;  /* Match the sender's Wi-Fi channel (A). */
constexpr uint8_t PACKET_HEADER[] = {'S', 'S', 'D', 1};  /* Define the SSD identifier and packet version (A). */

struct Reading {
  uint32_t receivedAt;  /* Keep the reading's local arrival timestamp (A). */
  uint16_t distanceMm;  /* Keep the decoded distance in millimeters (A). */
};

QueueHandle_t receiveQueue = nullptr;  /* Hold the queue shared by the radio callback and loop (A). */
State state = State::Waiting;  /* Start in the quiet waiting state (A). */
uint32_t stateStartedAt = 0;  /* Track when the current warning state started (A). */
uint32_t lastReadingAt = 0;  /* Track when the last valid reading arrived (A). */
bool motorOn = false;  /* Cache the last output level sent to the motor (A). */

// Return for ESP_OK. Otherwise turn the motor off, report the result for
// operation and halt before the loop can start.
void checkResult(esp_err_t result, const char *operation) {
  if (result == ESP_OK) return;
  digitalWrite(MOTOR_PIN, LOW);  /* Keep the motor output off (A). */
  Serial.printf("%s failed: %s\n", operation, esp_err_to_name(result));  /* Print the setup failure and SDK error (A). */
  while (true) delay(1000);  /* Keep the receiver halted after a setup failure (A). */
}

// ESP-NOW callback. The SDK supplies sender metadata, data and its byte length.
// Validate the payload and publish the distance and timestamp to the queue.
void onReceive(const esp_now_recv_info_t *, const uint8_t *data, int length) {
  if (!data || length != sizeof(PACKET_HEADER) + 2 ||
      memcmp(data, PACKET_HEADER, sizeof(PACKET_HEADER)) != 0) return;  /* Reject packets with an invalid length or header (A). */

  const uint16_t distanceMm = data[4] | (uint16_t(data[5]) << 8);  /* Decode millimeters (A). */
  if (distanceMm > 4000 || (distanceMm != 0 && distanceMm < 20)) return;  /* Reject out-of-range values (A). */

  const Reading reading = {millis(), distanceMm};  /* Timestamp the received reading (A). */
  xQueueOverwrite(receiveQueue, &reading);  /* Hand the newest reading to the loop (A). */
}

// Print this wrist's station MAC for sender configuration. Takes no arguments.
void printMac() {
  Serial.printf("Wrist STA MAC: %s\n", WiFi.macAddress().c_str());  /* Print the MAC to enter into the sender configuration (A). */
}

// Arduino startup hook. Takes no arguments. Set up motor GPIO, USB serial,
// the receive queue and the ESP-NOW receiver before reporting the wrist MAC.
void setup() {
  pinMode(MOTOR_PIN, OUTPUT);  /* Configure D10 as a motor driver output (A). */
  digitalWrite(MOTOR_PIN, LOW);  /* Hold the motor off at startup (A). */
  Serial.begin(115200);  /* Start the USB diagnostic port (A). */
  const uint32_t startedAt = millis();
  while (!Serial && millis() - startedAt < 2000) delay(10);  /* Allow up to two seconds for USB serial to connect (A). */

  receiveQueue = xQueueCreate(1, sizeof(Reading));  /* Keep one pending reading (A). */
  if (!receiveQueue) checkResult(ESP_ERR_NO_MEM, "Create receive queue");  /* Halt if the receive queue could not be created (A). */
  if (!WiFi.mode(WIFI_STA)) checkResult(ESP_FAIL, "Start Wi-Fi station mode");  /* Start the Wi-Fi station interface (A). */
  checkResult(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE), "Set channel");  /* Tune the radio to the sender's channel (A). */
  checkResult(esp_wifi_set_ps(WIFI_PS_NONE), "Disable Wi-Fi power saving");  /* Keep the radio awake to receive updates (A). */
  checkResult(esp_now_init(), "Start ESP-NOW");  /* Initialize ESP-NOW before registering the callback (A). */
  checkResult(esp_now_register_recv_cb(onReceive), "Register receive callback");  /* Receive distance packets (A). */
  printMac();  /* Display this wrist's station MAC (A). */
  Serial.println("Wrist ready. Send m to print its MAC again.");  /* Show how to request the MAC again (A). */
}

// Arduino foreground loop. Takes no arguments. Apply the newest valid reading
// to the haptic state and update the motor according to its elapsed pulse time.
void loop() {
  Reading reading;
  const bool received = xQueueReceive(receiveQueue, &reading, 0) == pdTRUE;  /* Read the queue without blocking (A). */
  const uint32_t now = millis();  /* Read the current time for timeout and pulse timing (A). */
  State nextState = state;  /* Keep the current warning unless a reading or timeout changes it (A). */

  if (received && !SenseStepHaptics::linkStale(now, reading.receivedAt)) {
    /* Reset the hysteresis history when the link resumes (A). */
    const State previous = SenseStepHaptics::linkStale(now, lastReadingAt) ? State::Waiting : state;  /* Use the current band for hysteresis only while the link is fresh (A). */
    lastReadingAt = reading.receivedAt;  /* Refresh the link timeout reference (A). */
    nextState = reading.distanceMm == 0 ? State::Clear
                                        : SenseStepHaptics::classifyDistance(reading.distanceMm, previous);  /* Clear on zero or classify the distance using hysteresis (A). */
  } else if (SenseStepHaptics::linkStale(now, lastReadingAt)) {
    nextState = State::Waiting;  /* Stop vibration after the receive timeout (A). */
  }

  if (nextState != state) {
    state = nextState;  /* Apply the newly selected warning state (A). */
    stateStartedAt = now;  /* Start a new pulse period when the warning changes (A). */
  }
  const bool shouldVibrate = SenseStepHaptics::motorOn(state, now - stateStartedAt);  /* Evaluate the pulse pattern at the current elapsed time (A). */
  if (shouldVibrate != motorOn) {
    motorOn = shouldVibrate;  /* Save the new output level before driving the motor (A). */
    digitalWrite(MOTOR_PIN, motorOn ? HIGH : LOW);  /* Update the haptic motor driver (A). */
  }

  if (Serial.available() && Serial.read() == 'm') printMac();  /* Print the station MAC on request (A). */
  delay(1);  /* Yield before checking for more packets (A). */
}
