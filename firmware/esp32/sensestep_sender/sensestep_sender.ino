/*
 * SenseStep cane sender for the XIAO ESP32-S3.
 * Author: Alfred (A).
 * Version control: see this sketch's Git history.
 * Board configuration: Arduino-ESP32 3.3.12, UART RX on D7.
 * setup() opens the MSP430 UART and the USB diagnostic port.
 * The sender uses ESP-NOW on Wi-Fi channel 1 in station mode.
 * The two configured wrist MAC addresses map to left and right readings.
 * readMsp430() aligns to the frame marker and checks the XOR and range.
 * A valid reading replaces the pending update for its side.
 * sendNextWrist() alternates between peers and drops stale updates.
 * A send callback releases the in-flight packet slot after transmission.
 * The Arduino loop repeats UART parsing and wireless forwarding.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <atomic>
#include <cstring>

// Enter the Wi-Fi MAC printed by each wrist ESP. A zero address skips that wrist.
constexpr uint8_t WRIST_MACS[2][6] = {
    {0, 0, 0, 0, 0, 0},  /* Configure the left wrist station MAC (A). */
    {0, 0, 0, 0, 0, 0},  /* Configure the right wrist station MAC (A). */
};
constexpr uint8_t WIFI_CHANNEL = 1;  /* Set the shared ESP-NOW channel (A). */
constexpr uint8_t UART_START = 0xA5;  /* Match the MSP430 frame marker (A). */
constexpr uint32_t MAX_PENDING_AGE_MS = 200;  /* Discard pending readings older than 200 ms (A). */
const char *const SIDE_NAMES[] = {"LEFT", "RIGHT"};  /* Provide side labels for diagnostics (A). */

struct WristUpdate {
  uint16_t distanceMm;  /* Store the latest side distance in millimeters (A). */
  uint32_t receivedAt;  /* Record when the UART frame arrived (A). */
  bool waiting;  /* Track an update waiting for radio transmission (A). */
};

WristUpdate updates[2] = {};  /* Keep one pending update per wrist (A). */
bool peerReady[2] = {};  /* Track which wrists registered successfully (A). */
uint8_t nextPeer = 0;  /* Start with the left wrist as the first send candidate (A). */
std::atomic<bool> sending{false};  /* Share the transmission state with the send callback (A). */

// Return for ESP_OK. Otherwise print the failing operation and result and
// stop setup so that the loop cannot run with a partially initialized radio.
void checkResult(esp_err_t result, const char *operation) {
  if (result == ESP_OK) return;
  Serial.printf("%s failed: %s\n", operation, esp_err_to_name(result));  /* Report a failed radio initialization step (A). */
  while (true) delay(1000);  /* Stop here after an initialization failure (A). */
}

// ESP-NOW send callback. The SDK supplies send metadata and status.
// The callback releases the send slot for either completion status.
void onSent(const esp_now_send_info_t *, esp_now_send_status_t) {
  sending.store(false);  /* Release the transmission slot (A). */
}

// Register the ESP-NOW peer for side (0 = left, 1 = right).
// Leave that side inactive if its MAC is missing, invalid or rejected.
void addWrist(uint8_t side) {
  constexpr uint8_t ZERO_MAC[6] = {};  /* Use the all-zero address as the unconfigured sentinel (A). */
  const uint8_t *mac = WRIST_MACS[side];  /* Get the configured address for this side (A). */
  if (memcmp(mac, ZERO_MAC, sizeof(ZERO_MAC)) == 0 || (mac[0] & 1)) {  /* Reject unconfigured and multicast addresses (A). */
    Serial.printf("Set a valid %s MAC in WRIST_MACS.\n", SIDE_NAMES[side]);
    return;
  }

  esp_now_peer_info_t peer = {};  /* Start with an empty peer configuration (A). */
  memcpy(peer.peer_addr, mac, sizeof(peer.peer_addr));  /* Set the wrist's destination MAC (A). */
  peer.channel = WIFI_CHANNEL;  /* Use the channel shared by both wrists (A). */
  peer.ifidx = WIFI_IF_STA;  /* Send using the Wi-Fi station interface (A). */
  const esp_err_t result = esp_now_add_peer(&peer);  /* Register the peer with ESP-NOW (A). */
  peerReady[side] = result == ESP_OK;  /* Queue updates only for registered peers (A). */
  if (!peerReady[side]) {
    Serial.printf("Could not add %s wrist: %s\n", SIDE_NAMES[side], esp_err_to_name(result));  /* Report a peer registration failure (A). */
  }
}

// Parse available Serial1 bytes without waiting on a partial frame.
// Takes no arguments. Store the newest valid reading for each enabled peer.
void readMsp430() {
  static uint8_t payload[4];  /* Hold the side, two distance fields and checksum (A). */
  static size_t used = sizeof(payload);  /* Wait for the first start marker (A). */

  while (Serial1.available()) {
    const uint8_t byte = static_cast<uint8_t>(Serial1.read());  /* Read the next available UART byte (A). */
    if (byte == UART_START) {
      used = 0;  /* Restart frame assembly at the marker (A). */
      continue;
    }
    if (used == sizeof(payload)) continue;  /* Ignore bytes until a frame marker starts a new frame (A). */
    if (byte & 0x80) {
      used = sizeof(payload);  /* Discard a frame containing an invalid payload byte (A). */
      continue;
    }
    payload[used++] = byte;  /* Append this byte to the current frame (A). */
    if (used != sizeof(payload)) continue;  /* Wait until all four payload bytes arrive (A). */

    const uint8_t side = payload[0];  /* Read the wrist destination from the frame (A). */
    const uint16_t distanceMm = payload[1] | (uint16_t(payload[2]) << 7);  /* Join the 7-bit fields (A). */
    const uint8_t checksum = side ^ payload[1] ^ payload[2];  /* Recompute the frame checksum (A). */
    if (side > 1 || payload[3] != checksum || distanceMm > 4000 ||
        (distanceMm != 0 && distanceMm < 20)) continue;  /* Reject invalid sides, checksums or distance values (A). */

    if (peerReady[side]) updates[side] = {distanceMm, millis(), true};  /* Keep the newest reading (A). */
    if (Serial) Serial.printf("%s %u mm%s\n", SIDE_NAMES[side], distanceMm,
                              peerReady[side] ? "" : " (wrist MAC missing or invalid)");  /* Log the decoded reading and any missing peer (A). */
  }
}

// Send at most one fresh pending update to its matching wrist peer.
// Takes no arguments. Continue from the other peer after each send attempt.
void sendNextWrist() {
  if (sending.load()) return;  /* Wait for the previous send callback (A). */

  for (uint8_t i = 0; i < 2; ++i) {
    const uint8_t side = (nextPeer + i) % 2;  /* Try the preferred wrist first and then the other wrist (A). */
    WristUpdate &update = updates[side];  /* Use the pending update for this side (A). */
    if (!update.waiting) continue;
    update.waiting = false;  /* Consume the queued update before attempting the send (A). */
    if (millis() - update.receivedAt >= MAX_PENDING_AGE_MS) continue;  /* Drop stale updates (A). */

    /* Pack the SSD identifier, version and little-endian distance (A). */
    const uint8_t packet[] = {'S', 'S', 'D', 1, uint8_t(update.distanceMm),
                              uint8_t(update.distanceMm >> 8)};  /* Finish packing the little-endian distance (A). */
    nextPeer = 1 - side;  /* Give the other wrist first chance next time (A). */
    sending.store(true);  /* Reserve the slot before the callback can run (A). */
    const esp_err_t result = esp_now_send(WRIST_MACS[side], packet, sizeof(packet));  /* Queue the radio packet (A). */
    if (result != ESP_OK) {
      sending.store(false);  /* Release the send slot if the SDK rejected the call (A). */
      if (Serial) Serial.printf("%s send error: %s\n", SIDE_NAMES[side], esp_err_to_name(result));  /* Log the error from the failed send request (A). */
    }
    return;  /* Attempt at most one send in this pass (A). */
  }
}

// Arduino startup hook. Takes no arguments and initializes both serial ports,
// the radio, the send callback and the configured wrist peers.
void setup() {
  Serial.begin(115200);  /* Start the USB diagnostic port (A). */
  const uint32_t startedAt = millis();
  while (!Serial && millis() - startedAt < 2000) delay(10);  /* Allow up to two seconds for USB serial to connect (A). */
  Serial1.begin(9600, SERIAL_8N1, D7, D6);  /* Match the MSP430 UART settings (A). */

  if (!WiFi.mode(WIFI_STA)) checkResult(ESP_FAIL, "Start Wi-Fi station mode");  /* Start the Wi-Fi station interface (A). */
  checkResult(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE), "Set channel");  /* Set the shared ESP-NOW channel (A). */
  checkResult(esp_wifi_set_ps(WIFI_PS_NONE), "Disable Wi-Fi power saving");  /* Keep the radio awake for ESP-NOW (A). */
  checkResult(esp_now_init(), "Start ESP-NOW");  /* Initialize ESP-NOW before adding peers (A). */
  checkResult(esp_now_register_send_cb(onSent), "Register send callback");  /* Track send completion (A). */
  addWrist(0);  /* Register the left wrist (A). */
  addWrist(1);  /* Register the right wrist (A). */
  Serial.println("Cane sender ready. Waiting for MSP430 distance frames on D7.");  /* Tell the user the sender is ready for UART data (A). */
}

// Arduino foreground loop. Takes no arguments and services both links.
void loop() {
  readMsp430();  /* Parse new MSP430 distance frames (A). */
  sendNextWrist();  /* Forward one fresh reading over ESP-NOW (A). */
  delay(1);  /* Yield before the next pass (A). */
}
