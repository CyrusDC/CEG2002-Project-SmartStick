# SenseStep: first ESP-NOW bench test

For a broadcast test that needs no MAC address edits, pair
[`board_a_sender.ino`](../board_a_sender/board_a_sender.ino) with
[`board_b_receiver.ino`](../board_b_receiver/board_b_receiver.ino).
This sketch uses its own configurable MAC-based protocol.

The DRV2605L and LRA chain below records the original proposal.
See the [repository README](../../../README.md) for the submission firmware
and wrist wiring.

Two Seeed Studio XIAO ESP32-S3 boards, using Arduino IDE and the Espressif
Arduino-ESP32 3.x board package. No extra ESP-NOW library is needed.

## How this fits the proposal

The SenseStep proposal uses this chain:

```text
MSP430F5529 --UART wires--> cane XIAO --ESP-NOW radio--> wrist XIAO
                                                        |
                                                       I2C
                                                        |
                                                   DRV2605L + LRA
```

ESP-NOW is the link between the two ESP32 boards. The MSP430 does not speak
ESP-NOW directly. This sketch tests the radio link before adding UART,
sensors, or haptics. Node 1 sends simulated distances; node 2 sends an
application acknowledgement (ACK). An ACK proves the receiver's application
handled a packet, beyond merely queuing a send request.

## 1. Hardware

- Label the boards A (cane) and B (wrist).
- Attach the supplied Wi-Fi antenna to each board, with power disconnected.
- Use USB data cables to power/program them. Start about 0.5-1 m apart.
- No wires are needed between the two ESP32 boards. No MSP430 is needed yet.
- A router, internet connection, SSID, and password are not needed for the test.

## 2. Arduino IDE

1. In File > Preferences, add this Additional Boards Manager URL if needed:
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. In Boards Manager, install **esp32 by Espressif Systems**, a stable **3.x**
   version. Use the same version for both boards and record that version.
3. Select **XIAO_ESP32S3** from the ESP32 boards and select the connected COM port.
4. Set **USB CDC On Boot > Enabled** if that menu is available. Leave other
   settings at the board defaults for this test.
5. Open `espnow_link_test.ino` from this folder. Arduino requires the sketch
   and its containing folder to have the same name, as provided here.
6. Use Serial Monitor at **115200 baud**. Close any other monitor holding the
   same COM port before uploading.

If uploading does not connect, hold BOOT, press and release RESET, then
release BOOT. Select the newly appearing port and upload again. Press RESET
after uploading if the application does not start, then reselect the port
if it changes.

## 3. Get the two MAC addresses

Leave `PEER_MAC` as all zeroes initially.

1. Upload to board A. Open Serial Monitor and press RESET.
2. Copy the address printed after `My STA MAC:`. Label it **A MAC**.
3. Repeat with board B and record **B MAC**.

The sketch deliberately stops after printing the MAC when `PEER_MAC` is
blank. That message is expected during discovery. A MAC identifies a radio
interface; use the station (STA) addresses printed by this sketch.

## 4. Configure each board

Edit the two lines near the top of the same sketch before each upload:

| Upload to | NODE_ID | PEER_MAC |
| --- | --- | --- |
| A, the cane | `1` | B's STA MAC |
| B, the wrist | `2` | A's STA MAC |

Example conversion only, not a real address for your boards:

```cpp
// If the OTHER board prints AA:BB:CC:12:34:56:
const uint8_t PEER_MAC[6] = {0xAA, 0xBB, 0xCC, 0x12, 0x34, 0x56};
```

Keep `WIFI_CHANNEL = 1` on both. Both boards add the other as a peer because
both send packets: data in one direction and ACKs in the other. Do not add
`WiFi.begin()` or connect to an access point during this test.

You can configure/upload A, then edit and upload B from the same sketch.
Changing the file on your PC does not change firmware already uploaded to A.
Record the two MACs and roles so later uploads use the correct settings.

## 5. What to expect

On B, expect lines resembling:

```text
RX seq=1 distance=350 mm; ACK queued
RX seq=2 distance=450 mm; ACK queued
```

On A, expect lines resembling (timing is illustrative):

```text
ACK seq=1 round-trip=4 ms; acknowledged=1 sent=1 timeouts=0
ACK seq=2 round-trip=3 ms; acknowledged=2 sent=2 timeouts=0
```

The normal interval is 100 ms. The sender waits for an ACK before sending
again and times out after 200 ms, so the rate falls during packet loss.
`ACK queued` on B means its reply was accepted for transmission. Only a
matching `ACK` on A confirms that the round trip completed. Round-trip time
includes application scheduling and is not one-way radio latency.

Keep B's Serial Monitor open and disconnect power from A. About 500 ms
after the last valid data packet, B should print:

```text
LINK STALE: no valid cane data for 500 ms (fault state)
```

Reconnect A. B should print `LINK RESTORED` and resume receiving. Also
disconnect B while watching A: it should report `ACK TIMEOUT`. Test each
board resetting independently, then test the intended distance and body
placement. Log sent/acknowledged counts, timeouts, and observed round-trip
times for the report. A timeout can mean either data or its ACK was lost;
it is not a measurement of one-way packet loss.

If there is no reception, check the OTHER board's MAC, board roles, matching
channel, antenna connections, and the selected USB ports. If Serial Monitor
is blank, check USB CDC On Boot, baud rate, USB data cable, and RESET. A
receive-callback compilation error often means an old 2.x board package or
old tutorial callback was used; this sketch targets 3.x.

## 6. Add MSP430 UART after the radio test passes

For an MSP430F5529 using USCI_A0, a candidate connection is:

| MSP430F5529 signal | Cane XIAO ESP32-S3 |
| --- | --- |
| P3.3 / UCA0TXD | D7 / GPIO44 / RX |
| P3.4 / UCA0RXD | D6 / GPIO43 / TX |
| GND | GND |

Confirm those pins are exposed and unused by your CEG2002 Booster before
wiring. Configure the MSP430 pins for their UART peripheral functions.
Use 3.3 V logic and a common ground; do not apply 5 V to either UART input.
Power each development board normally for the bench test; a common ground
does not require connecting their power outputs together.

On the cane XIAO, the later UART initialization can be:

```cpp
Serial1.begin(9600, SERIAL_8N1, D7, D6); // RX, then TX
```

Use the same baud rate, 8 data bits, no parity, and one stop bit on the
MSP430. Start at 9600 for a short text test, then choose the final rate from
the actual packet size and sample rate. USB `Serial` remains the debug log.

First send a fixed newline-terminated message from the MSP430 and verify
it on the XIAO. Then replace simulated distances with parsed UART readings.
Define an explicit UART frame with units, sequence, sensor validity, and
framing/checksum as needed. Do not memcpy an MSP430 C struct onto the wire:
integer sizes and padding can differ from the ESP32. The 16-byte struct in
this sketch is only for the two ESP32s running the same code.

For the final wrist firmware, distinguish an obstacle warning, valid clear
space, sensor fault, and stale link. Track the age of actual sensor data as
well as radio packets: a live radio must not hide a stalled MSP430 sending
no fresh readings. Duplicate/replayed data should not refresh freshness.
This bench sketch accepts valid packets from the configured peer and does
not implement production freshness, retry, haptic, or encryption behavior.

## Verification status

Prepared against the official API and pin references below. The ESP32
3.3.12 package is now installed. This MAC-based alternative has not been
compiled or tested on hardware. Both uploads and the disconnect/recovery
checks remain bench verification.

## Official references

- [Seeed XIAO ESP32-S3 setup and antenna](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
- [Seeed XIAO pin map and Serial1](https://wiki.seeedstudio.com/xiao_esp32s3_pin_multiplexing/)
- [Espressif ESP-NOW API, channels, callbacks and ACK guidance](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/network/esp_now.html)
- [TI MSP430F5529 datasheet](https://www.ti.com/lit/gpn/MSP430F5529)
