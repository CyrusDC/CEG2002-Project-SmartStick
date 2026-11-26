# SmartStick / SenseStep

**CEG2002 Embedded System Design · CEG2001 · Singapore Institute of Technology**

SmartStick is a foldable walking-stick prototype with left and right ultrasonic ranging, a water-sensor buzzer, and motorized sensor covers. The MSP430F5529 sends distance readings over UART to the cane's ESP32. The sender relays each side's reading over ESP-NOW to the corresponding wrist receiver, which varies its vibration rate with distance.

## Submission firmware

| Program | Source |
| --- | --- |
| MSP430 cane | [`main.c`](firmware/msp430/main.c) |
| ESP32 sender | [`sensestep_sender.ino`](firmware/esp32/sensestep_sender/sensestep_sender.ino) |
| ESP32 wrist receiver | [`sensestep_wrist.ino`](firmware/esp32/sensestep_wrist/sensestep_wrist.ino) and its two headers in the [receiver sketch folder](firmware/esp32/sensestep_wrist) |

Component tests and earlier prototypes are documented in the [testing guide](testing/README.md).

## How the final firmware works

1. At startup, the MSP430 enters low-power mode (LPM3). Timer_A2 wakes it every 250 ms to sample the FSR. A reading above 500 ADC counts wakes the cane and opens the sensor covers.
2. While the grip is held, both HC-SR04 sensors share one trigger. Timer_A1 times the two echo pulses. A valid distance below 200 mm lights the corresponding onboard LED. The buzzer sounds when the water sensor reads above 1,000 ADC counts.
3. After eight consecutive released-grip checks, the MSP430 clears the LEDs and buzzer, sends zero distances to the ESP32, closes the covers, and returns to LPM3.

The cane sends a five-byte UART frame for each side at 9600 baud, 8N1. A frame contains `0xA5`, the side (`0` for left, `1` for right), the low and high 7-bit fields of the distance in millimeters, and the XOR of the side and distance fields. A zero distance clears the alert or indicates an invalid measurement.

## Final-build MSP430F5529 wiring

| MSP430 pin | Part | Signal or connection |
| --- | --- | --- |
| P1.2 (TA0.1) | Left MG90S servo | PWM signal |
| P1.3 (TA0.2) | Right MG90S servo | PWM signal |
| P2.0 | Active buzzer | Buzzer +. Buzzer − connects to GND. |
| P3.5 | Both HC-SR04 sensors | Shared TRIG |
| P2.5 | Left HC-SR04 | ECHO through a 10 kΩ / 15 kΩ voltage divider |
| P1.5 | Right HC-SR04 | ECHO through a 10 kΩ / 15 kΩ voltage divider |
| P3.3 (UCA0TXD) | Cane-side ESP32 | Connect to the sender sketch's D7 / RX. UART uses 9600 baud, 8N1. |
| P7.0 (A12) | Water sensor | Signal S through a 10 kΩ / 15 kΩ voltage divider |
| P6.0 (A0) | FSR grip sensor | Junction between the FSR and the 10 kΩ pull-down |
| P1.0 | Onboard red LED | Left obstacle indicator |
| P4.7 | Onboard green LED | Right obstacle indicator |

The MSP430 source header records ESP RX as GPIO 18. The submitted sender sketch sets RX to D7, which is the pin shown in this wiring table.

**Power and ground**

- **3.3 V** supplies the FSR. Wire the FSR between 3.3 V and the ADC input, and the 10 kΩ pull-down between the ADC input and ground.
- **5 V** supplies the two HC-SR04 modules, water-sensor module, and servo power connections.
- **Common ground** connects the MSP430, XIAO, sensors, servos, buzzer, and voltage dividers.

## Build and upload

1. In Code Composer Studio, create an MSP430F5529 project and add `firmware/msp430/main.c` as its entry point.
2. Open the ESP32 sketches in Arduino IDE. The checked-in sketches target the `XIAO_ESP32S3` board with Espressif Arduino core 3.3.12 and USB CDC On Boot enabled.
3. Upload the same receiver sketch to each wrist board. Connect D10 to the motor module input and share ground. Open Serial Monitor at 115200 baud and send `m` to print each wrist's Wi-Fi station MAC address.
4. Fill in the left and right `WRIST_MACS` entries in the sender sketch before uploading it. The checked-in entries are all zeroes, so the sender skips both wrists until their addresses are set. Both sketches use Wi-Fi channel 1.
5. Connect the MSP430 and ESP32 sender as shown in the wiring table. Grip the FSR and verify that the sender logs left and right distances and that the corresponding wrist pulses faster as an obstacle gets closer.
