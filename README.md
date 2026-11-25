# SmartStick / SenseStep

**CEG2002 Embedded System Design · CEG2001 · Singapore Institute of Technology**

SmartStick is a foldable walking-stick prototype with left and right ultrasonic ranging, a water-sensor buzzer, and motorized sensor covers. The MSP430F5529 reads the sensors, drives the covers and buzzer, and sends distance data over UART to the cane-side XIAO ESP32-S3. The SenseStep development firmware includes the ESP-NOW bridge and wristband receivers for left and right vibration feedback.

## Where to start

| What you need | Where to find it |
| --- | --- |
| Final MSP430 cane firmware | [`main.c`](CEG2002_Project_Codes/MSP_Codes/main.c) |
| Final-build component tests | [FSR](CEG2002_Project_Codes/MSP_Codes/FSR_Test.c), [Servo](CEG2002_Project_Codes/MSP_Codes/Servo_Test.c), [Ultrasonic](CEG2002_Project_Codes/MSP_Codes/Ultrasonic_Test.c), [Water sensor](CEG2002_Project_Codes/MSP_Codes/Water_Sensor_Test.c) |
| ESP32 sender and wrist firmware snapshots | [ESP32 firmware](CEG2002_Project_Codes/SenseStep/firmware/esp32) |
| Earlier development and bench tests | [SenseStep archive guide](CEG2002_Project_Codes/SenseStep/README.md) · [MSP430 tests](CEG2002_Project_Codes/SenseStep/testing/msp430) · [ESP32 tests](CEG2002_Project_Codes/SenseStep/testing/esp32) |
| Wristband enclosure generators | [Mechanical scripts](CEG2002_Project_Codes/SenseStep/mechanical) |

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
| P3.3 (UCA0TXD) | Cane-side XIAO ESP32-S3 | Connect to D7 / RX. UART uses 9600 baud, 8N1. |
| P7.0 (A12) | Water sensor | Signal S through a 10 kΩ / 15 kΩ voltage divider |
| P6.0 (A0) | FSR grip sensor | Junction between the FSR and the 10 kΩ pull-down |
| P1.0 | Onboard red LED | Left obstacle indicator |
| P4.7 | Onboard green LED | Right obstacle indicator |

**Power and ground**

- **3.3 V** supplies the FSR. Wire the FSR between 3.3 V and the ADC input, and the 10 kΩ pull-down between the ADC input and ground.
- **5 V** supplies the two HC-SR04 modules, water-sensor module, and servo power connections.
- **Common ground** connects the MSP430, XIAO, sensors, servos, buzzer, and voltage dividers.

## Build and bench check

1. Create an MSP430F5529 project in Code Composer Studio. Add [`main.c`](CEG2002_Project_Codes/MSP_Codes/main.c) as the entry point, or build one component test at a time.
2. Check the final-build wiring against the table above. Use the individual test sources to confirm the grip input, cover motion, left and right ultrasonic readings, and water-sensor response on the bench.
3. For the wireless and wristband bench setup, follow the [SenseStep archive guide](CEG2002_Project_Codes/SenseStep/README.md). Earlier prototypes have their own pin assignments and sketch pairings.
