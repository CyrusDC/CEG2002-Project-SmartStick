# SmartStick development and bench tests

Use the pin assignments in each source file when reproducing a bench test. See the [repository README](../README.md) for the three submission programs and their wiring.

| Folder | Contents |
| --- | --- |
| [MSP430 component tests](msp430) | FSR, servo, ultrasonic and water sensor checks |
| [Earlier cane prototype](msp430/cane_prototype/main.c) | Grip detection, servo covers, ultrasonic ranging and UART |
| [MSP430 development tests](msp430) | Servo buttons, ultrasonic ranging, wrist distance frames, and UART vibration triggers |
| [ESP32 tests](esp32) | ESP-NOW link and haptic demos, motor-only checks, vibration command pair, and alternate wrist pulse patterns |

## Build the MSP430 programs

Create an MSP430F5529 project in Code Composer Studio and add the source file for one test at a time.

The earlier cane uses P2.3 and P2.4 for ultrasonic TRIG, P1.2 and P1.3 for ECHO through voltage dividers, and P3.5 and P3.6 for servo PWM. Its three FSR inputs are P6.0–P6.2. This snapshot defaults to testing grip with the LaunchPad's S1 button. The servo constants in the source are specific to the bench setup.

## Build the ESP32 sketches

Open the required `.ino` in Arduino IDE with the `XIAO_ESP32S3` board selected, Espressif Arduino core 3.3.12, and USB CDC On Boot enabled. The USB monitor runs at 115200 baud for sketches with serial logs.

- Pair `board_a_sender` with `board_b_receiver` for the broadcast radio test, or `board_a_haptic_demo` with `board_b_haptic_receiver` for the simulated distance demo.
- Pair `vibration_sender` with `vibration_receiver` to test a `v` command from USB serial or the MSP430 `vibration_uart` test. The configurable `espnow_link_test` uses its own packet format and ships with setup instructions.
- `Vibrate` and `motor_pulse_test` exercise the motor without a sender. Check the GPIO in the selected sketch before connecting the motor driver's input.
