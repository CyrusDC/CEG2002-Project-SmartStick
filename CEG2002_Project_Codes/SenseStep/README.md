# SenseStep development snapshots

These are earlier prototypes and bench tests for the SmartStick project. The [repository README](../../README.md) has the final pin assignments. Use the pin assignments in each source file when reproducing a bench test.

| Folder | Contents |
| --- | --- |
| [Firmware for MSP430F5529](firmware/msp430) | Earlier cane prototype with grip detection, servo covers, ultrasonic ranging and UART |
| [Firmware for XIAO ESP32-S3](firmware/esp32) | Cane UART-to-ESP-NOW bridge and wristband receiver |
| [MSP430 tests](testing/msp430) | Servo buttons, ultrasonic ranging, wrist distance frames, and UART vibration triggers |
| [ESP32 tests](testing/esp32) | ESP-NOW link and haptic demos, motor-only checks, vibration command pair, and alternate wrist pulse patterns |
| [Mechanical scripts](mechanical) | Python generators for the two wristband enclosure sizes |

## Build the MSP430 programs

Create an MSP430F5529 project in Code Composer Studio and use one program's `main.c` as the entry point. Each test is a separate build.

The earlier cane uses P2.3 and P2.4 for ultrasonic TRIG, P1.2 and P1.3 for ECHO through voltage dividers, and P3.5 and P3.6 for servo PWM. Its three FSR inputs are P6.0–P6.2. This snapshot defaults to testing grip with the LaunchPad's S1 button. The servo constants in the source are specific to the bench setup.

## Build the ESP32 sketches

Open the required `.ino` in Arduino IDE with the `XIAO_ESP32S3` board selected, Espressif Arduino core 3.3.12, and USB CDC On Boot enabled. The USB monitor runs at 115200 baud for sketches with serial logs.

- For the cane prototype, connect MSP430 P3.3/TX to the cane XIAO D7/RX and connect ground. The UART runs at 9600 baud, 8N1. Send `m` to each wrist's Serial Monitor to obtain its MAC, then set the left and right `WRIST_MACS` entries in `sensestep_sender.ino`. A zero entry skips that wrist.
- Pair `board_a_sender` with `board_b_receiver` for the broadcast radio test, or `board_a_haptic_demo` with `board_b_haptic_receiver` for the simulated distance demo.
- Pair `vibration_sender` with `vibration_receiver` to test a `v` command from USB serial or the MSP430 `vibration_uart` test. The configurable `espnow_link_test` uses its own packet format and ships with setup instructions.
- `Vibrate` and `motor_pulse_test` exercise the motor without a sender. Check the GPIO in the selected sketch before connecting the motor driver's input.
