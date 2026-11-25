# CEG2002-Project-SmartStick
CEG2002-CEG2001 Project
Pin Assignement
MSP430F5529 Pin Component Function / Wiring
P1.2 (TA0.1) Left Servo (MG90S) PWM Signal
P1.3 (TA0.2) Right Servo (MG90S) PWM Signal
P1.4 Capacitive Touch Sense pin (RECV_PIN) — copper pad + one end of 10 MΩ resistor
P1.6 Capacitive Touch Drive pin (SEND_PIN) — far end of 10 MΩ resistor
P2.0 Active Buzzer Buzzer + (Buzzer - to GND)
P3.5 Both HC-SR04s Shared TRIG
P2.5 Left HC-SR04 Left ECHO (via 10 kΩ / 15 kΩ voltage divider)
P1.5 Right HC-SR04 Right ECHO (via 10 kΩ / 15 kΩ voltage divider)
P3.3 (UCA0TXD) XIAO ESP (SenseStep) UART TX \rightarrow XIAO **D7 / RX** (9600 baud, 8N1)
P7.0 (A0) Water Sensor Signal S (via 10 kΩ / 15 kΩ divider, or 10 kΩ series resistor)
P6.1 (A1) FSR Grip Sensor 3.3V \rightarrow FSR \rightarrow P6.1 \rightarrow 10 kΩ \rightarrow GND
 |
Power & Ground:
- 3.3V: FSR leg 1.
- 5V: Both HC-SR04 VCC, Water Sensor +, and Servos VCC.
- GND: Common ground shared across the MSP430, XIAO ESP, servos, buzzer -, voltage dividers, and FSR pull-down.
