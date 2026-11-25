/* SenseStep motor-only bench test, XIAO ESP32-S3.
 * Active-high IN/VCC/GND motor module with its own MOSFET driver.
 * D0/GPIO1 -> IN ONLY with verified 3.3 V compatibility or a logic buffer.
 * Motor VCC -> USB 5 V; GND -> common ground. Never power motor from GPIO.
 * Plays five 250 ms pulses after startup, then stays off.
 * Serial Monitor 115200: t = repeat, x = stop.
 */
#include <Arduino.h>

constexpr uint8_t MOTOR_PIN = D0;
constexpr uint32_t ON_MS = 250;
constexpr uint32_t OFF_MS = 1750;
uint8_t pulsesRemaining = 5;
uint32_t phaseStartedAt = 0;
bool motorIsOn = false;

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  Serial.begin(115200);
  const uint32_t startedAt = millis();
  while (!Serial && millis() - startedAt < 3000) delay(10);
  Serial.println("Motor test: five pulses. t = repeat, x = stop. GPIO D0 / 1.");
  phaseStartedAt = millis();
}

void loop() {
  const uint32_t now = millis();
  while (Serial.available()) {
    const char command = Serial.read();
    if (command == 't' || command == 'x') {
      digitalWrite(MOTOR_PIN, LOW);
      motorIsOn = false;
      pulsesRemaining = command == 't' ? 5 : 0;
      phaseStartedAt = now;
      Serial.println(command == 't' ? "Starting five pulses." : "Motor stopped.");
    }
  }
  if (pulsesRemaining && !motorIsOn && now - phaseStartedAt >= OFF_MS) {
    motorIsOn = true;
    phaseStartedAt = now;
    digitalWrite(MOTOR_PIN, HIGH);
    Serial.printf("Motor ON; pulse %u of 5\n", 6 - pulsesRemaining);
  } else if (motorIsOn && now - phaseStartedAt >= ON_MS) {
    motorIsOn = false;
    phaseStartedAt = now;
    digitalWrite(MOTOR_PIN, LOW);
    --pulsesRemaining;
    Serial.println(pulsesRemaining ? "Motor OFF" : "Finished. Motor OFF; t to repeat.");
  }
  delay(1);
}
