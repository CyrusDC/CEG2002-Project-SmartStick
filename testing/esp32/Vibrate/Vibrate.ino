constexpr int motorPin = D10;

void setup() {
  pinMode(motorPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(motorPin, HIGH);
  digitalWrite(LED_BUILTIN, LOW);  // Orange LED on
  delay(500);

  digitalWrite(motorPin, LOW);
  digitalWrite(LED_BUILTIN, HIGH); // Orange LED off
  delay(1500);
}
