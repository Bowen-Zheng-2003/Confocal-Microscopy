void setup() {
  analogReference(EXTERNAL); // Set ADC to use AREF (e.g., 2.5V)
  Serial.begin(9600);
}

void loop() {
  int pos = analogRead(A0); // Positive signal
  int neg = analogRead(A1); // Negative signal
  float voltage = (pos - neg) * (5 / 1023.0); // Scale to 2.5V reference
  Serial.println(voltage, 3); // Print with 3 decimal places
  delay(100); // Adjust as needed
}
