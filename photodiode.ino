const int positivePin = A0;
const int negativePin = A1;
const float referenceVoltage = 3.710; // Arduino Mega reference voltage
const float ALPHA = 0.001; // Smoothing factor (0 to 1, smaller = more smoothing)
float smoothedVoltage = 0; // Initialize smoothed voltage

void setup() {
  Serial.begin(115200);
}

void loop() {
  int positive = analogRead(positivePin);
  int negative = analogRead(negativePin);
  int difference = positive - negative;
  float voltage = difference * (referenceVoltage / 677.0);

  // Apply EMA filter
  smoothedVoltage = (ALPHA * voltage) + ((1.0 - ALPHA) * smoothedVoltage);

  Serial.println(smoothedVoltage);

  // delay(500); // Uncomment if you want a delay
}
