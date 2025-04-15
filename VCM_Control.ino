#include <SoftwareSerial.h>
SoftwareSerial controllerSerial(10, 11); // RX, TX pins

char buffer[32]; // Buffer to store incoming message
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);      // USB Serial Monitor
  Serial3.begin(115200);     // Controller

  Serial.println("Arduino Mega ready!");

  Serial3.println(">auto\r");  // AutoCalibrate
  delay(1000);
  Serial3.println(">home\r");  // Home
  delay(1000);
  Serial3.println(">ma 10000\r"); // Move to position (1 mm)
  delay(1000);
  Serial3.println(">ma 30000\r"); // Move to position (3 mm)
}

// Buffer for incoming data
String incomingLine = "";

void loop() {
  // Pass-through from PC to controller
  if (Serial.available()) {
    char pcInput = Serial.read();
    Serial3.write(pcInput);
  }

  // Read full lines from controller
  while (Serial3.available()) {
    char c = Serial3.read();

    if (c == '\r' || c == '\n') { // End of message
      if (incomingLine.length() > 0) {
        processControllerMessage(incomingLine); // Process and print nicely
        incomingLine = ""; // Clear buffer
      }
    } else {
      incomingLine += c;
    }
  }
}

void processControllerMessage(String message) {
  if (message.startsWith("<o")) {
    Serial.println("Controller echo: OK");
  }
  else if (message.startsWith("_Initialize")) {
    Serial.println("Controller status: Initialization");
  }
  else if (message.startsWith("_ok")) {
    // Example: _ok,-4,0.4
    message.replace("_ok,", "Final position: ");
    message.replace(",", ", Speed: ");
    Serial.println(message + " mm/s");
  }
  else if (message.startsWith("_,")) {
    // Example: _,1,10000  --> Current position and target
    int comma1 = message.indexOf(',');
    int comma2 = message.indexOf(',', comma1 + 1);
    String currentPos = message.substring(comma1 + 1, comma2);
    String targetPos = message.substring(comma2 + 1);

    Serial.print("Current position: ");
    Serial.print(currentPos.toFloat() * 0.1); // convert to µm
    Serial.print(" µm, Target position: ");
    Serial.print(targetPos.toFloat() * 0.1); // convert to µm
    Serial.println(" µm");
  }
  else {
    Serial.print("Controller: ");
    Serial.println(message);
  }
}
