// ** FOR VCM CONTROL ** //
#include <SoftwareSerial.h>
SoftwareSerial controllerSerial(10, 11); // RX, TX pins

char buffer[32]; // Buffer to store incoming message
int bufferIndex = 0;
bool usedVCM = false;
bool directionVCM = false; // Forward = false, backward = true
// Buffer for incoming data
String incomingLine = "";

//** PIN ASSIGNMENT: **//
// FOR POSITION
const int Y_PIN_NR_ENCODER_A      = 18;  // Never change these, since the interrupts are attached to pin 2 and 3
const int Y_PIN_NR_ENCODER_B      = 19;  // Never change these, since the interrupts are attached to pin 2 and 3
const int Y_CS_PIN                = 17;  // Enables encoder A and B to start when set to LOW 

const int X_PIN_NR_ENCODER_A      = 20;  // Never change these, since the interrupts are attached to pin 2 and 3
const int X_PIN_NR_ENCODER_B      = 21;  // Never change these, since the interrupts are attached to pin 2 and 3
const int X_CS_PIN                = 16;  // Enables encoder A and B to start when set to LOW 

// FOR SIGNAL GENERATOR
const int MODE                  = 53; 
const int CURSOR                = 52; 
const int ADD                   = 51; 
const int SUBTRACT              = 50;
const int SIGNAL                = 49;
const int X_STAGE               = 8;
const int Y_STAGE               = 9; 

//** VARIABLE: **//
// Global variable that keeps track of the state:
bool axis_state                 = 0; // 0 is X axis, 1 is Y axis

// FOR POSITION
volatile int xPiezoPosition     = -15000; // [encoder counts] Current piezo position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int oldXPiezoPosition  = 200;
volatile int xEncoderStatus     = 0;      // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)

volatile int yPiezoPosition     = -15000; // [encoder counts] Current piezo position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int oldYPiezoPosition  = 200;
volatile int yEncoderStatus     = 0;      // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).

// FOR CALIBRATION
const int calibrateRange        = 2;
unsigned long startTime         = 0;
const unsigned long duration    = 2000;   // 2 seconds in milliseconds
bool calibrationDone            = false;
const float micronsToCount      = 0.513;
const float countsToMicrons     = 1.949;
const int range                 = 5;      // units are in encoder counts
const int useFinePosition       = 5;
const int useCoarsePosition     = 50;

// FOR SIGNAL GEN
const int DELAY_ON              = 50; 
const int DELAY_OFF             = 100; 
bool TOGGLE_STATE               = false;  // false is if signal generator is off, true is on
int cursor_location             = 6;      // Ranging from 1 to 6 - tells you where the cursor is on LCD
long disp_freq                  = 100000; // Default number that comes up when powering on device
long new_frequency              = 0;
bool direction                  = false;  // false is reverse, true is forward
bool defaultGains               = true;   // true is using default values, false is using new set

// FOR PID CONTROL
unsigned long executionDuration = 0;      // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0;      // [microseconds] System clock value at the moment the loop was started the last time
int  targetPosition             = 0;      // [encoder counts] desired piezo position
float positionError             = 0;      // [encoder counts] Position error
float integralError             = 0;      // [encoder counts * seconds] Integrated position error
float velocityError             = 0;      // [encoder counts / seconds] Velocity error
float desiredFrequency          = 0;      // [Hz] Desired frequency for piezo actuator
float piezoVelocity             = 0;      // [encoder counts / seconds] Current piezo velocity 
int previousPiezoPosition       = 0;      // [encoder counts] Piezo position the last time a velocity was computed 
long previousVelCompTime        = 0;      // [microseconds] System clock value the last time a velocity was computed 
const int  MIN_VEL_COMP_COUNT   = 2;      // [encoder counts] Minimal change in piezo position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME    = 10000;  // [microseconds] Minimal time that must pass between two velocity measurements
int numOfPresses                = 0;
const float KP                  = 2.5;    // [Volt / encoder counts] P-Gain
const float KD                  = 0.005;  // [Volt * seconds / encoder counts] D-Gain
const float KI                  = 0.005;  // [Volt / (encoder counts * seconds)] I-Gain

// Timing:
unsigned long startWaitTime;              // [microseconds] System clock value at the moment the WAIT state started
const long  WAIT_TIME           = 1000000;// [microseconds] Time waiting at each location
const int TARGET_BAND           = 2;      // [encoder counts] "Close enough" range when moving towards a target.

// USER INPUT:
const int NUM_POSITIONS = 4;
struct Position {
  bool axis;                              // 0 = X, 1 = Y
  float positionMicrons;                  // Target position in microns
  int positionCounts;                     // Converted to encoder counts
};

Position targetPositions[NUM_POSITIONS] = {
  {0, 650.0, 0},
  {0, 4782.0, 0},
  {0, 10045.0, 0},
  {0, 20541.0, 0}  
};

// Index to track current position in the sequence
int currentPositionIndex = 0;

bool notFinishedMoving   = true;
bool setAxis             = false;

void setup() {
  // FOR POSITION
  pinMode(X_PIN_NR_ENCODER_A,               INPUT);
  pinMode(X_PIN_NR_ENCODER_B,               INPUT); 
  pinMode(X_CS_PIN,                         OUTPUT);
  digitalWrite(X_CS_PIN,                    LOW);

  pinMode(Y_PIN_NR_ENCODER_A,               INPUT);
  pinMode(Y_PIN_NR_ENCODER_B,               INPUT); 
  pinMode(Y_CS_PIN,                         OUTPUT);
  digitalWrite(Y_CS_PIN,                    LOW);
  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updatePiezoPosition' is called:
  attachInterrupt(digitalPinToInterrupt(X_PIN_NR_ENCODER_A), updateXPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(digitalPinToInterrupt(X_PIN_NR_ENCODER_B), updateXPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  attachInterrupt(digitalPinToInterrupt(Y_PIN_NR_ENCODER_A), updateYPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(digitalPinToInterrupt(Y_PIN_NR_ENCODER_B), updateYPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  // FOR SIGNAL GEN
  pinMode(MODE,                           OUTPUT);
  pinMode(CURSOR,                         OUTPUT);
  pinMode(ADD,                            OUTPUT);
  pinMode(SUBTRACT,                       OUTPUT);
  pinMode(SIGNAL,                         OUTPUT);
  pinMode(X_STAGE,                        OUTPUT);
  pinMode(Y_STAGE,                        OUTPUT);
  Serial.begin(115200);
  Serial3.begin(115200); // FOR VCM
  analogReference(EXTERNAL); // FOR PHOTODIODE
  
  // Convert all positions from microns to encoder counts
  for (int i = 0; i < NUM_POSITIONS; i++) {
    targetPositions[i].positionCounts = targetPositions[i].positionMicrons * micronsToCount;
  }

  // For calibrating VCM:
  Serial3.println(">auto\r");  // AutoCalibrate
  delay(1000);
  Serial3.println(">home\r");  // Home
  delay(1000);
  Serial3.println(">speed 3\r");
  
  // For calibrating X-STAGE:
  digitalWrite(X_STAGE, HIGH);
  setSignal();
  signalOutput(); // turn it on
  setPosition(xPiezoPosition, oldXPiezoPosition);
  digitalWrite(X_STAGE, LOW);

  // For calibrating Y-STAGE:
  calibrationDone = false;
  digitalWrite(Y_STAGE, HIGH);
  signalOutput(); // turn it on
  setPosition(yPiezoPosition, oldYPiezoPosition);
  digitalWrite(Y_STAGE, LOW);

}

void loop() {
  // ** DEFINING AXIS: **//
  axis_state = targetPositions[currentPositionIndex].axis;

  // ** DEFINING MOVE: **//
  while (notFinishedMoving){
    if (!setAxis){
      if (axis_state == 0){
        enableX();
      } else{
        enableY();
      }
      targetPosition = targetPositions[currentPositionIndex].positionCounts;
      setAxis = true;
    }
    
    // Serial.print("Your new target position is: ");
    // Serial.println(targetPosition);
    if ((axis_state == 0 ? xPiezoPosition : yPiezoPosition)>=targetPosition-TARGET_BAND && (axis_state == 0 ? xPiezoPosition : yPiezoPosition)<=targetPosition+TARGET_BAND) { // We reached the position
      // Serial.println("YOU REACHED THE POSITION");
      notFinishedMoving = false;
      Serial.println("Starting");
      if (TOGGLE_STATE){ // make sure the signal generator is off!
        signalOutput();
      }  
    } 

    executionDuration = micros() - lastExecutionTime;
    lastExecutionTime = micros();

    // Compute the position error [encoder counts]
    positionError = targetPosition - (axis_state == 0 ? xPiezoPosition : yPiezoPosition);
    // Serial.print("CURRENT POSITION: ");
    // Serial.println(axis_state == 0 ? xPiezoPosition : yPiezoPosition);
    // Serial.print("Target position: ");
    // Serial.println(targetPosition);
    
    if ((abs(positionError) < useFinePosition)){
      numOfPresses = abs(positionError / 2.0);
      // Serial.print("numOfPresses is ");
      // Serial.println(numOfPresses);
      if (positionError >= 0){
        if (TOGGLE_STATE){
          signalOutput();
        }
        desiredFrequency = 1;
        changeSpeed(int(desiredFrequency));
        reverse();
        for (int i = 0; i < numOfPresses; i++) {
          signalOutput();
          delay(50);
          signalOutput();
          delay(100);
          // Serial.println("Entering positive fine control now");
          // Serial.print("CURRENT POSITION: ");
          // Serial.println(axis_state == 0 ? xPiezoPosition : yPiezoPosition);
          // Serial.print("Target position: ");
          // Serial.println(targetPosition);
        }
      } 
      else{ // you need to go the opposite direction
        if (TOGGLE_STATE){
          signalOutput();
        }
        desiredFrequency = 1;
        changeSpeed(int(desiredFrequency));
        forward();
        for (int i = 0; i < numOfPresses; i++) {
          signalOutput();
          delay(50);
          signalOutput();
          delay(100);
          // Serial.println("Entering negative fine control now");
          // Serial.print("CURRENT POSITION: ");
          // Serial.println(axis_state == 0 ? xPiezoPosition : yPiezoPosition);
          // Serial.print("Target position: ");
          // Serial.println(targetPosition);
        }
      }
    }

    // ENABLES COARSE CONTROL
    else if ((abs(positionError) > useFinePosition)){
      if (positionError >= 0){
        if (desiredFrequency != useCoarsePosition){
          desiredFrequency = useCoarsePosition;
          changeSpeed(int(desiredFrequency));
          reverse();
          signalOutput();
        }
      }
      else { // you need to go the opposite direction
        if (desiredFrequency != useCoarsePosition){
          desiredFrequency = useCoarsePosition;
          changeSpeed(int(desiredFrequency));
          forward();
          signalOutput();
        }
      }
    }
    // Serial.print("CURRENT POSITION: ");
    // Serial.println(axis_state == 0 ? xPiezoPosition : yPiezoPosition);
    // Serial.print("Target position: ");
    // Serial.println(targetPosition);
  }
  
  // ** DEFINING VCM & DATA COLLECTION
  static unsigned long vcmStartTime = 0;
  static bool vcmMoving = false;
  static unsigned long moveStartTime = 0;
  const unsigned long vcmDuration = 2000; // 2 seconds total duration
  const unsigned long moveTimeout = 1000; // 1 second for VCM to complete movement

  if (vcmStartTime == 0) {
    vcmStartTime = millis();
    vcmMoving = false;
    usedVCM = false;
  }

  // Run VCM once and collect voltage data for 2 seconds
  while (millis() - vcmStartTime < vcmDuration) {
    // Read and print voltage
    int pos = analogRead(A0); // Positive signal
    int neg = analogRead(A1); // Negative signal
    float voltage = (pos - neg) * (5 / 1023.0); // Scale to 2.5V reference
    Serial.println(voltage, 3); // Print with 3 decimal places

    // Handle VCM movement only once
    if (!usedVCM && !vcmMoving) {
      if (directionVCM) {
        Serial3.println(">ma 0\r");
        Serial3.println(">status\r");
        directionVCM = false;
      } else {
        Serial3.println(">ma 30000\r");
        Serial3.println(">status\r");
        directionVCM = true;
      }
      vcmMoving = true;
      moveStartTime = millis();
      usedVCM = true;
    }

    // Check if VCM movement is complete
    if (vcmMoving && (millis() - moveStartTime >= moveTimeout)) {
      vcmMoving = false;
      // Serial.println("VCM movement complete");
    }
  }

  // Reset VCM state
  vcmStartTime = 0;
  Serial.println("Finishing");

  currentPositionIndex++;
  if (currentPositionIndex >= NUM_POSITIONS) {
    Serial.println("All positions completed!");
    while(1);
  }
  notFinishedMoving = true;
  setAxis = false;
}

void mode() {
  digitalWrite(MODE, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(MODE, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  // Serial.println("Changing MODE");
}

void cursor() {
  digitalWrite(CURSOR, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(CURSOR, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  // Serial.println("Changing CURSOR");
  cursor_location = cursor_location-1;
  if (cursor_location == 0){ // help loop it back
    cursor_location = 6;
  }
}

void increment() {
  digitalWrite(ADD, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(ADD, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  // Serial.println("INCREASING");
}

void decrement() {
  digitalWrite(SUBTRACT, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(SUBTRACT, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  // Serial.println("DECREASING");
}

void signalOutput() {
  digitalWrite(SIGNAL, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(SIGNAL, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  TOGGLE_STATE = !TOGGLE_STATE;
  if (!TOGGLE_STATE){
    cursor_location = 6;
  }
  // Serial.print("Turning signal ");
  // Serial.println(TOGGLE_STATE ? "ON" : "OFF");
}

void forward() {
  if (!direction){ // if it's in reverse mode, do this
    for (int i = 0; i < 6; i++) {
      mode();
    }
    direction = true;
  } else{
    return;
  }
}

void reverse() {
  if (direction){
    mode();
    direction = false;
  } else{
    return;
  }
}

void setSignal() {
  for (int i = 0; i < 3; i++) {
    mode();
    direction = true; // making it move forward now
  }
  changeSpeed(1000);
  // Serial.println("Finished CALIBRATION");
  // Serial.println("Starting in 1 sec");
  delay(1000);
}

void setPosition(volatile int &piezoPosition, volatile int &oldPiezoPosition) {
  while((piezoPosition != 0) && (!calibrationDone)){
    if ((piezoPosition >= oldPiezoPosition - calibrateRange) && 
        (piezoPosition <= oldPiezoPosition + calibrateRange)) {
      if (startTime == 0) {
        startTime = millis();
      } else if (millis() - startTime >= duration) {
        signalOutput();
        piezoPosition = 0;
        calibrationDone = true;
        // Serial.println("Finished calibrating position");
        // Serial.print("The starting position is: ");
        // Serial.println(piezoPosition);
        break;
      }
    } else {
      startTime = 0;
    }
    oldPiezoPosition = piezoPosition;
    Serial2.print("The calibrating encoder value is ");
    Serial2.println(piezoPosition);
  }
}

void changeSpeed(long frequency) {
  String old_freq = String(disp_freq);
  String new_freq = String(frequency);
  if (old_freq.length() > new_freq.length()){
    int new_cursor_position = cursor_location - old_freq.length();
    for (int i = 0; i < new_cursor_position; i++){
      cursor();
    }
    for (int i = old_freq.length(); i > 0; i--) {
      if (i <= new_freq.length()){ // once the old and new frequency have same digits
        char old_freq_c = old_freq[old_freq.length()-i];
        int old_freq_i = old_freq_c - '0';
        char new_freq_c = new_freq[new_freq.length()-i];
        int new_freq_i = new_freq_c - '0';
        int diff = new_freq_i - old_freq_i;
        if (diff > 0){
          for (int i = 0; i < diff; i++){
            increment();
          }
        }
        else if (diff < 0){
          diff = abs(diff);
          for (int i = 0; i < diff; i++){
            decrement();
          }
        }
      }
      else {
        char old_freq_c = old_freq[old_freq.length()-i];
        int old_freq_i = old_freq_c - '0';
        if (old_freq_i > 0){
          for (int i = 0; i < old_freq_i; i++){
            decrement();
          }
        }
      }
      cursor();
    }
  }
  else if (old_freq.length() == new_freq.length()){
    int new_cursor_position = cursor_location - old_freq.length();
    for (int i = 0; i < new_cursor_position; i++){
      cursor();
    }
    for (int i = old_freq.length(); i > 0; i--) {
      char old_freq_c = old_freq[old_freq.length()-i];
      int old_freq_i = old_freq_c - '0';
      char new_freq_c = new_freq[new_freq.length()-i];
      int new_freq_i = new_freq_c - '0';
      int diff = new_freq_i - old_freq_i;
      if (diff > 0){
        for (int i = 0; i < diff; i++){
          increment();
        }
      }
      else if (diff < 0){
        diff = abs(diff);
        for (int i = 0; i < diff; i++){
          decrement();
        }
      }
      cursor();
    }
  }
  else if (new_freq.length() > old_freq.length()){
    int new_cursor_position = cursor_location - new_freq.length();
    for (int i = 0; i < new_cursor_position; i++){
      cursor();
    }
    for (int i = new_freq.length(); i > 0; i--) {
      if (i <= old_freq.length()){ // once the old and new frequency have same digits
        char old_freq_c = old_freq[old_freq.length()-i];
        int old_freq_i = old_freq_c - '0';
        char new_freq_c = new_freq[new_freq.length()-i];
        int new_freq_i = new_freq_c - '0';
        int diff = new_freq_i - old_freq_i;
        if (diff > 0){
          for (int i = 0; i < diff; i++){
            increment();
          }
        }
        else if (diff < 0){
          diff = abs(diff);
          for (int i = 0; i < diff; i++){
            decrement();
          }
        }
      }
      else {
        char new_freq_c = new_freq[new_freq.length()-i];
        int new_freq_i = new_freq_c - '0';
        if (new_freq_i > 0){
          for (int i = 0; i < new_freq_i; i++){
            increment();
          }
        }
      }
      cursor();
    }
  }
  disp_freq = frequency;
}

bool userDirection() {
  bool value = 0; // 0 = X, 1 = Y
  Serial.println("Direction: X or Y?");  
  while (Serial.available() == 0) {
    // Wait for user input
  }
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if ((input == "Y") || (input == "y")){
      value = 1;
    }
  }
  return value;
}

float userInput() {
  float value = 0; 
  Serial.println("Type in the position (microns) you want to move to!");
  while (Serial.available() == 0) {
    // Wait for user input
  }
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    value = input.toFloat()*micronsToCount;
  }
  return value;
}

void enableX() {
  digitalWrite(Y_STAGE, LOW);
  digitalWrite(X_STAGE, HIGH);
}

void enableY() {
  digitalWrite(X_STAGE, LOW);
  digitalWrite(Y_STAGE, HIGH);
}

//////////////////////////////////////////////////////////////////////
// This is a function to update the encoder count in the Arduino.   //
// It is called via an interrupt whenever the value on encoder      //
// channel A or B changes.                                          //
//////////////////////////////////////////////////////////////////////
void updateXPosition() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  xEncoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  xEncoderStatus |= digitalRead(X_PIN_NR_ENCODER_A);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  xEncoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  xEncoderStatus |= digitalRead(X_PIN_NR_ENCODER_B);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  xEncoderStatus &= 15;
  if (xEncoderStatus==2 || xEncoderStatus==4 || xEncoderStatus==11 || xEncoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    xPiezoPosition++;         // increase the encoder count by one
  } else {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    xPiezoPosition--;         // decrease the encoder count by one
  }
}

void updateYPosition() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  yEncoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  yEncoderStatus |= digitalRead(Y_PIN_NR_ENCODER_A);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  yEncoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  yEncoderStatus |= digitalRead(Y_PIN_NR_ENCODER_B);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  yEncoderStatus &= 15;
  if (yEncoderStatus==2 || yEncoderStatus==4 || yEncoderStatus==11 || yEncoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    yPiezoPosition++;         // increase the encoder count by one
  } else {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    yPiezoPosition--;         // decrease the encoder count by one
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
