#include <Servo.h>
#include <SoftwareSerial.h>
#include <math.h>

// Servo and serial setup
Servo servoA, servoB, servoC;
const int pinA = 2, pinB = 3, pinC = 4;
SoftwareSerial mySerial(6, 7);  // RX, TX

// Constants
const float mountA = 300.0, mountB = 60.0, mountC = 180.0; // angle positions
String lastCommand = "";
bool waitingForNoCube = false; // added to keep repeating commands 
bool startupSweepDone = false; // visual indicator 

void setup() {
  Serial.begin(9600); // debug serial 
  mySerial.begin(9600); // uno serial 

  servoA.attach(pinA);
  servoB.attach(pinB);
  servoC.attach(pinC);

  setAllServos(180);
  Serial.println("UNO Ready");
}

void loop() {
  if (mySerial.available()) {
    String input = mySerial.readStringUntil('\n'); // looking for command from opencv 
    input.trim(); 
    Serial.println("Received: " + input); // for debuging corrupted commands 

    if (input == "no cube") {      // gives a small dance after the first "no cube" command. helps to know when boot is finished in headless mode 
      if (!startupSweepDone) {
        startupSweep();
        startupSweepDone = true;
      }

      waitingForNoCube = false; // helps prevent repeating commands 
      lastCommand = "";
      Serial.println("Reset: waiting for new cube"); 
      return;
    }

    if (input.indexOf(',') == -1 || input.indexOf(',') != input.lastIndexOf(',') || // prevent corrupted commands from messing up run
        waitingForNoCube || input == lastCommand) {
      Serial.println("Ignored: " + input);
      return;
    }

    lastCommand = input;
    waitingForNoCube = true;

    int commaIndex = input.indexOf(','); 
    String color = input.substring(0, commaIndex);
    float size = input.substring(commaIndex + 1).toFloat();

    float targetAngle = 0; // reject everything except below
    if (size == 2.0) {
      if (color == "GREEN") targetAngle = 90;
      else if (color == "ORANGE") targetAngle = 180;
      else if (color == "YELLOW") targetAngle = 270;
    }
    // debug commands 
    Serial.print("Detected: ");
    Serial.print(color); Serial.print(", "); Serial.print(size, 2);
    Serial.print(" in... tilting to "); Serial.print(targetAngle); Serial.println("°");

    float angleA = -45.0 * cos((targetAngle - mountA) * PI / 180.0) + 135.0; // tilt angle code for servo a 
    float angleB = -45.0 * cos((targetAngle - mountB) * PI / 180.0) + 135.0; // tilt angle code for servo b
    float angleC = -45.0 * cos((targetAngle - mountC) * PI / 180.0) + 135.0; // tilt angle code for servo c 

    setServos(angleA, angleB, angleC); // actually tilt 
    delay(800); // wait for cube to fall 

    setAllServos(180); // call to function; go flat 
    Serial.println("Returned to flat"); // debug
  }
}
// return flat 
void setAllServos(int angle) {
  servoA.write(angle);
  servoB.write(angle);
  servoC.write(angle);
}

void setServos(float a, float b, float c) {
  servoA.write(a);
  servoB.write(b);
  servoC.write(c);
}
// does the startup dance 
void startupSweep() {
  Serial.println("startup sweep");
  setAllServos(180);
  delay(400);
  for (int pos = 180; pos >= 150; pos -= 2) {
    setAllServos(pos);
    delay(15);
  }
  for (int pos = 150; pos <= 180; pos += 2) {
    setAllServos(pos);
    delay(15);
  }
}
