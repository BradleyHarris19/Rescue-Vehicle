#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>

// #define USE_CONSOLE

const int STEER_SCALE = 2;
const int DEADZONE = 30;   //joystick deadzone

const int BUZZER = 2;

const int LMOTOR_ENA = 6;
const int LMOTOR_IN1 = 14;
const int LMOTOR_IN2 = 15;

const int RMOTOR_ENB = 5;
const int RMOTOR_IN3 = 16;
const int RMOTOR_IN4 = 17;

const int YAW_DIR = 4;
const int YAW_SPEED = 3;
const int SHOULDER_POS = 9;
const int ELBOW_DIR = 7;
const int ELBOW_SPEED = 8;
const int GRIPPER_DIR = 19;
const int GRIPPER_SPEED = 18;

Servo ShoulderPos;

const unsigned long COMMAND_TIMEOUT_MS = 2000;
const unsigned long REPORT_INTERVAL_MS = 1000;

#ifdef USE_CONSOLE
  char buffer[256];
#endif

#include "RobotCommand.h"

RobotCommand commandUpdate;
bool volatile commandUpdateReceived = false;
unsigned long lastUpdateTime = 0;
unsigned long lastReportTime = 0;
unsigned spiReceiveCount = 0;
byte* const spiReceiveBuffer = (byte* const)&commandUpdate;

void allOff();

void receiveSpiData() {

  // byte b = SPI.transfer(0);  // Doesn't work!
  byte b = SPDR; SPDR = 0;

/*
  static char buf[10];
  static byte buffer[64];
  static int bufcount=0;
  buffer[bufcount++] = b;
  if (bufcount == 60) {
    for (int i=0; i<60; i++) {
      sprintf(buf, "%02X ", buffer[i]);
      Serial.print(buf);
    }
    bufcount = 0;
  }
*/

  // Wait for the current command to be processed
  if (commandUpdateReceived)
    return;

  // Looking for the start of the command
  if (spiReceiveCount == 0) {
    if (b == 0xAA) {
      // Start sync received
      commandUpdateReceived = false;
      spiReceiveBuffer[spiReceiveCount++] = b;
    }
    else {
      // Invalid sync received
      spiReceiveCount = 0;  // Start again
    }
  }
  // Looking for the end of the command
  else if (spiReceiveCount == (sizeof(RobotCommand) - 1)) {
    if (b == 0x55) {
      // End sync received
      spiReceiveBuffer[spiReceiveCount] = b;
      spiReceiveCount = 0;
      commandUpdateReceived = true;
    }
    else {
      // Invalid sync received
      spiReceiveCount = 0;  // Start again
    }
  }
  // In the middle of receiving the command
  else {
    // Store received date
    spiReceiveBuffer[spiReceiveCount++] = b;
  }

}

void setup() {

  // Set up miscellaneous outputs
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  // Set up motor driver outputs
  pinMode(RMOTOR_ENB, OUTPUT);
  pinMode(RMOTOR_IN4, OUTPUT);
  pinMode(RMOTOR_IN3, OUTPUT);
  pinMode(LMOTOR_IN2, OUTPUT);
  pinMode(LMOTOR_IN1, OUTPUT);
  pinMode(LMOTOR_ENA, OUTPUT);

  pinMode(YAW_DIR, OUTPUT);
  pinMode(YAW_SPEED, OUTPUT);
  pinMode(SHOULDER_POS, OUTPUT);
  pinMode(ELBOW_DIR, OUTPUT);
  pinMode(ELBOW_SPEED, OUTPUT);
  pinMode(GRIPPER_DIR, OUTPUT);
  pinMode(GRIPPER_SPEED, OUTPUT);

  ShoulderPos.attach(SHOULDER_POS);

  allOff();
  lastUpdateTime = lastReportTime = millis();

  // Set up connection to controller
  spiReceiveCount = 0;
  commandUpdateReceived = false;
  SPI.begin(SPI_SLAVE);
  SPI.attachInterrupt(receiveSpiData);

#ifdef USE_CONSOLE
  Serial.begin(115200);
#endif
}

/* Update the platform motion
*/
void updatePlatform(int steer, int drive) {

  int Vl = drive + (steer/STEER_SCALE);
  int Vr = drive - (steer/STEER_SCALE);
  Vl = constrain(Vl, -100, 100);
  Vr = constrain(Vr, -100, 100);
  Vl = map(Vl, -100, 100, -255, 255);
  Vr = map(Vr, -100, 100, -255, 255);

/*
  sprintf(buffer, "l=%d, r=%d", Vl, Vr);
  Serial.println(buffer);
*/

   if (abs(Vl) <= DEADZONE)               //if inside deadzone do nothing
   {
     digitalWrite(LMOTOR_IN2, HIGH);
     digitalWrite(LMOTOR_IN1, HIGH);
     analogWrite(LMOTOR_ENA, 0);
   }
   else if (Vl < 0)                      //if less than 0 go forward
   {
     digitalWrite(LMOTOR_IN2, LOW);
     digitalWrite(LMOTOR_IN1, HIGH);
     analogWrite(LMOTOR_ENA, abs(Vl));
   }
   else {                                //else go backward
     digitalWrite(LMOTOR_IN2, HIGH);
     digitalWrite(LMOTOR_IN1, LOW);
     analogWrite(LMOTOR_ENA, abs(Vl));
   }

   if (abs(Vr) <= DEADZONE)               //if inside deadzone do nothing
   {
     digitalWrite(RMOTOR_IN3, HIGH);
     digitalWrite(RMOTOR_IN4, HIGH);
     analogWrite(RMOTOR_ENB, 255);
   }
   else if (Vr < 0)                      //if less than 0 go forward
   {
     digitalWrite(RMOTOR_IN3, LOW);
     digitalWrite(RMOTOR_IN4, HIGH);
     analogWrite(RMOTOR_ENB, abs(Vr));
   }
   else {                                //else go backward
     digitalWrite(RMOTOR_IN3, HIGH);
     digitalWrite(RMOTOR_IN4, LOW);
     analogWrite(RMOTOR_ENB, abs(Vr));
   }

}

/* Update motor driver
*/
void updateHG7881(int val, int const directionPin, int const speedPin)
{
  val = map(val, -100, 100, -255, 255);     //maps the values
  if (abs(val) <= DEADZONE)               //if inside deadzone do nothing
  {
    digitalWrite(directionPin, HIGH);
    analogWrite(speedPin, 255);
  }
  else if (val < 0)                      //if less than 0 go forward
  {
    digitalWrite(directionPin, LOW);
    analogWrite(speedPin, abs(val));
  }
  else {                                //else go backward
    digitalWrite(directionPin, HIGH);
    analogWrite(speedPin, 255-abs(val));
  }
}

/* Update the arm position
*/
void updateArm(int yaw, int shoulder, int elbow, int gripper) {

  updateHG7881(yaw, YAW_DIR, YAW_SPEED);
  updateHG7881(elbow, ELBOW_DIR, ELBOW_SPEED);
  updateHG7881(gripper, GRIPPER_DIR, GRIPPER_SPEED);

  int deg = map(shoulder, -100, 100, 0, 90);
  ShoulderPos.write(deg);
}

/* Update the buzzer
*/
void updateBuzzer(int buzzer) {
     digitalWrite(BUZZER, (buzzer > 0) ? HIGH : LOW);
}

/* Turn off all motors, make robot safe
*/
void allOff() {
  updatePlatform(0, 0);
  updateArm(0, 0, 0, 0);
  updateBuzzer(0);
}


void loop()
{
  // Look for a received command
  if (commandUpdateReceived) {
    lastUpdateTime = lastReportTime = millis();
#ifdef USE_CONSOLE
    Serial.print("COMMAND: ");
    printRobotCommand(commandUpdate);
#endif
    updatePlatform(commandUpdate.steer, commandUpdate.drive);
    updateArm(commandUpdate.yaw, commandUpdate.shoulder, commandUpdate.elbow, commandUpdate.gripper);
    updateBuzzer(commandUpdate.buzzer);
    commandUpdateReceived = false;
  }

  // If we haven't seen a command in a while then revert to a safe state
  if ((millis() - lastUpdateTime) > COMMAND_TIMEOUT_MS) {
    lastUpdateTime = lastReportTime = millis();
#ifdef USE_CONSOLE
    Serial.println("TIMEOUT");
#endif
    allOff();
  }

  // If we haven't seen a command in a while then revert to a safe state
  if ((millis() - lastReportTime) > REPORT_INTERVAL_MS) {
    lastReportTime = millis();
#ifdef USE_CONSOLE
    Serial.println(".");
#endif
  }
}
