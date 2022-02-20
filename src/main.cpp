#include <Arduino.h>

#undef USE_CONSOLE

const int DEADZONE = 20;   //joystick deadzone
const unsigned long COMMAND_TIMEOUT_MS = 2000;

typedef struct RobotCommand {
  // sync. Must be 0xAA
  byte preamble;
  // platform control. All values in range [-100/+100]
  char drive;
  char steer;
  // arm control. All values in range [-100/+100]
  char yaw;
  char shoulder;
  char elbow;
  char gripper;
  char spare;
  // sync. Must be 0x55
  byte postamble;

  // Initialise
  RobotCommand() { memset(this, 0, sizeof(RobotCommand)); preamble = 0xAA; postamble = 0x55; }
} RobotCommand;

RobotCommand commandUpdate;
bool commandUpdateReceived = false;
unsigned spiReceiveCount = 0;
byte* const spiReceiveBuffer = (byte* const)&commandUpdate;

void receiveSpiData() {

  byte b = SPI.transfer(0);

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
  else if (spiReceiveCount < sizeof(RobotCommand)) {
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
  }  // In the middle of receiving the command
  else {
    // Store received date
    spiReceiveBuffer[spiReceiveCount++] = b;
  }

}

void setup() {

  spiReceiveCount = 0;
  commandUpdateReceived = false;

  pinMode(2, OUTPUT);       //set up motor driver outputs
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  pinMode(14, INPUT);      //set up joystick inputs
  pinMode(15, INPUT);
  pinMode(16, INPUT);

  digitalWrite(8, HIGH);   //turns all motors off
  digitalWrite(9, HIGH);

  SPI.begin(SPI_SLAVE);
  SPI.attachInterrupt(receiveSpiData);

 #ifdef USE_CONSOLE
   Serial.begin(115200);
 #endif

}

void ProcessJoystickChannel(const int analogInput, const int directionPin, const int speedPin) //constant joystick convertor from the loop
{
  int val = analogRead(analogInput);
  //Serial.println(val);
  val = map(val, 0, 1023, -255, 255);     //maps the values
  //Serial.println(rightVertical);
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

/* Update the platform motion
*/
void updatePlatform(int steer, int drive) {

}

/* Update the arm position
*/
void updateArm(int yaw, int shoulder, int elbow, int gripper) {

}

/* Turn off all motors, make robot safe
*/
void allOff() {
  updatePlatform(0, 0);
  updateArm(0, 0, 0, 0);
}

void loop()
{
  unsigned long lastUpdateTime = millis();

  if (commandUpdateReceived) {
    lastUpdateTime = millis();
    updatePlatform(commandUpdate.steer, commandUpdate.drive);
    updateArm(commandUpdate.yaw, commandUpdate.shoulder, commandUpdate.elbow, commandUpdate.gripper);
    commandUpdateReceived = false;
  }

  if ((millis() - lastUpdateTime) > COMMAND_TIMEOUT_MS) {
    lastUpdateTime = millis();
    allOff();
  }


    ProcessJoystickChannel(16, 8, 9);
    ProcessJoystickChannel(17, 11, 10);

}
