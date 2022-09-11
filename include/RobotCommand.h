#pragma pack(push,1)
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
  char buzzer;
  // sync. Must be 0x55
  byte postamble;

  // Initialise
  RobotCommand() { memset(this, 0, sizeof(RobotCommand)); preamble = 0xAA; postamble = 0x55; }
} RobotCommand;
#pragma pack(pop)

void printRobotCommand(RobotCommand const& cmd) {
#ifdef USE_CONSOLE
  sprintf(buffer, "%d: %02X, { D=%4d, S=%4d }, { Y=%d, S=%4d, E=%4d, G=%4d }, Z=%4d, %02X", sizeof(RobotCommand), cmd.preamble, cmd.drive, cmd.steer, cmd.yaw, cmd.shoulder, cmd.elbow, cmd.gripper, cmd.buzzer, cmd.postamble);
  Serial.println(buffer);
#endif
}
