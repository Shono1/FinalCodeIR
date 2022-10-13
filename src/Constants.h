

#define IR_REMOTE_PIN 14
#define RANGEFINDER_ECHO_PIN 17
#define RANGEFINDER_TRIG_PIN 12

#define DRIVE_KP 50
#define LIFT_KP 0.7
#define BASE_MAX_DRIVE_POWER 100
#define BASE_MAX_LIFT_POWER 400
#define DRIVE_TOLERANCE 0.5
// TODO: CALCULATE LIFT TOLERANCE
#define LIFT_TOLERANCE 100
// TODO: Calculate BOTH deadbands
#define DRIVE_DEADBAND 50 
#define LIFT_DEADBAND 350
#define TICKS_PER_REVOLUTION 540

#define LEFT_LINE_PIN 20
#define RIGHT_LINE_PIN 22
#define BLACK_THRESH 400
#define LINE_FOLLOW_SPEED 40
#define LINE_FOLLOW_KP 0.05

#define IR_BUTTON_START 0x04 // setup
#define IR_BUTTON_PAUSE 0x01 // play/pause
#define IR_BUTTON_RESUME 0x05 // prev/up
#define IR_BUTTON_STOP 0x06 // stop/mode
#define IR_BUTTON_RESET 0x0E // back



enum OperatingState
{
  Operating_Idle,
  Operating_Running,
  Operating_Paused,
  Operating_Done,
  Operating_Stopped
} operatingState;

enum ChallengeState
{
  Challenge_010_StartPreTurn,
  Challenge_011_WaitForTurn,
  Challenge_012_SearchForLine,
  Challenge_020_FollowLine,
  Challenge_030_ForwardAtCross,
  Challenge_031_WaitForForwardAtCross,
  Challenge_032_StartTurnAtCross,
  Challenge_033_WaitForTurnAtCross,
  Challenge_034_SearchForLineAtCross,
  Challenge_040_ApproachPanel,
  Challenge_050_BeginLift
} challengeState;

enum MotorState
{
  MotorState_Idle,
  MotorState_Holding,
  MotorState_ToTarget,
  MotorState_LineFollow
} driveMotorState,
    blueMotorState;

enum Motor
{
  Motor_RightDrive,
  Motor_LeftDrive,
  Motor_Blue
};
