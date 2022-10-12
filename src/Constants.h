

#define IR_REMOTE_PIN 14
#define RANGEFINDER_ECHO_PIN 17
#define RANGEFINDER_TRIG_PIN 12
#define DRIVE_KP 0.7
#define LIFT_KP 0.7
#define BASE_MAX_DRIVE_POWER 200
#define BASE_MAX_LIFT_POWER 400
#define DRIVE_TOLERANCE 0.5
// TODO: Calculate BOTH deadbands
#define DRIVE_DEADBAND 50 
#define LIFT_DEADBAND 350

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
  Challenge_020_FollowLine,
  Challenge_030_StartWait,
  Challenge_040_StartSearch,
  Challenge_050_TurnatCrossing,
  Challenge_060_ApproachPanel,
  Challenge_070_ApproachPlacement,
  Challenge_080_TurnAround,
  Challenge_090_MovetoAngleOne,
  Challenge_100_MovetoAngleTwo,
  Challenge_110_OpentheGripper,
  Challenge_120_ClosetheGripper,
  Challenge_130_Stop,
  Challenge_140_StopIdle
} challengeState,
    nextChallengeState;

enum MotorState
{
  MotorState_Idle,
  MotorState_Holding,
  MotorState_Active
} driveMotorState,
    blueMotorState;

enum Motor
{
  Motor_RightDrive,
  Motor_LeftDrive,
  Motor_Blue
};

enum IRButton
{
    IRButton_Error,
    IRButton_Start,
    IRButton_Pause,
    IRButton_Stop,
    IRButton_Resume
};