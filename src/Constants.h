 

#define IR_REMOTE_PIN 14
#define RANGEFINDER_ECHO_PIN 17
#define RANGEFINDER_TRIG_PIN 12
#define GRIPPER_POT_PIN 18

#define DRIVE_KP 50
// TODO: Tune liftKP
#define LIFT_KP 1
#define BASE_MAX_DRIVE_POWER 120
#define BASE_MAX_LIFT_POWER 400
#define DRIVE_TOLERANCE 0.5
// TODO: CALCULATE LIFT TOLERANCE
#define LIFT_TOLERANCE 3
// TODO: Calculate BOTH deadbands
#define DRIVE_DEADBAND 50 
#define LIFT_DEADBAND 350
#define TICKS_PER_LIFT_OUTPUT_DEGREE 96

#define LEFT_LINE_PIN 20
#define RIGHT_LINE_PIN 22
#define BLACK_THRESH 400
#define LINE_FOLLOW_SPEED 80
#define LINE_FOLLOW_KP 0.05

#define IR_BUTTON_START 0x04 // setup
#define IR_BUTTON_PAUSE 0x01 // play/pause
#define IR_BUTTON_RESUME 0x05 // prev/up
#define IR_BUTTON_STOP 0x06 // stop/mode
#define IR_BUTTON_RESET 0x0E // back

#define TRANSFER_BLOCK_ANGLE 0
#define ROOF_25_ANGLE 135
#define ROOF_25_LIFTED_ANGLE 95
#define ROOF_45_ANGLE 87

#define GRIPPER_TIMEOUT 2000
#define GRIPPER_OPEN_MS 2250
#define GRIPPER_OPEN_POT 440
#define GRIPPER_CLOSED_MS 500
#define GRIPPER_CLOSED_POT 215

#define RANGE_EARLY_APPROACH_TOWER 24
#define RANGE_LATE_APPROACH_TOWER 21

#define FINAL_APPROACH_WAIT_TIME 5000
#define WAIT_FOR_GRIPPER_LIFT_TIME 3000
#define POST_GRIP_BACKWARDS_TIME 1500

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
  // Challenge_000_OpenAndStartMoving,
  // Challenge_001_BeginGrab,
  // Challenge_002_WaitForGrab,
  // Challenge_003_Reverse,
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
  Challenge_041_FinalApproach,
  Challenge_042_WaitForGrip,
  Challenge_043_GrabPlate,
  Challenge_044_LiftBeforeBack,
  Challenge_045_BackTime,
  Challenge_050_TurnOffLine,
  Challenge_051_WaitForStartGrab,
  Challenge_052_WaitForEndGrab,
  Challenge_053_LiftAndBack,
  Challenge_060_TurnAround,
  Challenge_061_DriveToCross,
  Challenge_062_ForwardOverCross,
  Challenge_063_TurnAtCross,
  Challenge_064_BackToTransfer,
  Challenge_065_FinalLoweringStage,
  Challenge_066_ReleaseAtBlock,
  Challenge_070_WaitForNewPlate,
  Challenge_071_GrabNewPlate,
  Challenge_080_TurnAround,
  Challenge_081_DriveToCross,
  Challenge_082_DrivePastCross,
  Challenge_083_TurnTowardRoof,
  Challenge_090_DriveToRoof,
  Challenge_091_FinalApproach,
  Challenge_092_WaitForRelease,
  Challenge_093_WaitForReleaseFinish,
  Challenge_100_BackOffRoof,
  Challenge_101_TurnAround,
  Challenge_102_DriveToCross,
  Challenge_103_DriveOverCross,
  Challenge_104_FindLine,
  Challenge_105_DriveDownOffset,
  Challenge_106_DriveUntilFarLine,
  Challenge_111_DrivePastLine,
  Challenge_112_FindLine,
  Challenge_113_DriveToCross,
  Challenge_114_DrivePastCross,
  Challenge_115_TurnToLine,
  Challenge_110_HeadToSecondRoof,
  Challenge_120_RemoveSecondPlate,
  Challenge_130_DepositSeondPlate,
  Challenge_140_WaitForNewPlate,
  Challenge_150_HeadBackToSecondRoof,
  Challenge_160_ReplaceNewPlate
} challengeState;

enum MotorState
{
  MotorState_Idle,
  MotorState_Holding,
  MotorState_ToTarget,
  MotorState_LineFollow
} driveMotorState,
    blueMotorState;

enum GripperState
{
  GripperState_Open,
  GripperState_CommandClose,
  GripperState_Closing,
  GripperState_Closed
} gripperState;

enum Motor
{
  Motor_RightDrive,
  Motor_LeftDrive,
  Motor_Blue
};
