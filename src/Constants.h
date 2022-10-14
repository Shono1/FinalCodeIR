#pragma once

#define IR_REMOTE_PIN 14
#define RANGEFINDER_ECHO_PIN 17
#define RANGEFINDER_TRIG_PIN 12
#define GRIPPER_POT_PIN 18

#define DRIVE_KP 75
// TODO: Tune liftKP
#define LIFT_KP 1
#define BASE_MAX_DRIVE_POWER 80
#define BASE_MAX_LIFT_POWER 400
#define DRIVE_TOLERANCE 1
// TODO: CALCULATE LIFT TOLERANCE
#define LIFT_TOLERANCE 3
#define DRIVE_DEADBAND 50 
#define LIFT_DEADBAND 350
#define TICKS_PER_LIFT_OUTPUT_DEGREE 96

#define CROSS_PASSING_DIST 7
#define TURN_OFF_LINE_ANGLE 60

#define LEFT_LINE_PIN 20
#define RIGHT_LINE_PIN 22
#define BLACK_THRESH 400
#define LINE_FOLLOW_SPEED 60
#define LINE_FOLLOW_KP 0.05

#define IR_BUTTON_START 0x04 // setup
#define IR_BUTTON_PAUSE 0x01 // play/pause
#define IR_BUTTON_RESUME 0x05 // prev/up
#define IR_BUTTON_STOP 0x06 // stop/mode
#define IR_BUTTON_RESET 0x0E // back

#define TRANSFER_BLOCK_ANGLE 0

#define ROOF_25_ANGLE 135
#define ROOF_25_LIFTED_ANGLE 95
#define ROOF_25_FINAL_ANGLE 122

#define ROOF_45_ANGLE 56
#define ROOF_45_FINAL_ANGLE 50
#define ROOF_45_LIFTED_ANGLE 70

#define GRIPPER_TIMEOUT 2000
#define GRIPPER_OPEN_MS 2800
#define GRIPPER_OPEN_POT 440
#define GRIPPER_CLOSED_MS 1450
#define GRIPPER_CLOSED_POT 360

#define RANGE_EARLY_APPROACH_TOWER 24
#define RANGE_LATE_APPROACH_TOWER 21

#define FINAL_APPROACH_WAIT_TIME 5000
#define WAIT_FOR_GRIPPER_LIFT_TIME 3000
#define FOLLOW_LINE_TILL_POINTED_AT_BOX_TIME 2000
#define BACK_UP_AFTER_PICKUP -4

#define TRANSFER_DROPOFF_OFFSET 10.795
#define TRANSFER_DROPOFF_LIFT_ANGLE -4
#define TRANSFER_BACKOFF -6
#define LIFT_ANGLE_AT_PICKUP 2

#define TOP_LINE_TO_CROSS_LINE 25
#define LEG_ONE_ANGLE -60
#define LEG_ONE_LENGTH 36
#define TACK_ANGLE -30

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
  Challenge_05a_ReverseUntilLine,
  Challenge_05b_DrivePastCross,
  Challange_05c_WaitForDrivePastCross,
  Challenge_05d_TurnOffLine,
  Challenge_05e_WaitForTurnOffLine,
  Challenge_05f_SearchForLine,
  Challenge_060_FollowLineUntilBox,
  Challenge_061_LowerArmForBox,
  Challenge_06a_WaitBeforeReleasing,
  Challenge_062_WaitForOpen,
  Challenge_063_BackOffPlatform,
  Challenge_064_WaitForBackOff,
  Challenge_070_WaitForNewPlate,
  Challenge_071_GoToNewPlate,
  Challenge_072_WaitForGoToNewPlate,
  Challenge_073_GrabNewPlate,
  Challenge_074_WaitForGrab,
  Challenge_075_BackUp,
  Chllanege_076_WaitForBackUp,
  Challenge_080_TurnOffLine,
  Challange_081_WaitForTurnOffLine,
  Challenge_082_SearchForLine,
  Challenge_083_DriveUntilCross,
  Challenge_084_DrivePastCross,
  Challenge_085_WaitForDrivePastCross,
  Challenge_086_TurnOffLine,
  Challenge_087_WaitForTurnOffLine,
  Challenge_088_SearchForLine,
  Challenge_090_BeginApproach,
  Challenge_091_FinalApproach,
  Challenge_092_FinalLift,
  Challenge_093_WaitForFinalLift,
  Challenge_09a_LiftDelay,
  Challenge_094_Release,
  Challenge_095_WaitForRelease,
  Challenge_100_ReverseUntilLine,
  Challenge_101_DrivePastCross,
  Challenge_102_WaitForDrivePastCross,
  Challenge_103_TurnOffLine,
  Challenge_104_WaitForTurnOffLine,
  Challenge_105_SearchForLine,
  Challenge_110_GoToBlock,
  Challenge_111_WaitForGoToBlock,
  Challenge_112_TurnTowardsOtherSide,
  Challenge_113_WaitForTurn,
  Challenge_114_StartLegOne,
  Challenge_115_WaitForLegOne,
  Challenge_116_Tack,
  Challenge_117_WaitForTack,
  Challenge_118_StartLegTwo,
  Challenge_120_DrivePastLine,
  Challenge_121_WaitForDrivePastLine,
  Challenge_122_SearchForLine,
  Challenge_123_DriveToCross,
  Challenge_124_DriveOverCross,
  Challenge_125_WaitForDriveOverCross,
  Challenge_126_TurnOffLine,
  Challenge_126a_WaitForTurnOffLine,
  Challenge_127_SearchForLine,
  Challenge_130_ApproachRoof,
  Challenge_131_FinalApproach,
  Challenge_132_WaitForGrip,
  Challenge_133_GrabPlate,
  Challenge_134_LiftPlate,
  Challenge_140_ReverseUntilLine,
  Challenge_141_DrivePastCross,
  Challange_142_WaitForDrivePastCross,
  Challenge_143_TurnOffLine,
  Challenge_144_WaitForTurnOffLine,
  Challenge_145_SearchForLine,
  Challenge_150_FollowLineUntilBox,
  Challenge_151_LowerArmForBox,
  Challenge_15a_WaitBeforeReleasing,
  Challenge_152_WaitForOpen,
  Challenge_153_BackOffPlatform,
  Challenge_154_WaitForBackOff,
  Challenge_160_WaitForNewPlate,
  Challenge_161_GoToNewPlate,
  Challenge_162_WaitForGoToNewPlate,
  Challenge_163_GrabNewPlate,
  Challenge_164_WaitForGrab,
  Challenge_165_BackUp,
  Chllanege_166_WaitForBackUp,
  Challenge_170_TurnOffLine,
  Challange_171_WaitForTurnOffLine,
  Challenge_172_SearchForLine,
  Challenge_173_DriveUntilCross,
  Challenge_174_DrivePastCross,
  Challenge_175_WaitForDrivePastCross,
  Challenge_176_TurnOffLine,
  Challenge_177_WaitForTurnOffLine,
  Challenge_178_SearchForLine,
  Challenge_180_BeginApproach,
  Challenge_181_FinalApproach,
  Challenge_182_FinalLift,
  Challenge_183_WaitForFinalLift,
  Challenge_18a_DelayRelease,
  Challenge_184_Release,
  Challenge_185_WaitForRelease,
  Challenge_190_ReverseUntilLine
} challengeState;

enum MotorState
{
  MotorState_Idle,
  MotorState_Holding,
  MotorState_ToTarget,
  MotorState_LineFollow,
  MotorState_LineFollowReverse,
  MotorState_LineFollowForDistance,
  MotorState_FreeDrive
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
