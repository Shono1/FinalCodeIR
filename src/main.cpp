#include <Arduino.h>
#include <Chassis.h>
#include <Romi32U4.h>
#include <RemoteConstants.h>
#include <IRdecoder.h>
#include <Timer.h>
#include <servo32u4.h>
#include "BlueMotor.h"
#include <Rangefinder.h>
#include "Constants.h"

int leftMotorTarget, rightMotorTarget, blueMotorTarget;
int leftDriveCurrentTicks, rightDriveCurrentTicks, blueMotorCurrentTicks;
int maxDrivePower, maxLiftPower;

bool driveMovementDone = false;
bool liftMovementDone = false;

bool leftBlackFlag = false;
bool rightBlackFlag = false;

long gripperStartTime;
long activeMovementStartTime;

Chassis chassis;
BlueMotor motor;
Romi32U4ButtonB buttonB;
IRDecoder decoder(IR_REMOTE_PIN);
Servo32U4 servo;
Rangefinder rangefinder(RANGEFINDER_ECHO_PIN, RANGEFINDER_TRIG_PIN);

void setTicksAtAngleDrive(double targetAngle);
void setTicksAtDistance(double targetDistance);
void updateDriveMotorPower_TargetMode(Motor whichMotor, int deltaCM, int maxPower);
void updateOpMode();
void updateMotors();
bool pollForSignal(int whichSignal);
void doChallenge();

// bool turnForLine(bool cw) {
//   setTicksAtAngleDrive()
// }
void driveToCross(ChallengeState nextState) {
  driveMotorState = MotorState_LineFollow;
  blueMotorState = MotorState_ToTarget;
  double leftColor = analogRead(LEFT_LINE_PIN);
  double rightColor = analogRead(RIGHT_LINE_PIN);

  if (leftColor > BLACK_THRESH && rightColor > BLACK_THRESH)
  {
    // driveMotorState = MotorState_ToTarget;
    // setTicksAtDistance(0);
    Serial.println("Hit line crossing.");
    challengeState = nextState;
  }
}

void drivePastCross(ChallengeState nextState) {
  driveMotorState = MotorState_ToTarget;
  blueMotorState = MotorState_ToTarget;
  driveMovementDone = false;
  setTicksAtDistance(CROSS_PASSING_DIST);
  challengeState = nextState;
}

bool searchForLine(ChallengeState nextState, bool CCW) {
  driveMotorState = MotorState_ToTarget;
  blueMotorState = MotorState_ToTarget;
  if (CCW) {
    setTicksAtAngleDrive(10);
    if (!leftBlackFlag)
    {
      leftBlackFlag = analogRead(LEFT_LINE_PIN) > BLACK_THRESH;
    }
    if (!rightBlackFlag && leftBlackFlag)
    {
      rightBlackFlag = analogRead(RIGHT_LINE_PIN) > BLACK_THRESH;
    }

    if (leftBlackFlag && rightBlackFlag)
    {
      setTicksAtAngleDrive(0);
      liftMovementDone = false;
      challengeState = nextState;
    }
  }
  else {
    setTicksAtAngleDrive(10);
    if (!rightBlackFlag)
    {
      rightBlackFlag = analogRead(LEFT_LINE_PIN) > BLACK_THRESH;
    }
    if (!leftBlackFlag && rightBlackFlag)
    {
      leftBlackFlag = analogRead(RIGHT_LINE_PIN) > BLACK_THRESH;
    }

    if (leftBlackFlag && rightBlackFlag)
    {
      setTicksAtAngleDrive(0);
      liftMovementDone = false;
      challengeState = nextState;
    }
  }
  
}

bool waitForDriveMovement(ChallengeState nextState) {
  driveMotorState = MotorState_ToTarget; 
  blueMotorState = MotorState_ToTarget; 
  // Serial.println("011 - Wait For Turn");
  if (driveMovementDone)
  {
    challengeState = nextState;
  }
  return driveMovementDone;
}

void turnOffLine(ChallengeState nextState, bool CCW) {
  driveMotorState = MotorState_ToTarget;
  blueMotorState = MotorState_ToTarget;
  driveMovementDone = false;
  setTicksAtAngleDrive(pow(-1, CCW + 1) * TURN_OFF_LINE_ANGLE);
  challengeState = nextState;
}

void openGripper() {
  gripperStartTime = millis();
  servo.writeMicroseconds(GRIPPER_OPEN_MS);
}

void closeGripper() {
  gripperStartTime = millis();
  servo.writeMicroseconds(500);
}

// void checkGripper() {

// }

void setTicksAtAngleDrive(double targetAngle) // Angle direction is determined by sign, positive is anticlockwise
{
  int ticksToGo = targetAngle * (chassis.robotRadius * PI / 180.0) / chassis.cmPerEncoderTick;
  // int motorTargets[2]; // 0 is left, 1 is right
  leftMotorTarget = chassis.leftMotor.getCount() - ticksToGo;
  rightMotorTarget = chassis.rightMotor.getCount() + ticksToGo;
  // return motorTargets;
}

double blueAngleFromTicks(int16_t ticks)
{
  double angle = ((double)ticks) / TICKS_PER_LIFT_OUTPUT_DEGREE;
  return angle;
}

int16_t blueTicksFromAngle(double angle)
{
  int16_t ticks = (int16_t) (angle * TICKS_PER_LIFT_OUTPUT_DEGREE);
  return ticks;
}

void setTicksAtAngleLift(double targetAngle)  // Absolute output angle, NOT A DELTA!!!!
{
  double currentAngle = blueAngleFromTicks(motor.getPosition());
  double angleToGo = targetAngle - currentAngle; 
  int16_t ticksToGo = blueTicksFromAngle(angleToGo);
  blueMotorTarget = motor.getPosition() + ticksToGo;
}

void setTicksAtDistance(double targetDistance) // Distance in cm, direction is determined by sign
{
  int ticksToGo = targetDistance / chassis.cmPerEncoderTick;
  leftMotorTarget = chassis.leftMotor.getCount() + ticksToGo;
  rightMotorTarget = chassis.rightMotor.getCount() + ticksToGo;
}

void updateDriveMotorPower_TargetMode(Motor whichMotor, int deltaCM, int maxPower)
{
  int16_t power = deltaCM * DRIVE_KP;
  int powSign = power < 0 ? -1 : 1;
  int16_t powerClamped = abs(power) > maxPower ? maxPower * powSign : power;
  switch (whichMotor)
  {
  case Motor_LeftDrive:
    chassis.leftMotor.setMotorEffort(powerClamped);
    break;
  case Motor_RightDrive:
    chassis.rightMotor.setMotorEffort(powerClamped);
    break;
  }
}

void updateDriveMotorPower_Directly(Motor whichMotor, int power)
{
  switch (whichMotor)
  {
  case Motor_LeftDrive:
    chassis.leftMotor.setMotorEffort(power);
    break;
  case Motor_RightDrive:
    chassis.rightMotor.setMotorEffort(power);
    break;
  }
}

void updateBlueMotorPower_TargetMode(double deltaDegrees, int maxPower)
{
  int16_t outPow = (int16_t)(deltaDegrees * LIFT_KP);
  motor.setEffort(outPow);
  // Serial.print("Blue Encoder: ");
  // Serial.println(motor.getPosition());
}

void updateOpMode()
{
  switch (operatingState)
  {
  case Operating_Idle:
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_Idle;
    // poll for start signal
    if (pollForSignal(IR_BUTTON_START))
    {
      // let the games begin
      operatingState = Operating_Running;
    }
    break;
  case Operating_Running:
    doChallenge();
    // poll for pause or stop signal
    if (pollForSignal(IR_BUTTON_PAUSE))
    {
      operatingState = Operating_Paused;
    }
    if (pollForSignal(IR_BUTTON_STOP))
    {
      operatingState = Operating_Stopped;
    }
    break;
  case Operating_Paused:
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_Idle; // if the plate's being held, keep holding! if its not, still keep holding!

    // poll for resume signal
    if (pollForSignal(IR_BUTTON_RESUME))
    {
      operatingState = Operating_Running;
    }
    if (pollForSignal(IR_BUTTON_RESET))
    // note: this bit is outside of isa 88 spec, but this a) isn't a batch control process and b) is helpful
    {
      operatingState = Operating_Idle;
      challengeState = Challenge_010_StartPreTurn;
    }
    break;
  case Operating_Done:
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_Idle;

    // poll for reset signal
    if (pollForSignal(IR_BUTTON_RESET))
    {
      operatingState = Operating_Idle;
      challengeState = Challenge_010_StartPreTurn;
    }
    break;
  case Operating_Stopped:
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_Idle;

    // poll for reset signal
    if (pollForSignal(IR_BUTTON_RESET))
    {
      operatingState = Operating_Idle;
      challengeState = Challenge_010_StartPreTurn;
    }
    break;
  }
}

void updateMotors()
{
  switch (driveMotorState)
  {
  case MotorState_Idle:
    chassis.leftMotor.setMotorEffort(0);
    chassis.rightMotor.setMotorEffort(0);
    // zero effort, no control, just stop, hammertime
    break;
  case MotorState_Holding:
    // proportional control w/constant setpoint (actively holding)
    // TODO: make this actually actively hold
    chassis.leftMotor.setMotorEffort(0);
    chassis.rightMotor.setMotorEffort(0);
    break;
  break;
  case MotorState_ToTarget:
    // proportional control w/variable setpoint (moving right round baby right round)
    {
      leftDriveCurrentTicks = chassis.leftMotor.getCount();
      rightDriveCurrentTicks = chassis.rightMotor.getCount();
      // blueMotorCurrentTicks = motor.getPosition();
      double leftDelta = (leftMotorTarget - leftDriveCurrentTicks) * chassis.cmPerEncoderTick;
      double rightDelta = (rightMotorTarget - rightDriveCurrentTicks) * chassis.cmPerEncoderTick;
      // double blueDelta = (blueMotorTarget - blueMotorCurrentTicks);
      updateDriveMotorPower_TargetMode(Motor_LeftDrive, leftDelta, maxDrivePower);
      updateDriveMotorPower_TargetMode(Motor_RightDrive, rightDelta, maxDrivePower);
      // updateBlueMotorPower_TargetMode(blueDelta, maxLiftPower);

      if (abs(leftDelta) < DRIVE_TOLERANCE || abs(rightDelta) < DRIVE_TOLERANCE)
      {
        driveMovementDone = true;
      }
      // if (blueDelta < LIFT_TOLERANCE)
      // {
      //   liftMovementDone = true;
      // }
    }
    // Serial.print("Left Delta: ");
    // Serial.println(leftDelta);
    break;
  case MotorState_LineFollow:
  {
    double leftColor = analogRead(LEFT_LINE_PIN);
    double rightColor = analogRead(RIGHT_LINE_PIN);
    double colorDelta = leftColor - rightColor;

    updateDriveMotorPower_Directly(Motor_LeftDrive, LINE_FOLLOW_SPEED - LINE_FOLLOW_KP * colorDelta);
    updateDriveMotorPower_Directly(Motor_RightDrive, LINE_FOLLOW_SPEED + LINE_FOLLOW_KP * colorDelta);

    // Serial.print("Left Power: ");
    // Serial.println(Motor_LeftDrive, LINE_FOLLOW_SPEED - LINE_FOLLOW_KP * colorDelta);
  }
  break;

  case MotorState_LineFollowReverse:
  {
    double leftColor = analogRead(LEFT_LINE_PIN);
    double rightColor = analogRead(RIGHT_LINE_PIN);
    double colorDelta = leftColor - rightColor;

    updateDriveMotorPower_Directly(Motor_LeftDrive, -LINE_FOLLOW_SPEED + LINE_FOLLOW_KP * colorDelta);
    updateDriveMotorPower_Directly(Motor_RightDrive, -LINE_FOLLOW_SPEED - LINE_FOLLOW_KP * colorDelta);
  }
  break;

  case MotorState_LineFollowForDistance:
  {
    double leftColor = analogRead(LEFT_LINE_PIN);
    double rightColor = analogRead(RIGHT_LINE_PIN);
    double colorDelta = leftColor - rightColor;

    leftDriveCurrentTicks = chassis.leftMotor.getCount();
    rightDriveCurrentTicks = chassis.rightMotor.getCount();
    double leftDelta = (leftMotorTarget - leftDriveCurrentTicks) * chassis.cmPerEncoderTick;
    double rightDelta = (rightMotorTarget - rightDriveCurrentTicks) * chassis.cmPerEncoderTick;

    updateDriveMotorPower_Directly(Motor_LeftDrive, LINE_FOLLOW_SPEED - LINE_FOLLOW_KP * colorDelta);
    updateDriveMotorPower_Directly(Motor_RightDrive, LINE_FOLLOW_SPEED + LINE_FOLLOW_KP * colorDelta);
      // updateBlueMotorPower_TargetMode(blueDelta, maxLiftPower);

    if (abs(leftDelta) < DRIVE_TOLERANCE || abs(rightDelta) < DRIVE_TOLERANCE)
    {
      driveMovementDone = true;
    }
  }
  break;
  }

  switch (blueMotorState)
  {
  case MotorState_Idle:
    // zero effort, no control, just stop, hammertime
    motor.setEffort(0);
    break;

  case MotorState_Holding:
    // proportional control w/constant setpoint (actively holding)
    // TODO: make this actually actively hold
    motor.setEffort(0);
    
    break;
  break;
  case MotorState_ToTarget:
    // proportional control w/variable setpoint (moving right round baby right round)
    { 
      blueMotorCurrentTicks = motor.getPosition();
      double blueDelta = (blueMotorTarget - blueMotorCurrentTicks);
      updateBlueMotorPower_TargetMode(blueDelta, maxLiftPower);
      // Serial.print("Blue Motor Angle Delta: ");
      // Serial.println(blueAngleFromTicks(blueDelta));
      if (abs(blueAngleFromTicks(blueDelta)) < LIFT_TOLERANCE)
      {
        liftMovementDone = true;
      }
    }
    // Serial.print("Angle: ");
    // Serial.println(((double) blueMotorCurrentTicks) / ((double)TICKS_PER_LIFT_OUTPUT_DEGREE));
    break;
  case MotorState_LineFollow:
  {
    // unused mode
  }
  break;
  }
}

void updateGripperState() {
  switch(gripperState) {
    case GripperState_Open:
    {
      servo.writeMicroseconds(GRIPPER_OPEN_MS);
    }
    break;

    case GripperState_CommandClose:
    {
      Serial.println("Command Close");
      gripperStartTime = millis();
      servo.writeMicroseconds(GRIPPER_CLOSED_MS);
      gripperState = GripperState_Closing;
    }
    break;

    case GripperState_Closing:
    {
      servo.writeMicroseconds(GRIPPER_CLOSED_MS);
      if (millis() > gripperStartTime + GRIPPER_TIMEOUT) {
        if (analogRead(GRIPPER_POT_PIN) > GRIPPER_CLOSED_POT) {
          gripperState = GripperState_Open;
        }
        else {
          gripperState = GripperState_Closed;
        }
      }
    }
    break;

    case GripperState_Closed:
    {
      servo.writeMicroseconds(GRIPPER_CLOSED_MS);
    }
  }
}

bool pollForSignal(int whichSignal)
{
  // if the signal from the IR decoder matches what we ask, return true.
  // Otherwise, return false.
  int decoded = decoder.getKeyCode(true); // allow repeat messages
  if (decoded == whichSignal)
  {
    Serial.print("IR Button Matched: ");
    Serial.println(decoded);
  }
  return decoded == whichSignal;
}

void doChallenge()
{
  switch (challengeState)
  {
  case Challenge_010_StartPreTurn:
  {
    driveMotorState = MotorState_ToTarget; 
    blueMotorState = MotorState_ToTarget; 
    setTicksAtAngleDrive(90);
    setTicksAtAngleLift(ROOF_25_ANGLE);
    driveMovementDone = false;
    liftMovementDone = false;
    challengeState = Challenge_011_WaitForTurn;
    // Serial.println("010 - Starting Pre Turn");
  }
  break;

  case Challenge_011_WaitForTurn:
  {
    // Serial.print("Gripper Pot: ");
    // Serial.println(analogRead(18));
    
    driveMotorState = MotorState_ToTarget; 
    blueMotorState = MotorState_ToTarget; 
    // Serial.println("011 - Wait For Turn");
    if (driveMovementDone)
    {
      challengeState = Challenge_012_SearchForLine;
    }
  }
  break;

  case Challenge_012_SearchForLine:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    // Serial.println("012 - Search for Line");
    setTicksAtAngleDrive(10);
    if (!leftBlackFlag)
    {
      leftBlackFlag = analogRead(LEFT_LINE_PIN) > BLACK_THRESH;
    }
    if (!rightBlackFlag && leftBlackFlag)
    {
      rightBlackFlag = analogRead(RIGHT_LINE_PIN) > BLACK_THRESH;
    }

    if (leftBlackFlag && rightBlackFlag)
    {
      setTicksAtAngleDrive(0);
      challengeState = Challenge_020_FollowLine;
    }
  }
  break;

  case Challenge_020_FollowLine:
  {
    // driveMotorState = MotorState_LineFollow;
    // blueMotorState = MotorState_ToTarget;
    // double leftColor = analogRead(LEFT_LINE_PIN);
    // double rightColor = analogRead(RIGHT_LINE_PIN);

    // if (leftColor > BLACK_THRESH && rightColor > BLACK_THRESH)
    // {
    //   // driveMotorState = MotorState_ToTarget;
    //   // setTicksAtDistance(0);
    //   Serial.println("Hit line crossing.");
    //   challengeState = Challenge_030_ForwardAtCross;
    // }
    driveToCross(Challenge_030_ForwardAtCross);
  }
  break;

  case Challenge_030_ForwardAtCross:
  {
    // driveMotorState = MotorState_ToTarget;
    // blueMotorState = MotorState_ToTarget;
    // driveMovementDone = false;
    // setTicksAtDistance(7);
    // challengeState = Challenge_031_WaitForForwardAtCross;

    drivePastCross(Challenge_031_WaitForForwardAtCross);
  }
  break;

  case Challenge_031_WaitForForwardAtCross:
  {
    // driveMotorState = MotorState_ToTarget;
    // blueMotorState = MotorState_ToTarget;
    // if (driveMovementDone)
    // {
    //   setTicksAtDistance(0);
    //   challengeState = Challenge_032_StartTurnAtCross;
    // }
    if (waitForDriveMovement(Challenge_032_StartTurnAtCross)) {
      setTicksAtDistance(0);
    }
  }
  break;

  case Challenge_032_StartTurnAtCross:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    driveMovementDone = false;
    setTicksAtAngleDrive(60);
    challengeState = Challenge_033_WaitForTurnAtCross;
  }
  break;

  case Challenge_033_WaitForTurnAtCross:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    if (driveMovementDone)
    {
      setTicksAtDistance(0);
      challengeState = Challenge_034_SearchForLineAtCross;
      leftBlackFlag = false;
      rightBlackFlag = false;
    }
  }
  break;

  case Challenge_034_SearchForLineAtCross:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    setTicksAtAngleDrive(10);
    if (!leftBlackFlag)
    {
      leftBlackFlag = analogRead(LEFT_LINE_PIN) > BLACK_THRESH;
    }
    if (!rightBlackFlag && leftBlackFlag)
    {
      rightBlackFlag = analogRead(RIGHT_LINE_PIN) > BLACK_THRESH;
    }

    if (leftBlackFlag && rightBlackFlag)
    {
      setTicksAtAngleDrive(0);
      liftMovementDone = false;
      challengeState = Challenge_040_ApproachPanel;
    }
  }
  break;

  case Challenge_040_ApproachPanel:
  {
    driveMotorState = MotorState_LineFollow;
    blueMotorState = MotorState_ToTarget;

    if (rangefinder.getDistance() < RANGE_EARLY_APPROACH_TOWER) {
      driveMotorState = MotorState_Idle;
      // Serial.println("Drive Stopped Early");
    }

    if (liftMovementDone)
    {
      Serial.println("Beginning Final Approach");
      activeMovementStartTime = millis();
      challengeState = Challenge_041_FinalApproach;
    }
  }
  break;

  case Challenge_041_FinalApproach:
  {
    driveMotorState = MotorState_LineFollow;
    blueMotorState = MotorState_ToTarget;

    // if (rangefinder.getDistance() < RANGE_LATE_APPROACH_TOWER) {
    //   driveMotorState = MotorState_Idle;
    //   Serial.println("Drive Stopped Late");
    // }
    // if (liftMovementDone) {
    //   Serial.println("Lift movement complete.");
    //   blueMotorState = MotorState_Idle;
    //   challengeState = Challenge_042_WaitForGrip;
    // }

    if (millis() > activeMovementStartTime + FINAL_APPROACH_WAIT_TIME) {
      Serial.println("Final Approach complete, waiting for grip signal...");
      challengeState = Challenge_042_WaitForGrip;
    }
  }
  break;

  case Challenge_042_WaitForGrip:
  {
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_Idle;
    if (pollForSignal(remote1)) {
      challengeState = Challenge_043_GrabPlate;
      gripperState = GripperState_CommandClose;
      Serial.println("Gripper commanded to close.");
    }
  }
  break;

  case Challenge_043_GrabPlate:
  {
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_Idle;
    if (gripperState == GripperState_Closed) {
      Serial.println("Gripper closed.");
      liftMovementDone = false;
      setTicksAtAngleLift(ROOF_25_LIFTED_ANGLE);
      activeMovementStartTime = millis();
      challengeState = Challenge_044_LiftBeforeBack;
    }
    else if (gripperState == GripperState_Open) {
      Serial.println("Gripper failed to close.");
      challengeState = Challenge_042_WaitForGrip;
    }
  }
  break;

  case Challenge_044_LiftBeforeBack:
  {
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_ToTarget;
    Serial.println("LiftingBeforeBacking");
    if (liftMovementDone && (millis() > activeMovementStartTime + WAIT_FOR_GRIPPER_LIFT_TIME)) {
      Serial.println("Lift movement complete, beginning to move backwards.");
      setTicksAtDistance(-7);
      driveMovementDone = false;
      challengeState = Challenge_045_BackTime;
    }
  }
  break;

  case Challenge_045_BackTime:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    if (driveMovementDone == true)
    {
      Serial.println("Finished Reversing.");
      // challengeState = Challenge_050_TurnOffLine;
      challengeState = Challenge_05a_ReverseUntilLine;
    }
  }
  break;

  case Challenge_05a_ReverseUntilLine:
  {
    driveMotorState = MotorState_LineFollowReverse;
    blueMotorState = MotorState_ToTarget;

    double leftColor = analogRead(LEFT_LINE_PIN);
    double rightColor = analogRead(RIGHT_LINE_PIN);

    if (leftColor > BLACK_THRESH && rightColor > BLACK_THRESH)
    {
      // driveMotorState = MotorState_ToTarget;
      // setTicksAtDistance(0);
      Serial.println("Hit line crossing.");
      // challengeState = nextState;
      challengeState = Challenge_05b_DrivePastCross;
    }

  }
  break;

  case Challenge_05b_DrivePastCross:
  {
    driveMovementDone = false;
    drivePastCross(Challange_05c_WaitForDrivePastCross);
  }
  break;

  case Challange_05c_WaitForDrivePastCross:
  {
    if(waitForDriveMovement(Challenge_05d_TurnOffLine)) {
      setTicksAtDistance(0);
      driveMovementDone = false;
    }
  }
  break;

  case Challenge_05d_TurnOffLine:
  {
    turnOffLine(Challenge_05e_WaitForTurnOffLine, true);
  }
  break;

  case Challenge_05e_WaitForTurnOffLine:
  {
    if (waitForDriveMovement(Challenge_05f_SearchForLine)) {
      setTicksAtAngleDrive(0);
      driveMovementDone = false;
      leftBlackFlag = false;
      rightBlackFlag = false;
    }
  }
  break;

  case Challenge_05f_SearchForLine:
  {
    if(searchForLine(Challenge_060_FollowLineUntilBox, true)) {
      driveMovementDone = false;
      setTicksAtAngleDrive(0);
    }
  }
  break;

  case Challenge_060_FollowLineUntilBox:
  {
    driveMotorState = MotorState_LineFollow;
    blueMotorState = MotorState_Idle;

    // Serial.print("Rangefinder Distance: ");
    // Serial.println(rangefinder.getDistance());

    // // if (millis() > activeMovementStartTime + FOLLOW_LINE_TILL_POINTED_AT_BOX_TIME)
    // // {
    // //   setTicksAtAngleLift(0);
    // //   liftMovementDone = false;
    // //   challengeState = Challenge_061_LowerArmForBox;
    // // }
    // if (driveMovementDone) {
    //   setTicksAtAngleLift(-2);
    //   liftMovementDone = false;
    //   challengeState = Challenge_061_LowerArmForBox;
      
    // }

    if (rangefinder.getDistance() < TRANSFER_DROPOFF_OFFSET) {
      challengeState = Challenge_061_LowerArmForBox;
      setTicksAtAngleLift(TRANSFER_DROPOFF_LIFT_ANGLE);
      liftMovementDone = false;
    }
    
  }
  break;

  case Challenge_061_LowerArmForBox:
  {
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_ToTarget;
    if (liftMovementDone)
    {
      Serial.println("Finished Lowering the Arm");
      setTicksAtDistance(0);
      driveMovementDone = false;
      challengeState = Challenge_062_WaitForOpen;
      gripperState = GripperState_Open;
    }
  }
  break;

  case Challenge_062_WaitForOpen:
  {
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_ToTarget;
    if (abs(analogRead(GRIPPER_POT_PIN) - GRIPPER_OPEN_POT) < 10) {
      challengeState = Challenge_063_BackOffPlatform;
      driveMovementDone = false;
    }
  }
  break;

  case Challenge_063_BackOffPlatform:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    driveMovementDone = false;
    setTicksAtDistance(TRANSFER_BACKOFF);
    challengeState = Challenge_064_WaitForBackOff;
  } 
  break;

  case Challenge_064_WaitForBackOff:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    if (waitForDriveMovement(Challenge_070_WaitForNewPlate)) {
      setTicksAtDistance(0);
      driveMotorState = MotorState_Idle;
      challengeState = Challenge_070_WaitForNewPlate;
    }
  }
  break;

  case Challenge_070_WaitForNewPlate:
  {
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_ToTarget;
    setTicksAtAngleLift(LIFT_ANGLE_AT_PICKUP);
    if (pollForSignal(remote2)) {
      challengeState = Challenge_071_GoToNewPlate;
    }
  }
  break;

  case Challenge_071_GoToNewPlate:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    driveMovementDone = false;
    setTicksAtDistance(-TRANSFER_BACKOFF);
    challengeState = Challenge_072_WaitForGoToNewPlate;
  }
  break;

  case Challenge_072_WaitForGoToNewPlate:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    if (waitForDriveMovement(Challenge_073_GrabNewPlate)) {
      setTicksAtDistance(0);
    }
  }
  break;

  case Challenge_073_GrabNewPlate:
  {
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_Idle;

    gripperState = GripperState_CommandClose;
  }

  default:
  {
    operatingState = Operating_Done;
  }
  break;
  }
}

void setup()
{
  chassis.init();
  decoder.init();
  motor.setup();
  motor.reset();
  rangefinder.init();
  Serial.begin(9600);
  servo.setMinMaxMicroseconds(900, 2500); // 500 is closed, 2000 is opened

  operatingState = Operating_Idle;
  // challengeState = Challenge_040_ApproachPanel;
  //challengeState = Challenge_010_StartPreTurn;
  challengeState = Challenge_070_WaitForNewPlate;
  driveMotorState = MotorState_Idle;
  blueMotorState = MotorState_Idle;
  gripperState = GripperState_Open;

  maxDrivePower = BASE_MAX_DRIVE_POWER;
  maxLiftPower = BASE_MAX_LIFT_POWER;

  // Delay so that the robot does not move away while the user is
  // still touching it.
  delay(2000);
  Serial.println("Program start!");
}

void loop()
{
  updateOpMode();
  updateMotors();
  updateGripperState();
}
