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

Chassis chassis;
BlueMotor motor;
Romi32U4ButtonB buttonB;
IRDecoder decoder(IR_REMOTE_PIN);
Servo32U4 servo;
Rangefinder rangefinder(RANGEFINDER_ECHO_PIN, RANGEFINDER_TRIG_PIN);

void setTicksAtAngle(float targetAngle);
void setTicksAtDistance(float targetDistance);
void setDriveMotorPower(Motor whichMotor, int deltaCM, int maxPower);
void updateOpMode();
void updateMotors();
bool pollForSignal(int whichSignal);
void doChallenge();

void setTicksAtAngle(float targetAngle) // Angle direction is determined by sign, positive is anticlockwise
{
  int ticksToGo = targetAngle * (chassis.robotRadius * PI / 180.0) / chassis.cmPerEncoderTick;
  // int motorTargets[2]; // 0 is left, 1 is right
  leftMotorTarget = chassis.leftMotor.getCount() - ticksToGo;
  rightMotorTarget = chassis.rightMotor.getCount() + ticksToGo;
  // return motorTargets;
}

void setTicksAtDistance(float targetDistance) // Distance in cm, direction is determined by sign
{
  int ticksToGo = targetDistance / chassis.cmPerEncoderTick;
  leftMotorTarget = chassis.leftMotor.getCount() + ticksToGo;
  rightMotorTarget = chassis.rightMotor.getCount() + ticksToGo;
}

void setDriveMotorPower(Motor whichMotor, int deltaCM, int maxPower)
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

void setDriveMotorPower(Motor whichMotor, int power)
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
    blueMotorState = MotorState_Holding; // if the plate's being held, keep holding! if its not, still keep holding!

    // poll for resume signal
    if (pollForSignal(IR_BUTTON_RESUME))
    {
      operatingState = Operating_Running;
    }
    if (pollForSignal(IR_BUTTON_RESET)) 
    //note: this bit is outside of isa 88 spec, but this a) isn't a batch control process and b) is helpful
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
  case MotorState_ToTarget:
    // proportional control w/variable setpoint (moving right round baby right round)
    {
      leftDriveCurrentTicks = chassis.leftMotor.getCount();
      rightDriveCurrentTicks = chassis.rightMotor.getCount();
      float leftDelta = (leftMotorTarget - leftDriveCurrentTicks) * chassis.cmPerEncoderTick;
      float rightDelta = (rightMotorTarget - rightDriveCurrentTicks) * chassis.cmPerEncoderTick;
      float blueDelta = (blueMotorTarget - blueMotorCurrentTicks);
      setDriveMotorPower(Motor_LeftDrive, leftDelta, maxDrivePower);
      setDriveMotorPower(Motor_RightDrive, rightDelta, maxDrivePower);
      setDriveMotorPower(Motor_Blue, blueDelta, maxLiftPower);

      if (abs(leftDelta) < DRIVE_TOLERANCE || abs(rightDelta) < DRIVE_TOLERANCE)
      {
        driveMovementDone = true;
      }
      if (blueDelta < LIFT_TOLERANCE)
      {
        liftMovementDone = true;
      }
    }
    // Serial.print("Left Delta: ");
    // Serial.println(leftDelta);
    break;
  case MotorState_LineFollow:
  {
    float leftColor = analogRead(LEFT_LINE_PIN);
    float rightColor = analogRead(RIGHT_LINE_PIN);
    float colorDelta = leftColor - rightColor;
    setDriveMotorPower(Motor_LeftDrive, LINE_FOLLOW_SPEED - LINE_FOLLOW_KP * colorDelta);
    setDriveMotorPower(Motor_RightDrive, LINE_FOLLOW_SPEED + LINE_FOLLOW_KP * colorDelta);

    // Serial.print("Left Power: ");
    // Serial.println(Motor_LeftDrive, LINE_FOLLOW_SPEED - LINE_FOLLOW_KP * colorDelta);
  }
  break;
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
    setTicksAtAngle(90);
    driveMovementDone = false;
    challengeState = Challenge_011_WaitForTurn;
    Serial.println("010 - Starting Pre Turn");
  }
  break;

  case Challenge_011_WaitForTurn:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    Serial.println("011 - Wait For Turn");
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
    Serial.println("012 - Search for Line");
    setTicksAtAngle(10);
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
      setTicksAtAngle(0);
      challengeState = Challenge_020_FollowLine;
    }
  }
  break;

  case Challenge_020_FollowLine:
  {
    driveMotorState = MotorState_LineFollow;
    blueMotorState = MotorState_ToTarget;
    float leftColor = analogRead(LEFT_LINE_PIN);
    float rightColor = analogRead(RIGHT_LINE_PIN);

    if (leftColor > BLACK_THRESH && rightColor > BLACK_THRESH)
    {
      // driveMotorState = MotorState_ToTarget;
      // setTicksAtDistance(0);
      challengeState = Challenge_030_ForwardAtCross;
    }
  }
  break;

  case Challenge_030_ForwardAtCross:
  {
    // driveMotorState = MotorState_ToTarget;
    // blueMotorState = MotorState_ToTarget;
    // driveMovementDone = false;
    // setTicksAtAngle(60);
    // challengeState = Challenge_031_WaitForForwardAtCross;

    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    driveMovementDone = false;
    setTicksAtDistance(7);
    challengeState = Challenge_031_WaitForForwardAtCross;   
  }
  break;

  case Challenge_031_WaitForForwardAtCross:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    if (driveMovementDone) {
      setTicksAtDistance(0);
      challengeState = Challenge_032_StartTurnAtCross;
    }
  }
  break; 

  case Challenge_032_StartTurnAtCross:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    driveMovementDone = false;
    setTicksAtAngle(-60);
    challengeState = Challenge_033_WaitForTurnAtCross;
  }
  break;

  case Challenge_033_WaitForTurnAtCross:
  {
    driveMotorState = MotorState_ToTarget;
    blueMotorState = MotorState_ToTarget;
    if (driveMovementDone) {
      setTicksAtDistance(0);
      challengeState = Challenge_034_SearchForLineAtCross;
      leftBlackFlag = false;
      rightBlackFlag = false;
    }
  }
  break;

  case Challenge_034_SearchForLineAtCross:
  {
    setTicksAtAngle(-10);
    if (!rightBlackFlag)
    {
      rightBlackFlag = analogRead(RIGHT_LINE_PIN) > BLACK_THRESH;
    }
    if (!leftBlackFlag && rightBlackFlag)
    {
      leftBlackFlag = analogRead(LEFT_LINE_PIN) > BLACK_THRESH;
    }

    if (leftBlackFlag && rightBlackFlag)
    {
      setTicksAtAngle(0);
      challengeState = Challenge_040_StartSearch;
    }
  }
  break;

  case Challenge_040_StartSearch:
  {
    operatingState = Operating_Done;
  }
  break;

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
  challengeState = Challenge_010_StartPreTurn;
  driveMotorState = MotorState_Idle;
  blueMotorState = MotorState_Idle;

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
}
