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
bool driveMovementDone, liftMovementDone = false;

Chassis chassis;
BlueMotor motor;
Romi32U4ButtonB buttonB;
IRDecoder decoder(IR_REMOTE_PIN);
Servo32U4 servo;
Rangefinder rangefinder(RANGEFINDER_ECHO_PIN, RANGEFINDER_TRIG_PIN);

void setTicksAtAngle(float targetAngle) // Angle direction is determined by sign, positive is anticlockwise
{
  int ticksToGo = targetAngle * (chassis.robotRadius * PI / 180.0) / chassis.cmPerEncoderTick;
  // int motorTargets[2]; // 0 is left, 1 is right
  leftMotorTarget = chassis.leftMotor.getEncoderCount() - ticksToGo;
  rightMotorTarget = chassis.rightMotor.getEncoderCount() + ticksToGo;
  // return motorTargets;
}

void setTicksAtDistance(float targetDistance) // Distance in cm, direction is determined by sign
{
  int ticksToGo = targetDistance / chassis.cmPerEncoderTick;
	leftMotorTarget = chassis.leftMotor.getEncoderCount() + ticksToGo;
	rightMotorTarget = chassis.rightMotor.getEncoderCount() + ticksToGo;
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

void updateOpMode()
{
  switch (operatingState)
  {
  case Operating_Idle:
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_Idle;
    // poll for start signal
    break;
  case Operating_Running:
    driveMotorState = MotorState_Active;
    blueMotorState = MotorState_Active;
    doChallenge(); // do the challenge
    // poll for pause/stop signal
    break;
  case Operating_Paused:
    driveMotorState = MotorState_Holding; // allow minimum slippage for consistency (needed?)
    blueMotorState = MotorState_Holding; // if the plate's being held, keep holding!
    // poll for resume signal
    break;
  case Operating_Done:
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_Idle;
    // poll for reset signal
    break;
  case Operating_Stopped:
    driveMotorState = MotorState_Idle;
    blueMotorState = MotorState_Idle;
    // poll for reset signal
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
      break;
    case MotorState_Active:
      // proportional control w/variable setpoint (moving right round baby right round)
			float leftDelta = (leftMotorTarget - leftDriveCurrentTicks) * chassis.cmPerEncoderTick;
			float rightDelta = (rightMotorTarget - rightDriveCurrentTicks) * chassis.cmPerEncoderTick;
			float blueDelta = (blueMotorTarget - blueMotorCurrentTicks);
			setDriveMotorPower(Motor_LeftDrive, leftDelta, maxDrivePower);
			setDriveMotorPower(Motor_RightDrive, rightDelta, maxDrivePower);
			setDriveMotorPower(Motor_Blue, blueDelta, maxLiftPower);
      break;
  }
}

void updateRemoteControl()
{
  //TODO: finish
  int pollCode = decoder.getCode();
  

}

void doChallenge()
{
  switch (challengeState)
  {
  case Challenge_010_StartPreTurn:
    setTicksAtAngle(45);
    break;
  case Challenge_020_FollowLine:
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
  // Wait for the user to press button B.
  buttonB.waitForButton();
  Serial.println("Button B Pressed");
  servo.setMinMaxMicroseconds(900, 2500); // 500 is closed, 2000 is opened

  operatingState = Operating_Idle;
  challengeState = Challenge_010_StartPreTurn;
  nextChallengeState = Challenge_020_FollowLine;
  driveMotorState = MotorState_Idle;
  blueMotorState = MotorState_Idle;

	maxDrivePower = BASE_MAX_DRIVE_POWER;
	maxLiftPower = BASE_MAX_LIFT_POWER; 

  // Delay so that the robot does not move away while the user is
  // still touching it.
  delay(2000);
}

void loop()
{
  updateOpMode();
  updateMotors();
}

