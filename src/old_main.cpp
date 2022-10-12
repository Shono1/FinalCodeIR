// #include <Arduino.h>
// #include <Chassis.h>
// #include <Romi32U4.h>
// #include <RemoteConstants.h>
// #include <IRdecoder.h>
// #include <Timer.h>
// #include <servo32u4.h>
// #include "BlueMotor.h"
// #include <Rangefinder.h>

// int irRemotePin = 14;
// float forwardSpeed = 0;
// float turnSpeed = 0;
// int target;
// float motorEffort = 400;
// int t = 500, tnew, told, move;
// volatile int checkval[5];
// int ind = 0;
// int leftRef = 20;
// int rightRef = 22;
// int gogo = 0, nono = 0;
// float distinch, inval;

// float kTurning = 0.5; // TODO: ACTUALLY FIND THIS VALUE

// const int BLACK_THRESH = 400;

// enum stateChoices
// {
//   FollowLine,
//   StartPreTurn,
//   StartWait,
//   StartSearch,
//   TurnatCrossing,
//   ApproachPanel,
//   ApproachPlacement,
//   TurnAround,
//   MovetoAngleOne,
//   MovetoAngleTwo,
//   OpentheGripper,
//   ClosetheGripper,
//   Stop,
//   StopIdle
// } romiState,
//     nextRomiState;

// Chassis chassis;
// BlueMotor motor;
// Romi32U4ButtonB buttonB;
// IRDecoder decoder(irRemotePin);
// Servo32U4 servo;
// Rangefinder rangefinder(17, 12);

// void OpenGripper();
// void CloseGripper();
// void TurnonLine();
// void LineFollow();
// void TurningPoint();
// void ApproachPannel(float inval);

// void setup()
// {
//   chassis.init();
//   decoder.init();
//   motor.setup();
//   motor.reset();
//   rangefinder.init();
//   Serial.begin(9600);
//   // Wait for the user to press button B.
//   buttonB.waitForButton();
//   Serial.println("Button B Pressed");
//   servo.setMinMaxMicroseconds(900, 2500); // 500 is closed, 2000 is opened

//   // Delay so that the robot does not move away while the user is
//   // still touching it.
//   delay(2000);
// }

// // Handles a key press on the IR remote
// void handleKeyPress(int keyPress)
// {
//   if (keyPress == remotePlayPause)
//   {
//     forwardSpeed = 0;
//     turnSpeed = 0;
//     chassis.setWheelSpeeds(forwardSpeed, forwardSpeed);
//     Serial.print("Hello Stop  ");
//     Serial.println(forwardSpeed);
//   }

//   if (keyPress == remoteUp)
//   {
//     forwardSpeed = forwardSpeed + 5.0;
//     chassis.setWheelSpeeds(forwardSpeed, forwardSpeed);
//     Serial.print("Hello Forward  ");
//     Serial.println(forwardSpeed);
//   }

//   if (keyPress == remoteDown)
//   {
//     forwardSpeed = forwardSpeed - 5.0;
//     Serial.print("Hello Backward  ");
//     Serial.println(forwardSpeed);
//     chassis.setWheelSpeeds(forwardSpeed, forwardSpeed);
//   }
//   if (keyPress == remote1) // Fourbar Up
//   {
//     target = 3000; // less steep
//     motor.moveTo(target, motorEffort);
//   }
//   if (keyPress == remote2) // Fourbar down
//   // Fourbar will need to move DOWN in order to lift the plate off of the roof
//   {
//     // target = -2900; //modify to make a value for the 3 positions
//     target = 0;
//     motor.moveTo(target, motorEffort);
//   }
//   if (keyPress == remote3) // gripper open
//   {
//     OpenGripper();
//   }
//   if (keyPress == remote4) // gripper close. default is closed
//   {
//     CloseGripper();
//   }
//   if (keyPress == remote5)
//   {
//     target = 2300; // grab the 25 platform
//     motor.moveTo(target, motorEffort);
//   }
//   if (keyPress == remote6)
//   {
//     target = 10;
//     motor.moveTo(target, motorEffort);
//   }
//   if (keyPress == remote7)
//   {
//     target = 3500;
//     motor.moveTo(target, motorEffort);
//   }
//   if (keyPress == remote8)
//   {
//     target = 2000;
//     motor.moveTo(target, motorEffort);
//   }
//   if (keyPress == remote9)
//   {
//     romiState == MovetoAngleTwo;
//   }
// }

// bool *isBlack()
// {
//   bool sensed[2];
//   sensed[0] = analogRead(leftRef) > BLACK_THRESH;
//   sensed[1] = analogRead(rightRef) > BLACK_THRESH;
//   return sensed;
// }

// void stopAllDriveMotors()
// {
//   chassis.leftMotor.setMotorEffort(0);
//   chassis.rightMotor.setMotorEffort(0);
// }

// int * setTicksAtAngle(float targetAngle) // Angle direction is determined by sign, positive is anticlockwise
// {
//   int ticksToGo = targetAngle * (chassis.robotRadius * 3.14 / 180.0) / chassis.cmPerEncoderTick;
//   int motorTargets[2]; // 0 is left, 1 is right
//   motorTargets[0] = chassis.leftMotor.getEncoderCount() - ticksToGo;
//   motorTargets[1] = chassis.rightMotor.getEncoderCount() + ticksToGo;
//   return motorTargets;
// }

// int setTicksAtDistance(float targetDistance) // Distance in cm, direction is determined by sign
// {
//   int ticksToGo = targetDistance / chassis.cmPerEncoderTick;
//   return t
// }

// bool leftLineFlag, rightLineFlag = false;
// int leftMotorPower, rightMotorPower, liftMotorPower;

// void loop()
// {
//   int keyPress = decoder.getKeyCode(); // true allows the key to repeat if held down
//   if (keyPress >= 0)
//     handleKeyPress(keyPress);

//   switch (romiState)
//   {
//   case StartPreTurn:
//     // From facing the placement box, turn 45 degrees to get sensors off line
//     chassis.turnFor(45, 30, false);
//     romiState = StartWait;
//     break;
//   case StartWait:
//     // Wait until initial turn is done
//     if (chassis.checkMotionComplete())
//     {
//       romiState = StartSearch;
//       chassis.leftMotor.setMotorEffort(-200);
//       chassis.rightMotor.setMotorEffort(200);
//     }
//     break;
//   case StartSearch:
//     break;
//   case FollowLine:
//     // Follow the line proportionally
//     break;
//   case TurnatCrossing:
//     // Drive past the cross point and then turn until line is picked up again
//     break;
//   case ApproachPanel:
//     // Drive to panel with line following and ultrasonic rangefinder until pickup point is reached
//     break;
//   case TurnAround:
//     // 180 degree turn on line
//     break;
//   case ApproachPlacement:
//     // Approach placement with line following and rangefinder until placement point is reached
//     break;
//   case MovetoAngleOne: // 25 degree roof
//                        // Move the fourbar to the motor count for roof angle one
//     break;
//   case MovetoAngleTwo: // 45 degree roof
//     // find first arm position
//     // drive until 20.5 cm away, then as you move to 18.5 cm away also raise the fourbar up
//     inval = 20.5;
//     target = 2000;
//     motorEffort = 400;
//     ApproachPannel(inval);
//     delay(50);
//     motor.moveTo(target, motorEffort);
//     delay(50);
//     inval = 18.5;
//     target = 2500;
//     motorEffort = 400;
//     while ((rangefinder.getDistance()) > inval)
//     {
//       ApproachPannel(inval);
//       motor.moveTo(target, motorEffort);
//     }

//     break;
//   case OpentheGripper:
//     // Gripper is opened
//     break;
//   case ClosetheGripper:
//     // Gripper is closed
//     break;
//   case Stop:
//     stopAllDriveMotors();
//     leftMotorPower = chassis.leftMotor.getCount();
//     break;
//   }
// }

// void OpenGripper()
// {
//   Serial.print("beg");
//   Timer b = Timer(2000);
//   tnew = 2500;
//   for (move = told; move <= tnew; move += 50)
//   {
//     servo.writeMicroseconds(move);
//     Serial.println(analogRead(18));
//     delay(20);
//     told = move;
//   }

//   while (analogRead(18) < 430)
//   {
//     if (b.isExpired())
//     {
//       Serial.print("boo");
//       CloseGripper();
//       return;
//     }
//   }
//   b.reset();
// }

// void CloseGripper()
// {
//   Serial.print("baz");
//   Timer a = Timer(2000);
//   tnew = 900;
//   for (move = told; move >= tnew; move -= 50)
//   {
//     servo.writeMicroseconds(move);
//     Serial.println(analogRead(18));
//     delay(20);
//     told = move;
//   }
//   while (analogRead(18) > 250)
//   {
//     if (a.isExpired())
//     {
//       Serial.print("end");
//       OpenGripper();
//       return;
//     }
//   }
//   a.reset();
// }

// void TurnonLine()
// {
//   while (((analogRead(rightRef)) - (analogRead(leftRef))) < 100) // At start, rotate left to get off of line
//   {
//     chassis.setTwist(0.0, 45.0);
//   }
//   while ((((analogRead(rightRef)) - (analogRead(leftRef))) > 100) | (((analogRead(leftRef)) - (analogRead(rightRef))) < 100)) // After already off line, keep rotating left until line reacquired
//   {
//     chassis.setTwist(0.0, 45.0);
//   }
//   chassis.turnFor(15.0, 45.0, 1); // turn slightly more to get both sensors on line
// }

// void LineFollow()
// {
//   while (((analogRead(leftRef)) & (analogRead(rightRef))) < 700)
//   {
//     chassis.setWheelSpeeds(10.0, 10.0);                         // drive forward
//     if (((analogRead(rightRef)) - (analogRead(leftRef))) > 200) // if veering left, turn right to correct
//     {
//       chassis.setTwist(0.0, -20.0);
//     }
//     if (((analogRead(leftRef)) - (analogRead(rightRef))) > 200) // if veering right, turn left to correct
//     {
//       chassis.setTwist(0.0, 20.0);
//     }
//   }
//   chassis.driveFor(8.0, 10.0, 1);   // drive until center of rotation is at crossing point
//   chassis.setWheelSpeeds(0.0, 0.0); // stop romi and prepare to rotate
// }

// void TurningPoint()
// {
//   chassis.driveFor(8.0, 10.0, 1);   // drive until center of rotation is at crossing point
//   chassis.setWheelSpeeds(0.0, 0.0); // stop romi and prepare to rotate
// }

// void ApproachPannel(float inval)
// {
//   while ((rangefinder.getDistance()) > inval) // distance in cm
//   {
//     chassis.setWheelSpeeds(10.0, 10.0);                                                                                         // drive forward
//     while ((((analogRead(leftRef)) - (analogRead(rightRef))) < 400) & (((analogRead(rightRef)) - (analogRead(leftRef))) < 400)) // while on line
//     {
//       (rangefinder.getDistance());
//       if (((analogRead(rightRef)) - (analogRead(leftRef))) > 200) // if veering left, turn right to correct
//       {
//         chassis.setTwist(0.0, -20.0);
//       }
//       if (((analogRead(leftRef)) - (analogRead(rightRef))) > 200) // if veering right, turn left to correct
//       {
//         chassis.setTwist(0.0, 20.0);
//       }
//     } // breaks when drive condition is reached
//   }
// }
