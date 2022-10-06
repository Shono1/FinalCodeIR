#include <Arduino.h>
#include <Chassis.h>
#include <Romi32U4.h>
#include <RemoteConstants.h>
#include <IRdecoder.h>
#include <Timer.h>
#include <servo32u4.h>
#include "BlueMotor.h"

int irRemotePin = 14;
float forwardSpeed = 0;
float turnSpeed = 0;
int target, motorEffort = 400;
int t = 500, tnew, told, move;
volatile int checkval[5];
int ind = 0;

Chassis chassis;
BlueMotor motor;
Romi32U4ButtonB buttonB;
IRDecoder decoder(irRemotePin);
Servo32U4 servo;

void OpenGripper();
void CloseGripper();

void setup()
{
  chassis.init();
  decoder.init();
  motor.setup();
  motor.reset();
  Serial.begin(9600);
  // Wait for the user to press button B.
  buttonB.waitForButton();
  Serial.println("Button B Pressed");
  servo.setMinMaxMicroseconds(900, 2500); //500 is closed, 2000 is opened
   
  // Delay so that the robot does not move away while the user is
  // still touching it.
  delay(2000);
}

// Handles a key press on the IR remote
void handleKeyPress(int keyPress)
{
  if (keyPress == remotePlayPause)
  {
    forwardSpeed = 0;
    turnSpeed = 0;
    chassis.setWheelSpeeds(forwardSpeed, forwardSpeed);
    Serial.print("Hello Stop  ");
    Serial.println(forwardSpeed);
  }

  if (keyPress == remoteUp)
  {
    forwardSpeed = forwardSpeed + 5.0;
    chassis.setWheelSpeeds(forwardSpeed, forwardSpeed);
    Serial.print("Hello Forward  ");
    Serial.println(forwardSpeed);
  }

  if (keyPress == remoteDown)
  {
    forwardSpeed = forwardSpeed - 5.0;
    Serial.print("Hello Backward  ");
    Serial.println(forwardSpeed);
    chassis.setWheelSpeeds(forwardSpeed, forwardSpeed);
  }
  if (keyPress == remote1) //Fourbar Up
  {
    target = 3000; //less steep
    motor.moveTo(target, motorEffort);
  }
  if (keyPress == remote2) //Fourbar down
  //Fourbar will need to move DOWN in order to lift the plate off of the roof
  {
    //target = -2900; //modify to make a value for the 3 positions 
    target = 0;
    motor.moveTo(target, motorEffort);
  }
  if (keyPress == remote3) //gripper open
  {
    OpenGripper();
  }
  if (keyPress == remote4) //gripper close. default is closed
  {
    CloseGripper();
  }
  if (keyPress == remote5)
  {
    target = 2300; //grab the 25 platform
    motor.moveTo(target, motorEffort);
  }
  if (keyPress == remote6)
  {
    target = 150;
    motor.moveTo(target, motorEffort);
  }
  if (keyPress == remote7)
  {
    target = 3500;
    motor.moveTo(target, motorEffort);
  }
  if (keyPress == remote8)
  {
    target = 2000;
    motor.moveTo(target, motorEffort);
  }
  if (keyPress == remote9)
  {
    target = 1500;
    motor.moveTo(target, motorEffort);
  }
}
void loop()
{
  // Check for a key press on the remote
  
  int keyPress = decoder.getKeyCode(); //true allows the key to repeat if held down
  if(keyPress >= 0) handleKeyPress(keyPress);
  
}

// void OpenGripper()
// {
//   tnew = 2500;
//   for (move = told; move <= tnew; move += 50)
//   {
//     servo.writeMicroseconds(move);
//     delay(20);
//     told = move;
//     checkval[ind] = move;
//     /*
//     ind++;
//     if (ind == 5)
//     {
//       if (checkval[4] == checkval[0])
//       {
//         servo.writeMicroseconds(500);
//         CloseGripper();
//         break;
//       }
//       ind = 0;
//     }
//     */
//    if (ind > 3)
//    {
//     if (abs(checkval[ind] - checkval[ind - 1]) < 100)
//     {
//       //OpenGripper();
//       servo.writeMicroseconds(900);
//       ind = 0;
//       checkval[4] = 0;
//       checkval[3] = 0;
//       checkval[2] = 0;
//       checkval[1] = 0;
//       checkval[0] = 0;
//       break;
//     }
//    }
//    ind++;
//   }
//   ind = 0;
//   checkval[4] = 0;
//   checkval[3] = 0;
//   checkval[2] = 0;
//   checkval[1] = 0;
//   checkval[0] = 0;
// }

// void CloseGripper()
// {
//   tnew = 900;
//   for (move = told; move >= tnew; move -= 50)
//   {
//     servo.writeMicroseconds(move);
//     delay(20);
//     told = move;
//     checkval[ind] = move;
    
//     /*
//     if (ind == 5)
//     {
//       if (checkval[4] == checkval[0])
//       {
//         servo.writeMicroseconds(2500);
//         OpenGripper();
//         break;
//       }
//       ind = 0;
//     }
//     */
//    if (ind > 3)
//    {
//     if (abs(checkval[ind] - checkval[ind - 1]) < 100)
//     {
//       //OpenGripper();
//       servo.writeMicroseconds(2500);
//       ind = 0;
//       checkval[4] = 0;
//       checkval[3] = 0;
//       checkval[2] = 0;
//       checkval[1] = 0;
//       checkval[0] = 0;
//       break;
//     }
//    }
//    ind++;
//   }
//   checkval[4] = 0;
//   checkval[3] = 0;
//   checkval[2] = 0;
//   checkval[1] = 0;
//   checkval[0] = 0;
// }

void OpenGripper()
{
  tnew = 2500;
  for (move = told; move <= tnew; move += 50)
  {
    servo.writeMicroseconds(move);
    delay(20);
    told = move;
  }
  /*
  while(analogRead(18) > 250) {
    if (t.isExpired()) {
      Serial.println("baz");
      OpenGripper();
      return;
    }
  }
  */
}

void CloseGripper() {
  Serial.println("foo");
  Timer t = Timer(2000);
  tnew = 900;
  for (move = told; move >= tnew; move -= 50)
  {
    // Serial.println("bar");
    servo.writeMicroseconds(move);
    Serial.println(analogRead(18));
    delay(20);
    told = move;
  }
  while(analogRead(18) > 250) {
    if (t.isExpired()) {
      Serial.println("baz");
      OpenGripper();
      return;
    }
  }
  
}