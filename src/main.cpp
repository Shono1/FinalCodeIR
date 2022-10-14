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

Chassis chassis;
BlueMotor motor;
Romi32U4ButtonB buttonB;
IRDecoder decoder(IR_REMOTE_PIN);
Servo32U4 servo;
Rangefinder rangefinder(RANGEFINDER_ECHO_PIN, RANGEFINDER_TRIG_PIN);

int goalIndex = 0;

int correctDrivePowerDeadband(int power)
{
    if (power == 0)
    {
        return 0;
    }
    return map(abs(power), 0, MAX_POWER, DRIVE_DEADBAND, MAX_POWER) * (power / abs(power));
}

void setMotorPowers(int leftEffort, int rightEffort)
{
    chassis.leftMotor.setMotorEffort(correctDrivePowerDeadband(leftEffort));
    chassis.rightMotor.setMotorEffort(correctDrivePowerDeadband(rightEffort));
}

void setBluePower(int effort)
{
    if (effort < 0)
    {
        effort = -effort;
        motor.setEffortDB(effort, true);
    }
    else
    {
        motor.setEffortDB(effort, false);
    }
}

void followLine(int speed)
{
    double leftColor = analogRead(LEFT_LINE_PIN);
    double rightColor = analogRead(RIGHT_LINE_PIN);
    double colorDelta = leftColor - rightColor;
    if (speed < 0)
    {
        setMotorPowers(speed - LINE_FOLLOW_KP * colorDelta, speed + LINE_FOLLOW_KP * colorDelta);
    }
    else
    {
        setMotorPowers(speed - LINE_FOLLOW_KP * colorDelta, speed + LINE_FOLLOW_KP * colorDelta);
    }
}

void blindForward(int speed)
{
    setMotorPowers(speed, speed);
}

void stopMotors()
{
    setMotorPowers(0, 0);
}

double getLeftMotorCM()
{
    return chassis.leftMotor.getCount() * chassis.cmPerEncoderTick;
}

double getRightMotorCM()
{
    return chassis.rightMotor.getCount() * chassis.cmPerEncoderTick;
}

double getBlueMotorAngle()
{
    return ((double)motor.getPosition()) / TICKS_PER_LIFT_OUTPUT_DEGREE;
}

class Goal
{
public:
    virtual bool updateGoal() = 0; // false if goal incomplete, true if complete
};

class LineFollowUntilCrossing : public Goal
{
public:
    LineFollowUntilCrossing(int commandedPower = 50)
    {
        powerCommand = commandedPower;
    }

    bool updateGoal()
    {
        followLine(powerCommand); // follow the line

        //  if both sensors detect a line
        if (analogRead(LEFT_LINE_PIN) > BLACK_THRESH && analogRead(RIGHT_LINE_PIN) > BLACK_THRESH)
        {
            stopMotors();
            return true; // done
        }

        return false; // not done
    }

private:
    int powerCommand;
};

class LineFollowDistance : public Goal
{
public:
    LineFollowDistance(double distance_cm, int max_speed = 50)
    {
        targetDistance = distance_cm;
        speedLimit = max_speed;
    }

    bool updateGoal()
    {
        if (!gotStartingDistances)
        {
            startLeft = getLeftMotorCM();
            startRight = getRightMotorCM();
            gotStartingDistances = true;
        }
        double traveledDistance = ((getLeftMotorCM() - startLeft) + (getRightMotorCM() - startRight)) / 2;
        double deltaDistance = targetDistance - traveledDistance;

        if (abs(deltaDistance) < DRIVE_TOLERANCE)
        {
            stopMotors();
            return true; // done motion
        }

        followLine(constrain(deltaDistance * DRIVE_KP, -speedLimit, speedLimit));
        return false; // not done
    }

private:
    double targetDistance;
    int speedLimit;
    double startLeft = 0;
    double startRight = 0;
    bool gotStartingDistances = false;
};

class LineFollowTime : public Goal
{
public:
    LineFollowTime(unsigned int time_ms, int commandedPower = 50)
    {
        millisToWait = time_ms;
        motorPow = correctDrivePowerDeadband(commandedPower);
    }

    bool updateGoal()
    {
        if (!gotStartingTime)
        {
            startMillis = millis();
            gotStartingTime = true;
        }
        if (millis() - startMillis > millisToWait)
        {
            stopMotors();
            return true;
        }
        followLine(motorPow);
        return false;
    }

private:
    unsigned int millisToWait;
    int motorPow;
    bool gotStartingTime = false;
    unsigned long startMillis = 0;
};

class LineFollowRange : public Goal
{
public:
    LineFollowRange(double thresholdDistance, bool distanceRising=false,int speed = 50)
    {
        targetDistance = thresholdDistance;
        commandedSpeed = speed;
        startedLess = distanceRising;
    }

    bool updateGoal()
    {
        double deltaRange = targetDistance - rangefinder.getDistance();
        followLine(deltaRange * DRIVE_KP);
        if ((startedLess && deltaRange < 0) || deltaRange > 0)
        {
            stopMotors();
            return true;
        }
        return false;
    }

private:
    double targetDistance;
    int commandedSpeed;
    bool startedLess = false;
    bool gotStartingSign = false;
};

class RotateUntilLine : public Goal
{
public:
    RotateUntilLine(bool goClockwise = false, int speed = 50)
    {
        clockwise = goClockwise;
        commandedSpeed = speed;
    }

    bool updateGoal()
    {
        if (clockwise)
        {
            if (analogRead(RIGHT_LINE_PIN) > BLACK_THRESH)
            {
                stopMotors();
                return true;
            }
            setMotorPowers(commandedSpeed, -commandedSpeed);
            return false;
        }
        else // counterclockwise
        {
            if (analogRead(LEFT_LINE_PIN) > BLACK_THRESH)
            {
                stopMotors();
                return true;
            }
            setMotorPowers(-commandedSpeed, commandedSpeed);
            return false;
        }
    }

private:
    bool clockwise;
    int commandedSpeed;
};

class BlindRotateAngle : public Goal
{
public:
    BlindRotateAngle(double angle, int max_speed = 50)
    {
        targetAngle = angle;
        speedLimit = max_speed;
    }

    bool updateGoal()
    {
        if (!gotStartingMotorCounts)
        {
            startLeft = getLeftMotorCM();
            startRight = getRightMotorCM();
            gotStartingMotorCounts = true;
        }
        double leftDistTraveled = getLeftMotorCM() - startLeft;
        double rightDistTraveled = getRightMotorCM() - startRight;
        double robotCircumference = 2 * PI * chassis.robotRadius;
        double angleTraveledLeft = -(leftDistTraveled / robotCircumference) * 360.0;
        double angleTraveledRight = (rightDistTraveled / robotCircumference) * 360.0;
        double totalAngleTraveled = angleTraveledLeft + angleTraveledRight;

        double deltaAngle = targetAngle - totalAngleTraveled;
        double turnPower = constrain(deltaAngle * DRIVE_KP, -speedLimit, speedLimit);

        if (abs(deltaAngle) < DRIVE_TOLERANCE)
        {
            stopMotors();
            return true;
        }
        setMotorPowers(-turnPower, turnPower);
        return false;
    }

private:
    double targetAngle;
    int speedLimit;
    double startLeft = 0;
    double startRight = 0;
    double gotStartingMotorCounts = false;
};

class BlindDriveDistance : public Goal
{
public:
    BlindDriveDistance(double distance_cm, int max_speed = 50)
    {
        targetDistance = distance_cm;
        commandedSpeed = max_speed;
    }

    bool updateGoal()
    {
        if (!gotStartingDistance)
        {
            startLeft = getLeftMotorCM();
            startRight = getRightMotorCM();
            gotStartingDistance = true;
        }
        double currentDistance = ((getLeftMotorCM() - startLeft) + (getRightMotorCM() - startRight)) / 2;
        double deltaDistance = targetDistance - currentDistance;

        if (abs(deltaDistance) < DRIVE_TOLERANCE)
        {
            stopMotors();
            return true;
        }

        double motorPow = constrain(deltaDistance * DRIVE_KP, -commandedSpeed, commandedSpeed);
        setMotorPowers(motorPow, motorPow);
        return false;
    }

private:
    double targetDistance;
    double commandedSpeed;
    double startLeft = 0;
    double startRight = 0;
    bool gotStartingDistance = false;
};

class BlindDriveTime : public Goal
{
public:
    BlindDriveTime(unsigned int time_ms, int commandedPower = 50)
    {
        millisToWait = time_ms;
        motorPow = correctDrivePowerDeadband(commandedPower);
    }

    bool updateGoal()
    {
        if (!gotStartingTime)
        {
            startMillis = millis();
            gotStartingTime = true;
        }
        if (millis() - startMillis > millisToWait)
        {
            stopMotors();
            return true;
        }
        setMotorPowers(motorPow, motorPow);
        return false;
    }

private:
    unsigned int millisToWait;
    int motorPow;
    bool gotStartingTime = false;
    unsigned int startMillis = 0;
};

class WaitForButtonPress : public Goal
{
public:
    WaitForButtonPress(int button)
    {
        targetButton = button;
    }

    bool updateGoal()
    {
        int currentButton = decoder.getKeyCode(true);
        if (currentButton == targetButton)
        {
            return true;
        }
        return false;
    }

private:
    int targetButton;
};

class MoveLifter : public Goal
{
public:
    MoveLifter(double absoluteAngle)
    {
        targetAngle = absoluteAngle;
    }

    bool updateGoal()
    {
        double deltaAngle = targetAngle - getBlueMotorAngle();

        if (abs(deltaAngle) < LIFT_TOLERANCE)
        {
            setBluePower(0);
            return true;
        }

        setBluePower(400*(deltaAngle / abs(deltaAngle)));
        return false;
    }

private:
    double targetAngle;
};

class OpenGripper : public Goal
{
public:
    bool updateGoal()
    {
        servo.writeMicroseconds(GRIPPER_OPEN_MS);
        return true;
    }
};

class CloseGripper : public Goal
{
public:
    bool updateGoal()
    {
        if (!gotStartMillis)
        {
            servo.writeMicroseconds(GRIPPER_CLOSED_MS);
            startMillis = millis();
            gotStartMillis = true;
        }

        if (millis() - startMillis > GRIPPER_TIMEOUT)
        {
            if (analogRead(GRIPPER_POT_PIN) > GRIPPER_CLOSED_POT)
            {
                // FAILURE
                servo.writeMicroseconds(GRIPPER_OPEN_MS);
                return true;
            }
            else
            {
                // SUCCESS
                return true;
            }
        }
        return false;
    }

private:
    unsigned int startMillis;
    bool gotStartMillis = false;
};

class IndefinitelyReadGripper : public Goal
{
    public:
    bool updateGoal()
    {
        Serial.print("Gripper reads: ");
        Serial.println(analogRead(GRIPPER_POT_PIN));

        return false;
    }
};

class Done : public Goal
{
public:
    bool updateGoal()
    {
        Serial.println("Done.");
        operatingState = Operating_Done;
        return false; // Exception to the rule: never pass this goal
    }
};

Goal *targets[] =
    {
        new OpenGripper(),
        new BlindRotateAngle(-90),
        new RotateUntilLine(true),
        new LineFollowUntilCrossing(),
        new BlindDriveDistance(CROSS_PASSING_DIST),
        new BlindRotateAngle(45),
        new RotateUntilLine(),
        new MoveLifter(ROOF_25_ANGLE),
        new LineFollowRange(RANGE_LATE_APPROACH_TOWER),
        new LineFollowTime(2000, 0),
        new MoveLifter(ROOF_25_ANGLE),
        new BlindDriveTime(FINAL_APPROACH_WAIT_TIME, 1),
        new WaitForButtonPress(remote1),
        new CloseGripper(),
        new MoveLifter(ROOF_25_LIFTED_ANGLE),
        new LineFollowUntilCrossing(-80),
        new BlindDriveDistance(4.5*2.54),
        new RotateUntilLine(),
        new LineFollowDistance(10, 20),
        new LineFollowRange(TRANSFER_DROPOFF_OFFSET),
        new MoveLifter(0),
        new OpenGripper(),
        new BlindDriveTime(500, 0),
        new BlindDriveDistance(-20, 20),
        new WaitForButtonPress(remote2),
        new BlindDriveDistance(20, 20),
        new CloseGripper(),
        new BlindDriveDistance(-20, 20),
        new BlindRotateAngle(-90),
        new RotateUntilLine(true),
        new LineFollowUntilCrossing(),
        new BlindDriveDistance(CROSS_PASSING_DIST),
        new BlindRotateAngle(45),
        new RotateUntilLine(),
        new MoveLifter(ROOF_25_LIFTED_ANGLE),
        new LineFollowRange(RANGE_EARLY_APPROACH_TOWER),
        new LineFollowTime(2000, 0),
        new MoveLifter(ROOF_25_FINAL_ANGLE),
        new OpenGripper(),
        new BlindDriveTime(1000, 0),

        new Done()
};


// Goal* potMode[] =
// {
//     new IndefinitelyReadGripper(),
//     new Done()
// };

void setup()
{
    chassis.init();
    decoder.init();
    motor.setup();
    motor.reset();
    rangefinder.init();
    Serial.begin(9600);
    servo.setMinMaxMicroseconds(900, 2800);
    Serial.println("Idling.");
}

void loop()
{
    switch (operatingState)
    {
    case Operating_Idle:
        stopMotors();
        if (decoder.getKeyCode(true) == IR_BUTTON_START)
        {
            Serial.println("Running.");
            operatingState = Operating_Running;
        }
        break;
    case Operating_Running:
        if (targets[goalIndex]->updateGoal())
        {
            Serial.print("Step complete: ");
            Serial.println(goalIndex);
            goalIndex++;
        }
        if (decoder.getKeyCode(true) == IR_BUTTON_PAUSE)
        {
            Serial.println("Paused.");
            operatingState = Operating_Paused;
        }
        break;
    case Operating_Paused:
        stopMotors();
        setBluePower(0);
        if (decoder.getKeyCode(true) == IR_BUTTON_RESUME)
        {
            Serial.println("Resumed.");
            operatingState = Operating_Running;
        }
        break;
    case Operating_Done:
        stopMotors();
        setBluePower(0);
        if (decoder.getKeyCode(true) == IR_BUTTON_RESET)
        {
            Serial.println("Idling.");
            operatingState = Operating_Idle;
        }
    }
}