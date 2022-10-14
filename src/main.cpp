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

int correctDrivePowerDeadband(int power)
{
    if (power == 0) {
        return 0;
    }
    return map(power, 0, MAX_POWER, DRIVE_DEADBAND, MAX_POWER);
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
    } else
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
        setMotorPowers(speed + LINE_FOLLOW_KP * colorDelta, speed - LINE_FOLLOW_KP * colorDelta);
    } else {
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
    return ((double) motor.getPosition()) / TICKS_PER_LIFT_OUTPUT_DEGREE;
}

class Goal
{
    public:
    virtual bool updateGoal() = 0; // false if goal incomplete, true if complete
};

class LineFollowUntilCrossing : Goal
{
    public:
    LineFollowUntilCrossing(int commandedPower=BASE_MAX_DRIVE_POWER)
    {
        powerCommand = commandedPower;
    }

    bool updateGoal()
    {
        followLine(powerCommand); // follow the line

        //  if both sensors detect a line
        if (analogRead(LEFT_LINE_PIN ) > BLACK_THRESH && analogRead(RIGHT_LINE_PIN) > BLACK_THRESH)
        {
            stopMotors();
            return true; // done
        }

        return false; // not done
    }

    private:
    int powerCommand;
};

class LineFollowDistance : Goal
{
    public:
    LineFollowDistance(double distance_cm)
    {
        targetDistance = distance_cm;
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
        
        followLine(deltaDistance * DRIVE_KP);
        return false; // not done
    }
    
    private:
    double targetDistance;
    double startLeft = 0;
    double startRight = 0;
    bool gotStartingDistances = false;
};

class LineFollowTime : Goal
{
    public:
    LineFollowTime(double time_ms, int commandedPower=BASE_MAX_DRIVE_POWER)
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
    int millisToWait;
    int motorPow;
    bool gotStartingTime = false;
    int startMillis = 0;
};

class LineFollowRange : Goal
{
    public:
    LineFollowRange(double thresholdDistance, int speed = BASE_MAX_DRIVE_POWER)
    {
        targetDistance = thresholdDistance;
        commandedSpeed = speed;
    }

    bool updateGoal()
    {
        double deltaRange = targetDistance - rangefinder.getDistance();
        if (!gotStartingSign)
        {
            startedLess = deltaRange < 0;
        }
        followLine(deltaRange*DRIVE_KP);
        if ((startedLess && deltaRange > 0) || deltaRange < 0)
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

class RotateUntilLine : Goal
{
    public:
    RotateUntilLine(bool goClockwise = false, int speed = BASE_MAX_DRIVE_POWER)
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

        } else // counterclockwise
        {
            if (analogRead(LEFT_LINE_PIN))
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

class BlindRotateAngle : Goal
{
    public:
    BlindRotateAngle(double angle)
    {
        targetAngle = angle;
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
        double angleTraveledLeft = (leftDistTraveled / robotCircumference) * 360.0;
        double angleTraveledRight = (rightDistTraveled / robotCircumference) * 360.0;
        double totalAngleTraveled = angleTraveledLeft + angleTraveledRight;

        double deltaAngle = targetAngle - totalAngleTraveled;
        double turnPower = deltaAngle * DRIVE_KP;
        if (deltaAngle < DRIVE_TOLERANCE)
        {
            stopMotors();
            return true;
        }
        setMotorPowers(-turnPower, turnPower);
        return false;
    }

    private:
    double targetAngle;
    double startLeft = 0;
    double startRight = 0;
    double gotStartingMotorCounts = false;
};

class BlindDriveDistance : Goal
{
    public:
    BlindDriveDistance(double distance_cm)
    {
        targetDistance = distance_cm;
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

        if (deltaDistance < DRIVE_TOLERANCE) {
            stopMotors();
            return true;
        }

        double motorPow = deltaDistance * DRIVE_KP;
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

class BlindDriveTime : Goal
{
    public:
    BlindDriveTime(double time_ms, int commandedPower=BASE_MAX_DRIVE_POWER)
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
    int millisToWait;
    int motorPow;
    bool gotStartingTime = false;
    int startMillis = 0;
};

class WaitForButtonPress : Goal
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

class MoveLifter : Goal
{
    public:
    MoveLifter(double absoluteAngle)
    {
        targetAngle = absoluteAngle;
    }

    bool updateGoal()
    {
        double deltaAngle = targetAngle - getBlueMotorAngle();

        if (deltaAngle < LIFT_TOLERANCE)
        {
            setBluePower(0);
            return true;
        }

        setBluePower(deltaAngle*LIFT_KP);
    }

    private:
    double targetAngle;
};

class OpenGripper : Goal
{
    public:
    OpenGripper();

    bool updateGoal()
    {
        servo.writeMicroseconds(GRIPPER_OPEN_MS);
        return true;
    }
};

class CloseGripper : Goal
{
    public:
    CloseGripper();

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
            } else {
                // SUCCESS
                return true;
            }
        }
        return false;
    }

    private:
    int startMillis;
    bool gotStartMillis = false;
};

void setup()
{
    chassis.init();
    decoder.init();
    motor.setup();
    motor.reset();
    rangefinder.init();
    Serial.begin(9600);
    servo.setMinMaxMicroseconds(900, 2800);

    Goal* targets[] = 
    {
        
    };
}

void loop(){}