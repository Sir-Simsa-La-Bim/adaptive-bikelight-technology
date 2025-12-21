#include "servoDriver.hpp"
#include <Servo.h>
#include "util.h"
#include "timestep.hpp"

// Settings
static int servoNeutralPosition = 1500;
static const float servoStepsPerDegree = 2000.0 / 270.0;
static const float servoDegreePerSecondLimit = 90;

// Derived setting values
static const float servoStepsPerRadiant = servoStepsPerDegree / DEG_TO_RAD;
static const int servoMicrosecPerStep = (int)(1e6 / (servoDegreePerSecondLimit * servoStepsPerDegree) + 0.5);

// State
static int currentPosition = servoNeutralPosition;
static int accumulatedTime_us = 0;
static Servo servo;

void initServo(int pin)
{
    servo.attach(pin);
    servo.writeMicroseconds(currentPosition);
}

void updateServo(float angle_rad)
{
    int desiredPosition = (int)(angle_rad * servoStepsPerRadiant + 0.5) + servoNeutralPosition;
    accumulatedTime_us += currentTimestep_us;
    int maxStep_dg = accumulatedTime_us / servoMicrosecPerStep;
    accumulatedTime_us -= maxStep_dg * servoMicrosecPerStep;
    int difference_dg = desiredPosition - currentPosition;

    if (difference_dg < 0)
    {
        if (-difference_dg < maxStep_dg)
            currentPosition = desiredPosition;
        else
            currentPosition -= maxStep_dg;
    }
    else
    {
        if (difference_dg < maxStep_dg)
            currentPosition = desiredPosition;
        else
            currentPosition += maxStep_dg;
    }

    servo.writeMicroseconds(currentPosition);
}
