#include "servoDriver.hpp"
#include "timestep.hpp"

void ServoDriver::begin(uint8_t pin)
{
    servo.attach(pin);
    servo.writeMicroseconds(currentPosition);
}

void ServoDriver::update(float position_rad)
{
    uint16_t desiredPosition = (uint16_t)(position_rad * settings.stepsPerRadiant + 0.5) + neutralPosition;
    accumulatedTime_us += currentTimestep_us;
    uint16_t maxStep_dg = accumulatedTime_us / settings.microsecPerStep;
    accumulatedTime_us -= maxStep_dg * settings.microsecPerStep;
    int16_t difference_dg = desiredPosition - currentPosition;

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
