#include <stdint.h>
#include "servoDriver.hpp"
#include "timestep.hpp"

/////////// TODO ////////////
#define SERVO_PIN 9

enum class State : uint8_t
{
    AutoComputed,
    AutoHint,
    AutoFixed,
    ManualCostum,
    ManualFixed,
};

State state = State::AutoComputed;
int8_t manualSelection = 0;

void setup()
{
    initServo(SERVO_PIN);
}

void loop()
{
    nextTimestep();

    float position;
    if (digitalRead(12))
    {
        position = 1.5707963267948966192313216916398;
    }
    else
    {
        position = 0;
    }

    updateServo(position);
}
