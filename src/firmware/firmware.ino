#include "servoDriver.hpp"
#include <stdint.h>

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

uint32_t lastTimestamp_ms = 0;

void setup()
{
    initServo(SERVO_PIN);
}

void loop()
{
    uint32_t currTimestamp_ms = (uint32_t)millis();
    uint32_t timestep_ms = currTimestamp_ms - lastTimestamp_ms;
    lastTimestamp_ms = currTimestamp_ms;

    float position, p2;
    if (digitalRead(12)) {
        position = 1.5707963267948966192313216916398;
    } else {
        position = 0;
    }

    updateServo(position, timestep_ms);
}
