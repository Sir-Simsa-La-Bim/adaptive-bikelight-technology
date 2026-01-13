#include "status.hpp"
#include <Arduino.h>
#include "timestep.hpp"

void StatusLed::begin()
{
    pinMode(settings.ledPin, OUTPUT);
}

void StatusLed::update()
{
    if (errorHoldTime_ms <= 0)
        return;

    bool ledOn;
    if (currentTimestep_ms > errorHoldTime_ms)
    {
        errorHoldTime_ms -= currentTimestep_ms;

        uint16_t oldBlinkTimer_ms = blinkTimer_ms;
        blinkTimer_ms -= currentTimestep_ms;
        if (blinkTimer_ms < currentTimestep_ms)
            blinkTimer_ms += settings.tempErrorBlinkPeriod_ms;

        ledOn = blinkTimer_ms >= (settings.tempErrorBlinkPeriod_ms >> 1);
    }
    else
    {
        errorHoldTime_ms = 0;
        blinkTimer_ms = 0;

        ledOn = active;
    }

    digitalWrite(settings.ledPin, ledOn);
}

void StatusLed::setActive(bool value)
{
    if (active != value)
    {
        active = value;
        if (errorHoldTime_ms <= 0)
            digitalWrite(settings.ledPin, value);
    }
}

void StatusLed::notifyError()
{
    errorHoldTime_ms = settings.errorHoldTime_ms;
}

[[noreturn]]
void StatusLed::fatalError()
{
    uint16_t offTime_ms = settings.fatalErrorBlinkPeriod_ms >> 1;
    uint16_t onTime_ms = settings.fatalErrorBlinkPeriod_ms - onTime_ms;

    while (1)
    {
        digitalWrite(settings.ledPin, 1);
        delay(onTime_ms);
        digitalWrite(settings.ledPin, 0);
        delay(offTime_ms);
    };
}
