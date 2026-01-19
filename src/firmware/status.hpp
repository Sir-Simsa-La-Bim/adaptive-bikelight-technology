#pragma once

#include <stdint.h>
#include "compiler_switches.h"

struct StatusLedSettings
{
    uint8_t ledPin;
    uint16_t errorHoldTime_ms;
    uint16_t tempErrorBlinkPeriod_ms;
    uint16_t fatalErrorBlinkPeriod_ms;
};

class StatusLed
{
private:
    const StatusLedSettings &settings;

    uint16_t errorHoldTime_ms;
    uint16_t blinkTimer_ms;
    bool active;

public:
    StatusLed(const StatusLedSettings &settings)
        : settings(settings)
    {
        errorHoldTime_ms = 0;
        blinkTimer_ms = 0;
        active = false;
    }

    void begin();
    void update();
    void setActive(bool value);
    void notifyError();
    void fatalError();
};