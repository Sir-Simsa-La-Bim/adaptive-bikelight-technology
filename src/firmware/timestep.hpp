#pragma once

#include <stdint.h>

class TimeoutGuard
{
private:
    uint16_t startTime_ms;
    uint16_t timeout_ms;

public:
    TimeoutGuard(uint16_t timeout_ms);
    bool isExpired();
};

extern uint64_t currentTimestamp_us;
extern uint32_t currentTimestep_us;
extern uint16_t currentTimestep_ms;

void nextTimestep();
