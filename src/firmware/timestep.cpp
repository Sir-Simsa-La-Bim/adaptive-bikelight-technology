#include "timestep.hpp"
#include <Arduino.h>

uint64_t currentTimestamp_us = 0;
uint32_t currentTimestep_us = 0;
uint16_t currentTimestep_ms = 0;

void nextTimestep()
{
    uint32_t lastTimestep_us = (uint32_t)currentTimestamp_us;
    currentTimestamp_us = micros();
    currentTimestep_us = (uint32_t)currentTimestamp_us - lastTimestep_us;
    currentTimestep_ms = currentTimestep_us / 1000;
}

TimeoutGuard::TimeoutGuard(uint16_t timeout_ms)
{
    this->startTime_ms = (uint16_t)millis();
    this->timeout_ms = timeout_ms;
}

bool TimeoutGuard::isExpired()
{
    uint16_t elapsedTime_ms = millis() - this->startTime_ms;
    return elapsedTime_ms >= timeout_ms;
}
