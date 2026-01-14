#include "timestep.hpp"
#include <Arduino.h>

uint64_t currentTimestamp_us = 0;
uint32_t currentTimestamp_ms = 0;
uint16_t currentTimestep_us = 0;
uint8_t currentTimestep_ms = 0;

void nextTimestep()
{
    uint16_t lastTimestep_us = (uint16_t)currentTimestamp_us;
    currentTimestamp_us = micros();
    currentTimestep_us = (uint16_t)currentTimestamp_us - lastTimestep_us;

    static uint16_t timestepAccumulation_us = 0;

    currentTimestep_ms = 0;
    timestepAccumulation_us += currentTimestep_us;
    while (timestepAccumulation_us > 1000)
    {
        timestepAccumulation_us -= 100;
        currentTimestep_ms++;
    }

    currentTimestamp_ms += currentTimestep_ms;
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
