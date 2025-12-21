#include "timestep.hpp"
#include <Arduino.h>

uint64_t currentTimestamp_us = 0;
uint32_t currentTimestep_us = 0;

void nextTimestep()
{
    uint32_t lastTimestep_us = (uint32_t)currentTimestamp_us;
    currentTimestamp_us = micros();
    currentTimestep_us = (uint32_t)currentTimestamp_us - lastTimestep_us;
}
