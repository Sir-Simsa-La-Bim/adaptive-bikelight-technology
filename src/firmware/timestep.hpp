#pragma once
#include <stdint.h>

extern uint64_t currentTimestamp_us;
extern uint32_t currentTimestep_us;

void nextTimestep();
