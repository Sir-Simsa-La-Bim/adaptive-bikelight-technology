#pragma once

#include <stdint.h>
#include "compiler_switches.h"

union ImuData
{
    struct
    {
        int16_t gyroX;
        int16_t gyroY;
        int16_t gyroZ;
        int16_t accelX;
        int16_t accelY;
        int16_t accelZ;
    } fields;
    int16_t array[sizeof(fields) / sizeof(int16_t)];
};

struct ImuRangeFactors
{
    float gyroFactor_rad;
    float accelFactor_si;
};

extern const float gravityAccel_si;