#pragma once

#include <stdint.h>

#define PI (3.1415926535897932384626433832795)
#define DEG_TO_RAD (PI / 180.0)

enum class Direction : int8_t
{
    Left = -1,
    None = 0,
    Right = 1,
};
