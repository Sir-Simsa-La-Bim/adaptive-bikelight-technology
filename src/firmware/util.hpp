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

template <class T>
class numeric_limits
{
public:
    static constexpr T minimum()
    {
        return T();
    }

    static constexpr T maximum()
    {
        return T();
    }
};

template <>
class numeric_limits<uint8_t>
{
public:
    static constexpr uint8_t minimum()
    {
        return 0;
    }

    static constexpr uint8_t maximum()
    {
        return 0xFF;
    }
};

template <>
class numeric_limits<int8_t>
{
public:
    static constexpr int8_t minimum()
    {
        return -0x80;
    }

    static constexpr int8_t maximum()
    {
        return 0x7F;
    }
};

template <>
class numeric_limits<uint16_t>
{
public:
    static constexpr uint16_t minimum()
    {
        return 0;
    }

    static constexpr uint16_t maximum()
    {
        return 0xFFFF;
    }
};

template <>
class numeric_limits<int16_t>
{
public:
    static constexpr int16_t minimum()
    {
        return -0x8000;
    }

    static constexpr int16_t maximum()
    {
        return 0x7FFF;
    }
};

template <>
class numeric_limits<uint32_t>
{
public:
    static constexpr uint32_t minimum()
    {
        return 0;
    }

    static constexpr uint32_t maximum()
    {
        return 0xFFFF;
    }
};

template <>
class numeric_limits<int32_t>
{
public:
    static constexpr int32_t minimum()
    {
        return -0x80000000;
    }

    static constexpr int32_t maximum()
    {
        return 0x7FFFFFFF;
    }
};

template <>
class numeric_limits<uint64_t>
{
public:
    static constexpr uint64_t minimum()
    {
        return 0;
    }

    static constexpr uint64_t maximum()
    {
        return 0xFFFFFFFFFFFFFFFF;
    }
};

template <>
class numeric_limits<int64_t>
{
public:
    static constexpr int64_t minimum()
    {
        return -0x8000000000000000;
    }

    static constexpr int64_t maximum()
    {
        return 0x7FFFFFFFFFFFFFFF;
    }
};

template <typename T>
inline constexpr int sign(T value)
{
    return (T(0) < value) - (value < T(0));
}