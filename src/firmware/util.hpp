#pragma once

#include <stdint.h>
#include "compiler_switches.h"

#define PI (3.1415926535897932384626433832795)
#define DEG_TO_RAD (PI / 180.0)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

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

union Float32Info
{
public:
    float value;
    uint32_t bits;

    Float32Info()
    {
        bits = 0;
    }

    Float32Info(float value)
    {
        this->value = value;
    }

    Float32Info(uint32_t bits)
    {
        this->value = value;
    }

    uint32_t getBits(uint8_t bitPos, uint8_t bitWidth) const
    {
        uint32_t mask = ((uint32_t)1u << bitWidth) - 1u;
        return (bits >> bitPos) & mask;
    }

    void setBits(uint8_t bitPos, uint8_t bitWidth, uint32_t value)
    {
        uint32_t mask = ((uint32_t)1u << bitWidth) - 1u;

        uint32_t shiftedValue = value << bitPos;
        uint32_t shiftedMask = mask << bitWidth;

        bits ^= (bits ^ shiftedValue) & shiftedMask;
    }

    bool getSign() const
    {
        return getBits(31, 1);
    }

    void setSign(bool value)
    {
        setBits(31, 1, value);
    }

    uint8_t getExponent() const
    {
        return getBits(23, 8);
    }

    void setExponent(uint8_t value)
    {
        setBits(23, 8, value);
    }

    uint32_t getMantissa() const
    {
        return getBits(0, 23);
    }

    void setMantissa(uint32_t value)
    {
        setBits(0, 23, value);
    }

    bool isZero() const
    {
        return (bits & 0x7FFFFFFFu) == 0;
    }
};

template <typename T>
inline constexpr int8_t sign(T value)
{
    return (T(0) < value) - (value < T(0));
}
