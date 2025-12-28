#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "util.hpp"

class DebouncedButton
{
private:
    static const uint32_t DEBOUCING_TIME_US = 20 * 1000;

    uint8_t pin;
    bool value;
    uint32_t steadyTime_us;

public:
    DebouncedButton(uint8_t pin, bool initialValue = false)
    {
        this->pin = pin;
        this->value = initialValue;
        this->steadyTime_us = 0;
    }

    uint32_t getPin()
    {
        return pin;
    }

    bool getValue()
    {
        return value;
    }

    uint32_t getSteadyTime_us()
    {
        return steadyTime_us;
    }

    void update();
};

struct ControlElementSettings
{
    uint32_t longPressThreshold_us;
    uint32_t bothButtonPressThreshold_us;
};

enum class ControlElementState : uint8_t
{
    Neutral,
    LeftPressedUnderThreshold,
    LeftPressedOverThreshold,
    RightPressedUnderThreshold,
    RightPressedOverThreshold,
    BothPressedUnderThreshold,
    WaitForNeutral,
};

enum class ControlElementActionType : uint8_t
{
    None = 0,
    ShortPress,
    LongPress,
    LongPressReleased,
    BothPressed,
};

struct ControlElementAction
{
    Direction direction;
    ControlElementActionType type;
};

class ControlElement
{
private:
    const ControlElementSettings &settings;
    DebouncedButton leftButton;
    DebouncedButton rightButton;
    ControlElementState state;

public:
    ControlElement(const ControlElementSettings &settings, uint8_t leftPin, uint8_t rightPin)
        : settings(settings), leftButton(leftPin), rightButton(rightPin)
    {
        state = ControlElementState::Neutral;
    }

    ControlElementAction update();
};