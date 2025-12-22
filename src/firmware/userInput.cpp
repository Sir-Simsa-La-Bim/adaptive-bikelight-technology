#include "userInput.hpp"
#include <Arduino.h>
#include "timestep.hpp"

static const int debouncingTime_us = 20 * 1000;

void DebouncedButton::update()
{
    steadyTime_us += currentTimestep_us;
    if (steadyTime_us < debouncingTime_us)
        return;

    bool lastValue = value;
    bool currValue = digitalRead(pin);

    if (lastValue != currValue)
        steadyTime_us = 0;

    value = currValue;
}

ControlElementAction ControlElement::update()
{
    leftButton.update();
    rightButton.update();

    bool leftPressed = leftButton.getValue();
    uint32_t leftTime_us = leftButton.getSteadyTime_us();

    bool rightPressed = rightButton.getValue();
    uint32_t rightTime_us = rightButton.getSteadyTime_us();

    ControlElementAction action = {
        .direction = Direction::Left,
        .type = ControlElementActionType::None,
    };

    switch (state)
    {
    case ControlElementState::Neutral:
        if (leftPressed)
        {
            if (rightPressed)
                state = ControlElementState::BothPressedUnderThreshold;
            else
                state = ControlElementState::LeftPressedUnderThreshold;
        }
        else
        {
            if (rightPressed)
                state = ControlElementState::RightPressedUnderThreshold;
        }
        break;

    case ControlElementState::LeftPressedUnderThreshold:
        if (leftPressed)
        {
            if (rightPressed)
                state = ControlElementState::BothPressedUnderThreshold;
            else if (leftTime_us >= settings.longPressThreshold_us)
            {
                action = (ControlElementAction){
                    .direction = Direction::Left,
                    .type = ControlElementActionType::LongPress,
                };
                state = ControlElementState::LeftPressedOverThreshold;
            }
        }
        else
        {
            action = (ControlElementAction){
                .direction = Direction::Left,
                .type = ControlElementActionType::ShortPress,
            };
            state = ControlElementState::WaitForNeutral;
        }
        break;

    case ControlElementState::LeftPressedOverThreshold:
        if (!leftPressed)
        {
            action = (ControlElementAction){
                .direction = Direction::Left,
                .type = ControlElementActionType::LongPressReleased,
            };
            state = ControlElementState::WaitForNeutral;
        }
        break;

    case ControlElementState::RightPressedUnderThreshold:
        if (rightPressed)
        {
            if (leftPressed)
                state = ControlElementState::BothPressedUnderThreshold;
            else if (rightTime_us >= settings.longPressThreshold_us)
            {
                action = (ControlElementAction){
                    .direction = Direction::Right,
                    .type = ControlElementActionType::LongPress,
                };
                state = ControlElementState::RightPressedOverThreshold;
            }
        }
        else
        {
            action = (ControlElementAction){
                .direction = Direction::Right,
                .type = ControlElementActionType::ShortPress,
            };
            state = ControlElementState::WaitForNeutral;
        }
        break;

    case ControlElementState::RightPressedOverThreshold:
        if (!rightPressed)
        {
            action = (ControlElementAction){
                .direction = Direction::Right,
                .type = ControlElementActionType::LongPressReleased,
            };
            state = ControlElementState::WaitForNeutral;
        }
        break;

    case ControlElementState::BothPressedUnderThreshold:
        if (!leftPressed || !rightPressed)
            state = ControlElementState::WaitForNeutral;
        else if (leftTime_us >= settings.bothButtonPressThreshold_us && rightTime_us >= settings.bothButtonPressThreshold_us)
        {
            action = (ControlElementAction){
                .direction = Direction::Left,
                .type = ControlElementActionType::BothPressed,
            };
            state = ControlElementState::WaitForNeutral;
        }
        break;

    case ControlElementState::WaitForNeutral:
    default:
        if (!leftPressed && !rightPressed)
            state = ControlElementState::Neutral;
        break;
    }

    return action;
}
