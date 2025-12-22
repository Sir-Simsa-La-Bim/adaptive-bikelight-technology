#include <stdint.h>
#include "servoDriver.hpp"
#include "timestep.hpp"
#include "userInput.hpp"

#define PIN_SERVO 9
#define PIN_BUTTON_LEFT 12
#define PIN_BUTTON_RIGHT 11

enum class State : uint8_t
{
    AutoComputed,
    AutoHint,
    AutoFixed,
    ManualCostum,
    ManualFixed,
};

static State state = State::AutoComputed;
static int8_t manualSelection = 0;

static const ServoDriverSettings settings(2000.0F / 270.0F, 90.0F);
static ServoDriver servoDriver(settings, 1500);

static const ControlElementSettings controlElementSettings = {
    .longPressThreshold_us = 500000,
    .bothButtonPressThreshold_us = 1000000,
};
static ControlElement controlElement(controlElementSettings, PIN_BUTTON_LEFT, PIN_BUTTON_RIGHT);

static float computedDirection = 0;
static float manualDirection = 0;

static void input();
static void evaluate();
static void output();

void setup()
{
    servoDriver.begin(PIN_SERVO);
}

void loop()
{
    nextTimestep();
    input();
    evaluate();
    output();
}

static void input()
{
    ControlElementAction action = controlElement.update();
    if (action.type != ControlElementActionType::None)
    {
        const char *text = "";
        switch (action.type)
        {
        case ControlElementActionType::ShortPress:
            text = "short press";
            break;
        case ControlElementActionType::LongPress:
            text = "long press";
            break;
        case ControlElementActionType::LongPressReleased:
            text = "long press released";
            break;
        case ControlElementActionType::BothPressed:
            text = "both";
            break;
        }

        Serial.print(text);
        Serial.print(" ");

        switch (action.direction)
        {
        case Direction::Left:
            text = "left";
            break;
        case Direction::Right:
            text = "right";
            break;
            ;
        }

        Serial.print(text);
        Serial.println();
    }
}

static void evaluate()
{
    // ToDo: update computed direction

    // switch (state)
    // {
    // case State::AutoComputed:
    //     break;
    // case State::AutoHint:
    //     break;
    // case State::AutoFixed:
    //     break;
    // case State::ManualCostum:
    //     break;
    // case State::ManualFixed:
    //     break;
    // }
}

static void output()
{
    float servoPosition =
        state == State::AutoComputed
            ? computedDirection
            : manualDirection;
    servoDriver.update(servoPosition);
}