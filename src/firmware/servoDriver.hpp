#pragma once
#include <stdint.h>
#include <Servo.h>
#include "compiler_switches.h"
#include "util.hpp"

class ServoDriverSettings
{
public:
    uint16_t microsecPerStep;
    float stepsPerRadiant;

    ServoDriverSettings(float stepsPerRadiant, uint16_t microsecPerStep)
    {
        this->microsecPerStep = microsecPerStep;
        this->stepsPerRadiant = stepsPerRadiant;
    }

    ServoDriverSettings(float stepsPerDegree, float degreePerSecondLimit)
    {
        this->microsecPerStep = (uint16_t)(1e6 / (degreePerSecondLimit * stepsPerDegree) + 0.5F);
        this->stepsPerRadiant = stepsPerDegree / DEG_TO_RAD;
    }
};

class ServoDriver
{
private:
    const ServoDriverSettings &settings;
    uint16_t neutralPosition;
    uint16_t currentPosition;
    uint16_t accumulatedTime_us;
    Servo servo;

public:
    ServoDriver(const ServoDriverSettings &settings, uint16_t neturalPosition)
        : settings(settings)
    {
        this->neutralPosition = neturalPosition;
        this->currentPosition = neturalPosition;
        this->accumulatedTime_us = 0;
    }

    void begin(uint8_t pin);
    void update(float position_rad);
};
