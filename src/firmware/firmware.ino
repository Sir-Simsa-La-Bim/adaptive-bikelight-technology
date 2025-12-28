#include <stdint.h>
#include <math.h>
#include "servoDriver.hpp"
#include "timestep.hpp"
#include "userInput.hpp"
#include "util.hpp"
#include "mpu6050.hpp"
#include "vec.hpp"

#define PIN_SERVO 9
#define PIN_BUTTON_LEFT 12
#define PIN_BUTTON_RIGHT 11

enum class OperatingMode
{
    Automatic,
    Manual
};

enum class OperatingSubmode
{
    Main,
    Hint,
    Fixed
};

static const MPU6050::DriverSetupConfig driverConfig = {
    .sampleRateDivider = 4,
    .filter = MPU6050::DigitalLowPassFilter::Accel5Hz_Gyro5Hz,
    .gyroRange = MPU6050::GyroFullScaleRange::_250_degPerSec,
    .accelRange = MPU6050::AccelFullScaleRange::_2g,
};

// ToDo: add good values
static const float minAngularSpeed_1PerSec = 1e-3;
static const float minCentripetalAccel_mPerSec2 = 1e-6;
static const float lightDistance_m = 5.0;

static MPU6050::FifoDriver imuDriver(0x68);
static float centripetalAccel_mPerSec2;
static float angularSpeed_1PerSec;

static const ServoDriverSettings settings(2000.0F / 270.0F, 90.0F);
static ServoDriver servoDriver(settings, 1500);

static const ControlElementSettings controlElementSettings = {
    .longPressThreshold_us = 500000,
    .bothButtonPressThreshold_us = 1000000,
};
static ControlElement controlElement(controlElementSettings, PIN_BUTTON_LEFT, PIN_BUTTON_RIGHT);

static const float automaticHintStep = 20.0 * DEG_TO_RAD;
static const float automaticFixedStep = 20.0 * DEG_TO_RAD;
static const float manualClickStep = 10.0 * DEG_TO_RAD;
static const float manualFixedStep = 20.0 * DEG_TO_RAD;
static const int8_t manualStepLimit = 5;

static OperatingMode operatingMode = OperatingMode::Automatic;
static OperatingSubmode operatingSubmode = OperatingSubmode::Main;
static Direction operatingDirection = Direction::None;
static int8_t manualSteps = 0;

static void updateImu();
static float evaluateAutomaticMode(ControlElementAction action);
static float evaluateManualMode(ControlElementAction action);
static float calcAutomaticDirection();

void setup()
{
    Serial.begin(115200);

    imuDriver.begin();
    servoDriver.begin(PIN_SERVO);

    MPU6050::DriverError imuError;

    // ToDo: Error handling
    imuError = imuDriver.setup(driverConfig);

    while (!Serial)
        ;
}

void loop()
{
    nextTimestep();

    updateImu();

    ControlElementAction action = controlElement.update();
    if (action.type == ControlElementActionType::BothPressed)
        switchOperatingMode();

    float direction;
    switch (operatingMode)
    {
    case OperatingMode::Automatic:
        direction = evaluateAutomaticMode(action);
        break;

    case OperatingMode::Manual:
    default:
        direction = evaluateManualMode(action);
        break;
    }

    servoDriver.update(direction);
}

static void updateImu()
{
    MPU6050::SensorData data;
    int16_t rawZentripetalAccel, rawAngularSpeed;

    int8_t sampleCount = 0;
    MPU6050::FifoDriverReader reader = imuDriver.read();

    while (reader.getNext(data))
    {
        // ToDo: Filtering of raw values
        rawZentripetalAccel = data.accel.y();
        rawAngularSpeed = data.gyro.x();

        sampleCount++;
    }

    // ToDo: Error handling
    MPU6050::DriverError error = reader.getError();

    if (sampleCount > 0)
    {
        centripetalAccel_mPerSec2 = rawZentripetalAccel * driverConfig.getAccelFactorInSi();
        angularSpeed_1PerSec = rawAngularSpeed * driverConfig.getGyroFactorInRad();

        Serial.print("samples ");
        Serial.print(sampleCount);
        Serial.print(" accel ");
        Serial.print(centripetalAccel_mPerSec2);
        Serial.print(" ang speed ");
        Serial.print(angularSpeed_1PerSec);
        Serial.println();
    }
}

static void switchOperatingMode()
{
    switch (operatingMode)
    {
    case OperatingMode::Automatic:
        operatingMode = OperatingMode::Manual;
        break;

    case OperatingMode::Manual:
    default:
        operatingMode = OperatingMode::Automatic;
        break;
    }

    operatingSubmode = OperatingSubmode::Main;
    operatingDirection = Direction::None;
    manualSteps = 0;
}

static float evaluateAutomaticMode(ControlElementAction action)
{
    float calculatedDirection = calcAutomaticDirection();

    switch (operatingSubmode)
    {
    case OperatingSubmode::Main:
        if (action.type == ControlElementActionType::ShortPress)
        {
            operatingSubmode = OperatingSubmode::Hint;
            operatingDirection = action.direction;
        }
        else if (action.type == ControlElementActionType::LongPress)
        {
            operatingSubmode = OperatingSubmode::Fixed;
            operatingDirection = action.direction;
        }
        break;

    case OperatingSubmode::Hint:
        if (action.type == ControlElementActionType::LongPress)
        {
            operatingSubmode = OperatingSubmode::Fixed;
            operatingDirection = action.direction;
        }
        else if (
            (action.type == ControlElementActionType::ShortPress && action.direction != operatingDirection) ||
            abs(calculatedDirection) >= automaticHintStep)
        {
            operatingSubmode = OperatingSubmode::Main;
            operatingDirection = Direction::None;
        }
        break;

    case OperatingSubmode::Fixed:
    default:
        if (action.type == ControlElementActionType::LongPressReleased)
        {
            operatingSubmode = OperatingSubmode::Main;
            operatingDirection = Direction::None;
        }
        break;
    }

    switch (operatingSubmode)
    {
    case OperatingSubmode::Main:
        return calculatedDirection;

    case OperatingSubmode::Hint:
        return (int8_t)operatingDirection * automaticHintStep;

    case OperatingSubmode::Fixed:
    default:
        return (int8_t)operatingDirection * automaticFixedStep;
    }
}

static float evaluateManualMode(ControlElementAction action)
{
    switch (operatingSubmode)
    {
    case OperatingSubmode::Main:
        if (action.type == ControlElementActionType::ShortPress)
        {
            manualSteps += (int8_t)action.direction;
            if (manualSteps > manualStepLimit)
                manualSteps = manualStepLimit;
            else if (manualSteps < -manualStepLimit)
                manualSteps = -manualStepLimit;
        }
        if (action.type == ControlElementActionType::LongPress)
        {
            operatingSubmode = OperatingSubmode::Fixed;
            operatingDirection = action.direction;
        }
        break;

    case OperatingSubmode::Fixed:
    default:
        if (action.type == ControlElementActionType::LongPressReleased)
        {
            operatingSubmode = OperatingSubmode::Main;
            operatingDirection = Direction::None;
            manualSteps = 0;
        }
        break;
    }

    switch (operatingSubmode)
    {
    case OperatingSubmode::Main:
        return manualSteps * manualClickStep;

    case OperatingSubmode::Fixed:
    default:
        return (int8_t)operatingDirection * manualFixedStep;
    }
}

static float calcAutomaticDirection()
{
    if (abs(angularSpeed_1PerSec) < minAngularSpeed_1PerSec || abs(centripetalAccel_mPerSec2) < minCentripetalAccel_mPerSec2)
        return 0;

    float lightAngle = asin(0.5 * lightDistance_m * angularSpeed_1PerSec * angularSpeed_1PerSec / centripetalAccel_mPerSec2);
    return lightAngle;
}