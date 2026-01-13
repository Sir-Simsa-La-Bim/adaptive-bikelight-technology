#include <stdint.h>
#include <math.h>
#include "servoDriver.hpp"
#include "timestep.hpp"
#include "userInput.hpp"
#include "status.hpp"
#include "mpu6050.hpp"
#include "circularMotion.hpp"

#define PIN_SERVO 9
#define PIN_BUTTON_LEFT 12
#define PIN_BUTTON_RIGHT 11
#define PIN_LED 13

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

// ToDo: add good values
static const float lightDistance_m = 5.0;

static const MPU6050::DriverSetupConfig imuSetup = {
    .sampleRateDivider = 50,
    .filter = MPU6050::DigitalLowPassFilter::Accel5Hz_Gyro5Hz,
    .gyroRange = MPU6050::GyroFullScaleRange::_250_degPerSec,
    .accelRange = MPU6050::AccelFullScaleRange::_2g,
};
static const ImuRangeFactors imuRangeFactors = imuSetup.getRangeFactors();
static const uint8_t driverSetupTrials = 10;
static MPU6050::DirectDriver imuDriver(0x68);

static const uint16_t calibrationSampleRateDivider = 5;
static const uint16_t calibrationTimeout_ms = 3000;
static const uint16_t calibrationMaxSampleCount = 500;
static const uint16_t calibrationMinSampleCount = 300;

static int16_t gyroXFilterBuffer[10];
static int16_t gyroYFilterBuffer[10];
static EncodedRadius radiusMedianFilterBuffer[5];
static EncodedRadius radiusMovAvgFilterBuffer[10];
static CircularMotionProcessor motionProcessor(
    RawValueFilter(gyroXFilterBuffer, ARRAY_SIZE(gyroXFilterBuffer)),
    RawValueFilter(gyroYFilterBuffer, ARRAY_SIZE(gyroYFilterBuffer)),
    RadiusFilter(radiusMedianFilterBuffer, ARRAY_SIZE(radiusMedianFilterBuffer), radiusMovAvgFilterBuffer, ARRAY_SIZE(radiusMovAvgFilterBuffer)));

static const ServoDriverSettings servoSettings(2000.0F / 270.0F, 90.0F);
static ServoDriver servoDriver(servoSettings, 1500);

static const ControlElementSettings controlElementSettings = {
    .longPressThreshold_us = 500000,
    .bothButtonPressThreshold_us = 1000000,
};
static ControlElement controlElement(controlElementSettings, PIN_BUTTON_LEFT, PIN_BUTTON_RIGHT);

static const StatusLedSettings statusLedSettings = {
    .ledPin = PIN_LED,
    .errorHoldTime_ms = 1000,
    .tempErrorBlinkPeriod_ms = 500,
    .fatalErrorBlinkPeriod_ms = 100,
};
static StatusLed statusLed(statusLedSettings);

static const float automaticHintStep = 20.0 * DEG_TO_RAD;
static const float automaticFixedStep = 20.0 * DEG_TO_RAD;
static const float manualClickStep = 10.0 * DEG_TO_RAD;
static const float manualFixedStep = 20.0 * DEG_TO_RAD;
static const int8_t manualStepLimit = 5;

static OperatingMode operatingMode = OperatingMode::Automatic;
static OperatingSubmode operatingSubmode = OperatingSubmode::Main;
static Direction operatingDirection = Direction::None;
static int8_t manualSteps = 0;
static float automaticDirection = 0;

static void imuApplySetup();
static void imuCalibrationSequence();
static void switchOperatingMode();
static void updateCurveData();
static float evaluateAutomaticMode(ControlElementAction action);
static float evaluateManualMode(ControlElementAction action);

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    statusLed.begin();
    servoDriver.begin(PIN_SERVO);
    imuDriver.begin();

    imuApplySetup();
    imuCalibrationSequence();

    statusLed.setActive(true);
}

void loop()
{
    nextTimestep();
    updateCurveData();

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
    statusLed.update();
}

static void imuApplySetup()
{
    for (uint8_t i = 0; i < driverSetupTrials; i++)
    {
        if (imuDriver.setup(imuSetup) == MPU6050::DriverError::Success)
            return;

        delay(100);
    }

    statusLed.fatalError();
}

static void imuCalibrationSequence()
{
    MPU6050::Interface &imuInterface = imuDriver.getInterface();
    imuInterface.setSampleRateDivider(calibrationSampleRateDivider);
    imuInterface.resetError();

    TimeoutGuard timeout(calibrationTimeout_ms);
    CircularMotionCalibrationBuilder calibrationBuilder;
    do
    {
        MPU6050::DriverError driverError;
        bool hasDataAvailable;
        driverError = imuDriver.available(hasDataAvailable);
        if (driverError != MPU6050::DriverError::Success || !hasDataAvailable)
            continue;

        ImuData imuData;
        driverError = imuDriver.read(imuData);
        if (driverError != MPU6050::DriverError::Success)
            continue;

        calibrationBuilder.addSample(imuData);
    } while (!timeout.isExpired() && calibrationBuilder.getSampleCount() < calibrationMaxSampleCount);

    imuInterface.setSampleRateDivider(imuSetup.sampleRateDivider);
    imuInterface.resetError();

    if (calibrationBuilder.getSampleCount() < calibrationMinSampleCount)
        statusLed.fatalError();

    motionProcessor.setCalibration(calibrationBuilder);
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

static void updateCurveData()
{
    bool hasDataAvailable;
    MPU6050::DriverError driverError = imuDriver.available(hasDataAvailable);
    if (driverError != MPU6050::DriverError::Success)
    {
        statusLed.notifyError();
        return;
    }

    if (!hasDataAvailable)
        return;

    ImuData imuData;
    driverError = imuDriver.read(imuData);
    if (driverError != MPU6050::DriverError::Success)
    {
        statusLed.notifyError();
        return;
    }

    CircularMotionData motionData;
    motionProcessor.update(imuData, imuRangeFactors, motionData);

    float sinAngle = (0.5 * lightDistance_m) * motionData.invRadius;
    float angle;
    if (fabsf(sinAngle) <= 1)
        angle = asin(angle);
    else
        angle = sign(sinAngle) * (PI * 0.5F);

    automaticDirection = angle;
}

static float evaluateAutomaticMode(ControlElementAction action)
{
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
            abs(automaticDirection) >= automaticHintStep)
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
        return automaticDirection;

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
