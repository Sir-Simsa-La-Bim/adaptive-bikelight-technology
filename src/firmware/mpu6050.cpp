#include "mpu6050.hpp"
#include <Arduino.h>

#undef USE_SOFT_I2C_MASTER_H_AS_PLAIN_INCLUDE
#define I2C_HARDWARE 1
#define I2C_PULLUP 1
#define I2C_TIMEOUT 10
#define I2C_NOINTERRUPT 0
#define I2C_FASTMODE 1
#include <SoftI2CMaster.h>

#include <hardwareSerial.h>

namespace MPU6050
{
    static const uint8_t whoamiCode = 0x68;

    static const float gyroRangeFactorsDeg[] = {
        250.0F / (1 << 15),
        500.0F / (1 << 15),
        1000.0F / (1 << 15),
        2000.0F / (1 << 15),
    };

    static const float gyroRangeFactorsRad[] = {
        250.0F / (1 << 15) * DEG_TO_RAD,
        500.0F / (1 << 15) * DEG_TO_RAD,
        1000.0F / (1 << 15) * DEG_TO_RAD,
        2000.0F / (1 << 15) * DEG_TO_RAD,
    };

    static const float accelRangeFactorsG[] = {
        2.0F / (1 << 15),
        4.0F / (1 << 15),
        8.0F / (1 << 15),
        16.0F / (1 << 15),
    };

    static const float g_si = 9.81;

    static const float accelRangeFactorsSi[] = {
        2.0F / (1 << 15) * g_si,
        4.0F / (1 << 15) * g_si,
        8.0F / (1 << 15) * g_si,
        16.0F / (1 << 15) * g_si,
    };

    const CalibrationSettins CALIBRATION_1kHz = {
        .filter = DigitalLowPassFilter::Accel184Hz_Gyro188Hz,
        .sampleRateDivider = 1,
        .settlingTime_ms = 6,
    };
    const CalibrationSettins CALIBRATION_100Hz = {
        .filter = DigitalLowPassFilter::Accel94Hz_Gyro98Hz,
        .sampleRateDivider = 10,
        .settlingTime_ms = 11,
    };
    const CalibrationSettins CALIBRATION_10Hz = {
        .filter = DigitalLowPassFilter::Accel10Hz_Gyro10Hz,
        .sampleRateDivider = 100,
        .settlingTime_ms = 100,
    };

    void Connection::begin()
    {
        i2c_init();
    }

    uint8_t Connection::readByte(Register reg)
    {
        uint8_t data;
        bool ack = i2c_start(shiftedAddress | I2C_WRITE);
        success &= ack;
        if (ack)
        {
            success &= i2c_write((uint8_t)reg);
            success &= i2c_rep_start(shiftedAddress | I2C_READ);
            data = i2c_read(true);
        }

        i2c_stop();
        return data;
    }

    void Connection::readBytes(Register reg, uint8_t *data, uint8_t length)
    {
        bool ack = i2c_start(shiftedAddress | I2C_WRITE);
        success &= ack;
        if (ack)
        {
            success &= i2c_write((uint8_t)reg);
            success &= i2c_rep_start(shiftedAddress | I2C_READ);

            while (length)
            {
                length--;
                *data = i2c_read(!length);
                data++;
            }
        }

        i2c_stop();
    }

    void Connection::writeByte(Register reg, uint8_t data)
    {
        bool ack = i2c_start(shiftedAddress | I2C_WRITE);
        success &= ack;
        if (ack)
        {
            success &= i2c_write((uint8_t)reg);
            success &= i2c_write(data);
        }

        i2c_stop();
    }

    void Connection::writeBytes(Register reg, const uint8_t *data, uint8_t length)
    {
        bool ack = i2c_start(shiftedAddress | I2C_WRITE);
        success &= ack;
        if (ack)
        {
            success &= i2c_write((uint8_t)reg);

            while (length)
            {
                success &= i2c_write(*data);
                data++;
                length--;
            }
        }

        i2c_stop();
    }

    float convertRawToTempDegreeCelcius(int16_t rawValue)
    {
        return rawValue / 340.0F + 36.53F;
    }

    Vec3<int16_t> RawSensorData::getAccelRawValue()
    {
        return Vec3<int16_t>(
            getInt16Value(&data[0]),
            getInt16Value(&data[2]),
            getInt16Value(&data[4]));
    }

    int16_t RawSensorData::getTempRawValue()
    {
        getInt16Value(&data[6]);
    }

    Vec3<int16_t> RawSensorData::getGyroRawValue()
    {
        return Vec3<int16_t>(
            getInt16Value(&data[8]),
            getInt16Value(&data[10]),
            getInt16Value(&data[12]));
    }

    void Driver::setupSilent(const DriverSetupConfig &config)
    {
        interface.reset();
        delay(100);

        interface.setPowerManagement1((PowerManagement1){
            .fields = {
                .clockSelect = ClockSelect::PllWithZGyroRef,
                .disableTempSensor = false,
                .enableCycleMode = false,
                .enableSleepMode = false,
                .deviceReset = false,
            },
        });

        setConfigurationSilent(config.filter);
        setSampleRateDividerSilent(config.sampleRateDivider);
        setGyroRangeSilent(config.gyroRange);
        setAccelRangeSilent(config.accelRange);

        interface.setInterruptBypassConfig((InterruptBypassConfig){
            .fields = {
                .enableI2cBypass = false,
                .enableFrameSyncInterrupt = false,
                .frameSyncLogicLevel = LogicLevel::ActiveHigh,
                .anyReadInterruptClear = false,
                .latchInterrupt = true,
                .openDrainInterrupt = false,
                .interruptLogicLevel = LogicLevel::ActiveHigh,
            },
        });
        interface.setInterruptEnable((InterruptEnable){
            .fields = {
                .dataReady = true,
                .i2cMaster = false,
                .fifoOverflow = true,
            },
        });
    }

    void Driver::setSampleRateDividerSilent(uint8_t divider)
    {
        interface.setSampleRateDivider(divider);
    }

    void Driver::setConfigurationSilent(DigitalLowPassFilter filter)
    {
        interface.setConfiguration((Configuration){
            .fields = {
                .lowPassFilter = filter,
                .extFrameSync = MPU6050::ExternalFrameSync::InputDisabled,
            },
        });
    }

    void Driver::setGyroRangeSilent(GyroFullScaleRange range)
    {
        interface.setGyroConfig((GyroConfig){
            .fields = {
                .fullScaleRange = range,
                .enableSelfTestZ = false,
                .enableSelfTestY = false,
                .enableSelfTestX = false,
            },
        });
    }

    void Driver::setAccelRangeSilent(AccelFullScaleRange range)
    {
        interface.setAccelConfig((AccelConfig){
            .fields = {
                .fullScaleRange = range,
                .enableSelfTestZ = false,
                .enableSelfTestY = false,
                .enableSelfTestX = false,
            },
        });
    }

    DriverError Driver::setup(const DriverSetupConfig &config)
    {
        interface.resetError();

        uint8_t code = interface.getWhoami();
        if (interface.hasError())
            return DriverError::ConnectionError;
        if (code != whoamiCode)
            return DriverError::UnknownDevice;

        setupSilent(config);

        if (interface.hasError())
            return DriverError::ConnectionError;

        this->config = config;

        resetGyroCalibration();
        resetAccelCalibration();

        return DriverError::Success;
    }

    DriverError Driver::setSampleRateDivider(uint8_t divider)
    {
        interface.resetError();

        setSampleRateDividerSilent(divider);

        if (interface.hasError())
            return DriverError::ConnectionError;

        config.sampleRateDivider = divider;

        return DriverError::Success;
    }

    DriverError Driver::setFilter(DigitalLowPassFilter filter)
    {
        interface.resetError();

        setConfigurationSilent(filter);

        if (interface.hasError())
            return DriverError::ConnectionError;

        config.filter = filter;

        return DriverError::Success;
    }

    DriverError Driver::setGyroRange(GyroFullScaleRange range)
    {
        interface.resetError();

        setGyroRangeSilent(range);

        if (interface.hasError())
            return DriverError::ConnectionError;

        config.gyroRange = range;
        resetGyroCalibration();

        return DriverError::Success;
    }

    DriverError Driver::setAccelRange(AccelFullScaleRange range)
    {
        interface.resetError();

        setAccelRangeSilent(range);

        if (interface.hasError())
            return DriverError::ConnectionError;

        config.accelRange = range;
        resetAccelCalibration();

        return DriverError::Success;
    }

    // ToDo: Take gravity in a useful format
    DriverError Driver::zeroCalibrate(const CalibrationSettins &settings, uint16_t sampleCount, Vec3<int16_t> gravity)
    {
        interface.resetError();

        setConfigurationSilent(settings.filter);
        setSampleRateDividerSilent(settings.sampleRateDivider);

        delay(settings.settlingTime_ms);

        if (!interface.hasError())
        {
            Vec3<int32_t> sumGyroBias = Vec3<int32_t>(0, 0, 0);
            Vec3<int32_t> sumAccelBias = Vec3<int32_t>(0, 0, 0);
            uint16_t remainingSampleCount = sampleCount;
            RawSensorData data;

            interface.getInterruptStatus();

            while (remainingSampleCount > 0)
            {
                InterruptStatus status = interface.getInterruptStatus();
                if (!status.fields.dataReady)
                    continue;

                interface.getSensorData(data);

                sumGyroBias -= (Vec3<int32_t>)data.getGyroRawValue();
                sumAccelBias -= (Vec3<int32_t>)data.getAccelRawValue();
                remainingSampleCount--;
            }

            if (!interface.hasError())
            {
                gyroBias = (Vec3<int16_t>)(sumGyroBias / sampleCount);
                accelBias = (Vec3<int16_t>)(sumAccelBias / sampleCount) + gravity;
            }
        }

        setConfigurationSilent(config.filter);
        setSampleRateDividerSilent(config.sampleRateDivider);

        if (interface.hasError())
            return DriverError::ConnectionError;

        return DriverError::Success;
    }

    float Driver::getGyroFactorInDeg()
    {
        return gyroRangeFactorsDeg[(uint8_t)config.gyroRange];
    }

    float Driver::getGyroFactorInRad()
    {
        return gyroRangeFactorsRad[(uint8_t)config.gyroRange];
    }

    float Driver::getAccelFactorInG()
    {
        return accelRangeFactorsG[(uint8_t)config.accelRange];
    }

    float Driver::getAccelFactorInSi()
    {
        return accelRangeFactorsSi[(uint8_t)config.accelRange];
    }

    DriverError Driver::readRawSensorData(RawSensorData &data)
    {
        interface.resetError();
        interface.getSensorData(data);
        if (interface.hasError())
            return DriverError::ConnectionError;
        return DriverError::Success;
    }

    DriverError Driver::readCalibratedSensorData(CalibratedSensorData &data)
    {
        RawSensorData rawData;
        DriverError error = readRawSensorData(rawData);

        if (error == DriverError::Success)
        {
            data.gyro = rawData.getGyroRawValue() + gyroBias;
            data.temp = rawData.getTempRawValue();
            data.accel = rawData.getAccelRawValue() + accelBias;
        }

        return error;
    }
}
