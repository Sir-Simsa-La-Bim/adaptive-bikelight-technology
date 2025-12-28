#include "mpu6050.hpp"
#include <Arduino.h>

#undef USE_SOFT_I2C_MASTER_H_AS_PLAIN_INCLUDE
#define I2C_HARDWARE 1
#define I2C_PULLUP 1
#define I2C_TIMEOUT 10
#define I2C_NOINTERRUPT 0
#define I2C_FASTMODE 1
#include <SoftI2CMaster.h>

namespace MPU6050
{
    static const uint8_t whoamiCode = 0x68;

    const float gyroRangeFactorsDeg[] = {
        250.0F / (1 << 15),
        500.0F / (1 << 15),
        1000.0F / (1 << 15),
        2000.0F / (1 << 15),
    };

    const float gyroRangeFactorsRad[] = {
        250.0F / (1 << 15) * DEG_TO_RAD,
        500.0F / (1 << 15) * DEG_TO_RAD,
        1000.0F / (1 << 15) * DEG_TO_RAD,
        2000.0F / (1 << 15) * DEG_TO_RAD,
    };

    const float accelRangeFactorsG[] = {
        2.0F / (1 << 15),
        4.0F / (1 << 15),
        8.0F / (1 << 15),
        16.0F / (1 << 15),
    };

    static const float g_si = 9.81;

    const float accelRangeFactorsSi[] = {
        2.0F / (1 << 15) * g_si,
        4.0F / (1 << 15) * g_si,
        8.0F / (1 << 15) * g_si,
        16.0F / (1 << 15) * g_si,
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

    void SensorData::decode(const SensorDataRegisterContent &rawData)
    {
        accel = Vec3<int16_t>(
            getInt16Value(&rawData.bytes[0]),
            getInt16Value(&rawData.bytes[2]),
            getInt16Value(&rawData.bytes[4]));

        gyro = Vec3<int16_t>(
            getInt16Value(&rawData.bytes[8]),
            getInt16Value(&rawData.bytes[10]),
            getInt16Value(&rawData.bytes[12]));
    }

    void SensorData::decode(const FifoSensorData &rawData)
    {
        accel = Vec3<int16_t>(
            getInt16Value(&rawData.bytes[0]),
            getInt16Value(&rawData.bytes[2]),
            getInt16Value(&rawData.bytes[4]));

        gyro = Vec3<int16_t>(
            getInt16Value(&rawData.bytes[6]),
            getInt16Value(&rawData.bytes[8]),
            getInt16Value(&rawData.bytes[10]));
    }

    DriverError Driver::basicSetup(const DriverSetupConfig &config)
    {
        interface.resetError();

        uint8_t code = interface.getWhoami();
        if (interface.hasError())
            return DriverError::ConnectionError;
        if (code != whoamiCode)
            return DriverError::UnknownDevice;

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

        interface.setConfiguration((Configuration){
            .fields = {
                .lowPassFilter = config.filter,
                .extFrameSync = MPU6050::ExternalFrameSync::InputDisabled,
            },
        });

        interface.setSampleRateDivider(config.sampleRateDivider);

        interface.setGyroConfig((GyroConfig){
            .fields = {
                .fullScaleRange = config.gyroRange,
                .enableSelfTestZ = false,
                .enableSelfTestY = false,
                .enableSelfTestX = false,
            },
        });

        interface.setAccelConfig((AccelConfig){
            .fields = {
                .fullScaleRange = config.accelRange,
                .enableSelfTestZ = false,
                .enableSelfTestY = false,
                .enableSelfTestX = false,
            },
        });

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

        if (interface.hasError())
            return DriverError::ConnectionError;
        return DriverError::Success;
    }

    DriverError DirectDriver::setup(const DriverSetupConfig &config)
    {
        return basicSetup(config);
    }

    DriverError DirectDriver::available(bool &result)
    {
        interface.resetError();

        InterruptStatus status = interface.getInterruptStatus();

        if (interface.hasError())
            return DriverError::ConnectionError;

        result = status.fields.dataReady;

        return DriverError::Success;
    }

    DriverError DirectDriver::read(SensorData &data)
    {
        interface.resetError();

        SensorDataRegisterContent rawData;
        interface.getSensorData(rawData);

        if (interface.hasError())
            return DriverError::ConnectionError;

        data.decode(rawData);

        return DriverError::Success;
    }

    DriverError FifoDriver::setup(const DriverSetupConfig &config)
    {
        DriverError error = basicSetup(config);
        if (error != DriverError::Success)
            return error;

        interface.setFifoEnable((FifoEnable){
            .fields = {
                .external0 = false,
                .external1 = false,
                .external2 = false,
                .accelXYZ = true,
                .gyroZ = true,
                .gyroY = true,
                .gyroX = true,
                .temp = false,
            },
        });

        interface.setUserControl((UserControl){
            .fields = {
                .resetAllSignalPaths = false,
                .resetI2cMaster = false,
                .resetFifo = true,
                .enableI2cMasterMode = false,
                .fifoEnable = true,
            },
        });

        return DriverError::Success;
    }

    bool FifoDriverReader::popNextFromFifo(SensorData &data)
    {
        interface.popFifo(dataBuffer.bytes, sizeof(dataBuffer.bytes));
        availableDataCount -= sizeof(dataBuffer.bytes);

        if (interface.hasError())
        {
            error |= DriverError::ConnectionError;
            return false;
        }

        data.decode(dataBuffer);
        return true;
    }

    bool FifoDriverReader::getNext(SensorData &data)
    {
        interface.resetError();

        if (availableDataCount >= sizeof(dataBuffer.bytes))
            return popNextFromFifo(data);

        uint16_t newFifoCount = interface.getFifoCount();
        if (interface.hasError())
        {
            error |= DriverError::ConnectionError;
            return false;
        }

        availableDataCount = newFifoCount;
        if (newFifoCount < sizeof(dataBuffer.bytes))
            return false;

        bool result = popNextFromFifo(data);

        interface.resetError();

        InterruptStatus status = interface.getInterruptStatus();
        if (interface.hasError())
        {
            error |= DriverError::ConnectionError;
            return false;
        }
        if (status.fields.fifoOverflow)
        {
            error |= DriverError::FifoOverflow;

            interface.setUserControl((UserControl){
                .fields = {
                    .resetAllSignalPaths = false,
                    .resetI2cMaster = false,
                    .resetFifo = true,
                    .enableI2cMasterMode = false,
                    .fifoEnable = true,
                },
            });

            return false;
        }

        return result;
    }
}
