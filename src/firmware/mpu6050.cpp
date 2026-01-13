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

    const float gyroRangeBaseFactor_deg = 250.0F / (1uL << 15);
    const float gyroRangeBaseFactor_rad = DEG_TO_RAD * gyroRangeBaseFactor_deg;
    const float accelRangeBaseFactor_g = 2.0F / (1uL << 15);
    const float accelRangeBaseFactor_si = accelRangeBaseFactor_g * gravityAccel_si;

    const float gyroRangeFactors_deg[] = {
        gyroRangeBaseFactor_deg,
        gyroRangeBaseFactor_deg * 2,
        gyroRangeBaseFactor_deg * 4,
        gyroRangeBaseFactor_deg * 8,
    };

    const float gyroRangeFactors_rad[] = {
        gyroRangeBaseFactor_rad,
        gyroRangeBaseFactor_rad * 2,
        gyroRangeBaseFactor_rad * 4,
        gyroRangeBaseFactor_rad * 8,
    };

    const float accelRangeFactors_g[] = {
        accelRangeBaseFactor_g,
        accelRangeBaseFactor_g * 2,
        accelRangeBaseFactor_g * 4,
        accelRangeBaseFactor_g * 8,
    };

    const float accelRangeFactors_si[] = {
        accelRangeBaseFactor_si,
        accelRangeBaseFactor_si * 2,
        accelRangeBaseFactor_si * 4,
        accelRangeBaseFactor_si * 8,
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

    void SensorDataRegisterContent::decode(ImuData &result)
    {
        result.fields.accelX = getInt16Value(&bytes[0]);
        result.fields.accelY = getInt16Value(&bytes[2]);
        result.fields.accelZ = getInt16Value(&bytes[4]);
        result.fields.gyroX = getInt16Value(&bytes[8]);
        result.fields.gyroY = getInt16Value(&bytes[10]);
        result.fields.gyroZ = getInt16Value(&bytes[12]);
    }

    void FifoSensorData::decode(ImuData &result)
    {
        result.fields.accelX = getInt16Value(&bytes[0]);
        result.fields.accelY = getInt16Value(&bytes[2]);
        result.fields.accelZ = getInt16Value(&bytes[4]);
        result.fields.gyroX = getInt16Value(&bytes[6]);
        result.fields.gyroY = getInt16Value(&bytes[8]);
        result.fields.gyroZ = getInt16Value(&bytes[10]);
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

    DriverError DirectDriver::read(ImuData &data)
    {
        interface.resetError();

        SensorDataRegisterContent rawData;
        interface.getSensorData(rawData);

        if (interface.hasError())
            return DriverError::ConnectionError;

        rawData.decode(data);

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

    bool FifoDriverReader::popNextFromFifo(ImuData &data)
    {
        interface.popFifo(dataBuffer.bytes, sizeof(dataBuffer.bytes));
        availableDataCount -= sizeof(dataBuffer.bytes);

        if (interface.hasError())
        {
            error |= DriverError::ConnectionError;
            return false;
        }

        dataBuffer.decode(data);

        return true;
    }

    bool FifoDriverReader::getNext(ImuData &data)
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
