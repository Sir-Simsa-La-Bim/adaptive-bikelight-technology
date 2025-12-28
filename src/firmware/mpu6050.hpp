#pragma once

#include <stdint.h>
#include "util.hpp"
#include "vec.hpp"

namespace MPU6050
{
    /////////////// register settings ///////////////

    enum class Register : uint8_t
    {
        SampleRateDivider = 0x19,
        Configuration = 0x1A,
        GyroConfig = 0x1B,
        AccelConfig = 0x1C,
        FifoEnable = 0x23,
        InterruptBypassConfig = 0x37,
        InterruptEnable = 0x38,
        InterrupStatus = 0x3A,
        SensorData = 0x3B,
        AccelData = 0x3B,
        TempData = 0x41,
        GyroData = 0x43,
        UserControl = 0x6A,
        PowerManagement1 = 0x6B,
        PowerManagement2 = 0x6C,
        FifoCount = 0x72,
        FifoReadWrite = 0x74,
        WhoAmI = 0x75,
    };

    enum class DigitalLowPassFilter : uint8_t
    {
        Accel260Hz_Gyro256Hz = 0,
        Accel184Hz_Gyro188Hz = 1,
        Accel94Hz_Gyro98Hz = 2,
        Accel44Hz_Gyro42Hz = 3,
        Accel21Hz_Gyro20Hz = 4,
        Accel10Hz_Gyro10Hz = 5,
        Accel5Hz_Gyro5Hz = 6,
    };

    enum class ExternalFrameSync : uint8_t
    {
        InputDisabled = 0,
        TempOut = 1,
        GyroXOutL = 2,
        GyroYOutL = 3,
        GyroZOutL = 4,
        AccelXOutL = 5,
        AccelYOutL = 6,
        AccelZOutL = 7,
    };

    union Configuration
    {
        struct
        {
            DigitalLowPassFilter lowPassFilter : 3;
            ExternalFrameSync extFrameSync : 3;
            unsigned : 2;
        } fields;
        uint8_t bits;
    };

    enum class GyroFullScaleRange : uint8_t
    {
        _250_degPerSec = 0,
        _500_degPerSec = 1,
        _1000_degPerSec = 2,
        _2000_degPerSec = 3,
    };

    union GyroConfig
    {
        struct
        {
            unsigned : 3;
            GyroFullScaleRange fullScaleRange : 2;
            bool enableSelfTestZ : 1;
            bool enableSelfTestY : 1;
            bool enableSelfTestX : 1;
        } fields;
        uint8_t bits;
    };

    enum class AccelFullScaleRange : uint8_t
    {
        _2g = 0,
        _4g = 1,
        _8g = 2,
        _16g = 3,
    };

    union AccelConfig
    {
        struct
        {
            unsigned : 3;
            AccelFullScaleRange fullScaleRange : 2;
            bool enableSelfTestZ : 1;
            bool enableSelfTestY : 1;
            bool enableSelfTestX : 1;
        } fields;
        uint8_t bits;
    };

    union FifoEnable
    {
        struct
        {
            bool external0 : 1;
            bool external1 : 1;
            bool external2 : 1;
            bool accelXYZ : 1;
            bool gyroZ : 1;
            bool gyroY : 1;
            bool gyroX : 1;
            bool temp : 1;
        } fields;
        uint8_t bits;
    };

    enum class LogicLevel : uint8_t
    {
        ActiveHigh = 0,
        ActiveLow = 1,
    };

    union InterruptBypassConfig
    {
        struct
        {
            unsigned : 1;
            bool enableI2cBypass : 1;
            bool enableFrameSyncInterrupt : 1;
            LogicLevel frameSyncLogicLevel : 1;
            bool anyReadInterruptClear : 1;
            bool latchInterrupt : 1;
            bool openDrainInterrupt : 1;
            LogicLevel interruptLogicLevel : 1;
        } fields;
        uint8_t bits;
    };

    union InterruptEnable
    {
        struct
        {
            bool dataReady : 1;
            unsigned : 2;
            bool i2cMaster : 1;
            bool fifoOverflow : 1;
            unsigned : 3;
        } fields;
        uint8_t bits;
    };

    union InterruptStatus
    {
        struct
        {
            bool dataReady : 1;
            unsigned : 2;
            bool i2cMaster : 1;
            bool fifoOverflow : 1;
            unsigned : 3;
        } fields;
        uint8_t bits;
    };

    enum class ClockSelect : uint8_t
    {
        Internal8Mhz = 0,
        PllWithXGyroRef = 1,
        PllWithYGyroRef = 2,
        PllWithZGyroRef = 3,
        PllWithExternal32kHz = 4,
        PllWithExternal19MHz = 5,
        Stopped = 7,
    };

    struct SensorDataRegisterContent
    {
        uint8_t bytes[14];
    };

    union UserControl
    {
        struct
        {
            bool resetAllSignalPaths : 1;
            bool resetI2cMaster : 1;
            bool resetFifo : 1;
            unsigned : 1;
            unsigned : 1;
            bool enableI2cMasterMode : 1;
            bool fifoEnable : 1;
            unsigned : 1;
        } fields;
        uint8_t bits;
    };

    union PowerManagement1
    {
        struct
        {
            ClockSelect clockSelect : 3;
            bool disableTempSensor : 1;
            unsigned : 1;
            bool enableCycleMode : 1;
            bool enableSleepMode : 1;
            bool deviceReset : 1;
        } fields;
        uint8_t bits;
    };

    enum class WakeupFrequency : uint8_t
    {
        _1Hz25 = 0,
        _5Hz = 1,
        _20Hz = 2,
        _40Hz = 3,
    };

    union PowerManagement2
    {
        struct
        {
            bool disableGyroZ : 1;
            bool disableGyroY : 1;
            bool disableGyroX : 1;
            bool disableAccelZ : 1;
            bool disableAccelY : 1;
            bool disableAccelX : 1;
            WakeupFrequency wakupFrequency : 2;
        } fields;
        uint8_t bits;
    };

    /////////////// other ///////////////

    extern const float gyroRangeFactorsDeg[];
    extern const float gyroRangeFactorsRad[];
    extern const float accelRangeFactorsG[];
    extern const float accelRangeFactorsSi[];

    inline int16_t getInt16Value(uint8_t data[2])
    {
        return ((int16_t)data[0] << 8) | data[1];
    }

    struct FifoSensorData
    {
        uint8_t bytes[12];
    };

    struct SensorData
    {
        Vec3<int16_t> accel;
        Vec3<int16_t> gyro;

        void decode(const SensorDataRegisterContent &rawData);
        void decode(const FifoSensorData &rawData);
    };

    struct DriverSetupConfig
    {
        uint8_t sampleRateDivider;
        DigitalLowPassFilter filter;
        GyroFullScaleRange gyroRange;
        AccelFullScaleRange accelRange;

        float getGyroFactorInDeg() const
        {
            return gyroRangeFactorsDeg[(uint8_t)gyroRange];
        }

        float getGyroFactorInRad() const
        {
            return gyroRangeFactorsRad[(uint8_t)gyroRange];
        }

        float getAccelFactorInG() const
        {
            return accelRangeFactorsG[(uint8_t)accelRange];
        }

        float getAccelFactorInSi() const
        {
            return accelRangeFactorsSi[(uint8_t)accelRange];
        }
    };

    /////////////// driver ///////////////

    class Connection
    {
    private:
        uint8_t shiftedAddress;
        bool success;

    public:
        Connection(uint8_t address)
        {
            shiftedAddress = address << 1;
        }

        void begin();

        void resetError()
        {
            success = true;
        }

        bool hasError()
        {
            return !success;
        }

        bool popError()
        {
            bool temp = success;
            success = false;
            return !temp;
        }

        uint8_t readByte(Register reg);
        void readBytes(Register reg, uint8_t *data, uint8_t length);

        void writeByte(Register reg, uint8_t data);
        void writeBytes(Register reg, const uint8_t *data, uint8_t length);
    };

    class Interface
    {
    private:
        Connection connection;

    public:
        Interface(uint8_t address)
            : connection(address)
        {
        }

        void begin()
        {
            connection.begin();
        }

        void resetError()
        {
            connection.resetError();
        }

        bool hasError()
        {
            return connection.hasError();
        }

        bool popError()
        {
            return connection.popError();
        }

        void setSampleRateDivider(uint8_t divider)
        {
            connection.writeByte(Register::SampleRateDivider, divider - 1);
        }

        void setConfiguration(Configuration value)
        {
            connection.writeByte(Register::Configuration, value.bits);
        }

        void setGyroConfig(GyroConfig value)
        {
            connection.writeByte(Register::GyroConfig, value.bits);
        }

        void setAccelConfig(AccelConfig value)
        {
            connection.writeByte(Register::AccelConfig, value.bits);
        }

        void setFifoEnable(FifoEnable value)
        {
            connection.writeByte(Register::FifoEnable, value.bits);
        }

        void setInterruptBypassConfig(InterruptBypassConfig value)
        {
            connection.writeByte(Register::InterruptBypassConfig, value.bits);
        }

        void setInterruptEnable(InterruptEnable value)
        {
            connection.writeByte(Register::InterruptEnable, value.bits);
        }

        InterruptStatus getInterruptStatus()
        {
            return (InterruptStatus){
                .bits = connection.readByte(Register::InterrupStatus),
            };
        }

        void getSensorData(SensorDataRegisterContent &value)
        {
            connection.readBytes(Register::SensorData, value.bytes, sizeof(value.bytes));
        }

        void setUserControl(UserControl value)
        {
            connection.writeByte(Register::UserControl, value.bits);
        }

        void setPowerManagement1(PowerManagement1 value)
        {
            return connection.writeByte(Register::PowerManagement1, value.bits);
        }

        void setPowerManagement2(PowerManagement2 value)
        {
            return connection.writeByte(Register::PowerManagement2, value.bits);
        }

        uint16_t getFifoCount()
        {
            uint8_t data[2];
            connection.readBytes(Register::FifoCount, data, sizeof(data));
            return getInt16Value(data);
        }

        uint8_t popFifo()
        {
            return connection.readByte(Register::FifoReadWrite);
        }

        void popFifo(uint8_t *data, uint16_t length)
        {
            connection.readBytes(Register::FifoReadWrite, data, length);
        }

        void pushFifo(uint8_t data)
        {
            connection.writeByte(Register::FifoReadWrite, data);
        }

        void pushFifo(const uint8_t *data, uint16_t length)
        {
            connection.writeBytes(Register::FifoReadWrite, data, length);
        }

        uint8_t getWhoami()
        {
            return connection.readByte(Register::WhoAmI);
        }

        void reset()
        {
            setPowerManagement1((PowerManagement1){
                .fields = {
                    .clockSelect = MPU6050::ClockSelect::Internal8Mhz,
                    .disableTempSensor = false,
                    .enableCycleMode = false,
                    .enableSleepMode = false,
                    .deviceReset = true,
                },
            });
        }
    };

    enum class DriverError : uint8_t
    {
        Success = 0,
        ConnectionError = 1 << 1,
        UnknownDevice = 1 << 2,
        FifoOverflow = 1 << 3,
    };

    inline DriverError operator|(DriverError a, DriverError b)
    {
        return (DriverError)((uint8_t)a | (uint8_t)b);
    }

    inline DriverError operator|=(DriverError &a, DriverError b)
    {
        return a = (a | b);
    }

    class Driver
    {
    protected:
        Interface interface;

        DriverError basicSetup(const DriverSetupConfig &config);

    public:
        Driver(uint8_t address)
            : interface(address)
        {
        }

        void begin()
        {
            interface.begin();
        }
    };

    class DirectDriver : public Driver
    {
    public:
        DirectDriver(uint8_t address) : Driver(address) {}

        DriverError setup(const DriverSetupConfig &config);
        DriverError available(bool &result);
        DriverError read(SensorData &data);
    };

    class FifoDriverReader
    {
    private:
        Interface &interface;
        DriverError error;
        uint16_t availableDataCount;
        FifoSensorData dataBuffer;

        FifoDriverReader() = delete;

        FifoDriverReader(Interface &interface)
            : interface(interface)
        {
            error = DriverError::Success;
            availableDataCount = 0;
        }

        bool popNextFromFifo(SensorData &data);

    public:
        bool getNext(SensorData &data);

        DriverError getError() const
        {
            return error;
        }

        friend class FifoDriver;
    };

    class FifoDriver : public Driver
    {
    public:
        FifoDriver(uint8_t address) : Driver(address) {}

        DriverError setup(const DriverSetupConfig &config);

        FifoDriverReader read()
        {
            return FifoDriverReader(interface);
        }
    };
};
