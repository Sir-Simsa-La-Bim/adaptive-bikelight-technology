#pragma once

#include <stdint.h>
#include "compiler_switches.h"
#include "imu.hpp"
#include "filter.hpp"

#if ENABLE_INTERFACE
#include "interface.hpp"
#endif

inline int16_t getGyroYaw(const ImuData &sample)
{
    return sample.fields.gyroX;
}

inline int16_t getGyroPitch(const ImuData &sample)
{
    return sample.fields.gyroY;
}

inline int16_t getGyroRoll(const ImuData &sample)
{
    return sample.fields.gyroZ;
}

struct CircularMotionData
{
    float invRadius;
};

typedef int16_t EncodedRadius;
typedef MovAvgFilter<uint8_t, int16_t, int32_t> RawValueFilter;
typedef CombinedFilter<uint8_t, EncodedRadius, int32_t> RadiusFilter;

class CircularMotionCalibration
{
public:
    int16_t gyroYawOffset;
    int16_t gyroPitchOffset;

    CircularMotionCalibration()
    {
        gyroYawOffset = 0;
        gyroPitchOffset = 0;
    }
};

class CircularMotionCalibrationBuilder
{
private:
    int32_t gyroYawAcc;
    int32_t gyroPitchAcc;
    uint16_t sampleCount;

    int16_t calcOffset(int32_t accValue) const
    {
        return -(int16_t)(accValue / sampleCount);
    }

public:
    CircularMotionCalibrationBuilder()
    {
        gyroYawAcc = 0;
        gyroPitchAcc = 0;
        sampleCount = 0;
    }

    uint16_t getSampleCount() const
    {
        return sampleCount;
    }

    void addSample(ImuData &sample)
    {
        gyroYawAcc += getGyroYaw(sample);
        gyroPitchAcc += getGyroPitch(sample);
        sampleCount++;
    }

    void build(CircularMotionCalibration &calibration) const;
};

class CircularMotionProcessor
{
private:
    CircularMotionCalibration calibration;

    MovAvgFilter<uint8_t, int16_t, int32_t> gyroXFilter;
    MovAvgFilter<uint8_t, int16_t, int32_t> gyroYFilter;
    CombinedFilter<uint8_t, EncodedRadius, int32_t> radiusFilter;

    static const EncodedRadius infiniteRadius = 0;
    static const int16_t maxEncodableRadius = 100;
    static EncodedRadius encodeRadius(float radius);
    static float decodeInvRadius(EncodedRadius encodedRadius);
    static EncodedRadius calcRadius(float wx, float wy);

public:
    CircularMotionProcessor(MovAvgFilter<uint8_t, int16_t, int32_t> gyroXFilter,
                            MovAvgFilter<uint8_t, int16_t, int32_t> gyroYFilter,
                            CombinedFilter<uint8_t, EncodedRadius, int32_t> radiusFilter)
        : gyroXFilter(gyroXFilter), gyroYFilter(gyroYFilter), radiusFilter(radiusFilter), calibration()
    {
    }

    void setCalibration(const CircularMotionCalibrationBuilder &builder)
    {
        builder.build(calibration);
    }

    void update(const ImuData &imuData, const ImuRangeFactors &imuRange, CircularMotionData &result);

#if ENABLE_INTERFACE
    constexpr ParamDef getCalibrationAsParameter()
    {
        return param(&calibration);
    }
#endif
};
