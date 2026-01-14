#pragma once

#include <stdint.h>
#include "imu.hpp"
#include "filter.hpp"

struct CircularMotionData
{
    float invRadius;
};

typedef int16_t EncodedRadius;
typedef MovAvgFilter<uint8_t, int16_t, int32_t> RawValueFilter;
typedef CombinedFilter<uint8_t, EncodedRadius, int32_t> RadiusFilter;

class CircularMotionCalibrationBuilder
{
private:
    int32_t gyroXAcc;
    int32_t gyroYAcc;
    uint16_t sampleCount;

    int16_t calcOffset(int32_t accValue)
    {
        return -(int16_t)(accValue / sampleCount);
    }

public:
    CircularMotionCalibrationBuilder()
    {
        gyroXAcc = 0;
        gyroYAcc = 0;
        sampleCount = 0;
    }

    uint16_t getSampleCount()
    {
        return sampleCount;
    }

    void addSample(ImuData &sample)
    {
        gyroXAcc += sample.fields.gyroX;
        gyroYAcc += sample.fields.gyroY;
        sampleCount++;
    }

    friend class CircularMotionProcessor;
};

// ToDo: calibration
class CircularMotionProcessor
{
private:
    int16_t gyroOffsetX;
    int16_t gyroOffsetY;

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
        : gyroXFilter(gyroXFilter), gyroYFilter(gyroYFilter), radiusFilter(radiusFilter)
    {
        gyroOffsetX = 0;
        gyroOffsetY = 0;
    }

    void setCalibration(CircularMotionCalibrationBuilder &calibration);
    void update(const ImuData &imuData, const ImuRangeFactors &imuRange, CircularMotionData &result);
};
