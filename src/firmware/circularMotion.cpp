#include "circularMotion.hpp"
#include <math.h>
#include "util.hpp"

void CircularMotionCalibrationBuilder::build(CircularMotionCalibration &calibration) const
{
    calibration.gyroOffsetX = calcOffset(gyroXAcc);
    calibration.gyroOffsetY = calcOffset(gyroYAcc);
}

EncodedRadius CircularMotionProcessor::encodeRadius(float radius)
{
    if (!isfinite(radius))
        return infiniteRadius;

    float absRadius = fabsf(radius);
    if (absRadius > maxEncodableRadius)
        return infiniteRadius;

    const float quantizationFactor = (float)(((uint16_t)1 << 15) - 1) / maxEncodableRadius;
    uint16_t quantizedAbsRadius = (uint16_t)(absRadius * quantizationFactor);

    if (quantizedAbsRadius == 0) // radius == 0
    {
        // This is an error case and should not happen
        // use default value as fallback
        return infiniteRadius;
    }

    if (Float32Info(radius).getSign()) // radius < 0
        return ((uint16_t)1 << 15) - quantizedAbsRadius;
    else // r > 0
        return quantizedAbsRadius - ((uint16_t)1 << 15);
}

float CircularMotionProcessor::decodeInvRadius(EncodedRadius encodedRadius)
{
    if (encodedRadius == infiniteRadius)
        return 0;

    int16_t quantizedRadius;
    if (encodedRadius < 0) // radius > 0
        quantizedRadius = encodedRadius + ((uint16_t)1 << 15);
    else // radius < 0
        quantizedRadius = ((uint16_t)1 << 15) - encodedRadius;

    const float quantizationFactor = (float)(((uint16_t)1 << 15) - 1) / maxEncodableRadius;
    return quantizationFactor / quantizedRadius;
}

EncodedRadius CircularMotionProcessor::calcRadius(float wx, float wy)
{
    // w^2 = wx^2 + wy^2
    // az = wy / wx * g
    // radius = az / w^2 = (wy / wx * g) / (wx^2 + wy^2) = (g * wy) / (wx * (wx^2 + wy^2))

    if (wx == 0)
        return infiniteRadius;

    float radius = (gravityAccel_si * wy) / (wx * (wx * wx + wy * wy));
    return encodeRadius(radius);
}

void CircularMotionProcessor::update(const ImuData &imuData, const ImuRangeFactors &imuRange, CircularMotionData &result)
{
    float wx = gyroXFilter.updateAsFloat(imuData.fields.gyroX + calibration.gyroOffsetX) * imuRange.gyroFactor_rad;
    float wy = gyroYFilter.updateAsFloat(imuData.fields.gyroY + calibration.gyroOffsetY) * imuRange.gyroFactor_rad;
    EncodedRadius unfilteredEncodedRadius = calcRadius(wx, wy);
    EncodedRadius filteredEncodedRadius = radiusFilter.update(unfilteredEncodedRadius);
    result.invRadius = decodeInvRadius(filteredEncodedRadius);
}
