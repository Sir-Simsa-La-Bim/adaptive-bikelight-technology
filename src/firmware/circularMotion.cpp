#include "circularMotion.hpp"
#include <math.h>
#include "util.hpp"

void CircularMotionCalibrationBuilder::build(CircularMotionCalibration &calibration) const
{
    calibration.gyroYawOffset = calcOffset(gyroYawAcc);
    calibration.gyroPitchOffset = calcOffset(gyroPitchAcc);
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
        quantizedRadius = -(((uint16_t)1 << 15) - encodedRadius);

    const float quantizationFactor = (float)(((uint16_t)1 << 15) - 1) / maxEncodableRadius;
    return quantizationFactor / quantizedRadius;
}

EncodedRadius CircularMotionProcessor::calcRadius(float gyroYaw, float gyroPitch)
{
    // total^2 = yaw^2 + pitch^2
    // az = pitch / yaw * g
    // radius = az / total^2 = (pitch / yaw * g) / (yaw^2 + pitch^2) = (g * pitch) / (yaw * (yaw^2 + pitch^2))

    if (gyroYaw == 0)
        return infiniteRadius;

    float yawAbs = fabsf(gyroYaw);
    float pitchAbs = fabsf(gyroPitch);

    Float32Info radius = (gravityAccel_si * pitchAbs) / (yawAbs * (yawAbs * yawAbs + pitchAbs * pitchAbs));

    // Quick bit-hack to set the radius sign equal to the yaw sign
    radius.setSign(Float32Info(gyroYaw).getSign());

    return encodeRadius(radius.value);
}

void CircularMotionProcessor::update(const ImuData &imuData, const ImuRangeFactors &imuRange, CircularMotionData &result)
{
    float gyroYaw = -gyroXFilter.updateAsFloat(getGyroYaw(imuData) + calibration.gyroYawOffset) * imuRange.gyroFactor_rad;
    float gyroPitch = gyroYFilter.updateAsFloat(getGyroPitch(imuData) + calibration.gyroPitchOffset) * imuRange.gyroFactor_rad;
    EncodedRadius unfilteredEncodedRadius = calcRadius(gyroYaw, gyroPitch);
    EncodedRadius filteredEncodedRadius = radiusFilter.update(unfilteredEncodedRadius);
    result.invRadius = decodeInvRadius(filteredEncodedRadius);
}
