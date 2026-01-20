from driver import *
from cancel_token.token import CancelToken
import typing

async def runLogger(port: str, path: str, cancelToken: CancelToken|None = None)->None:
    file = open(path, 'w')

    synchronizer = None
    try:
        synchronizer = ParameterSynchronizer(port)

        imuTimestampParam = synchronizer.add(IMU_TIMESTAMP)
        imuDataParam = synchronizer.add(IMU_DATA)
        motionDataParam = synchronizer.add(MOTION_DATA)
        directionParam = synchronizer.add(AUTOMATIC_DIRECTION)
        calibrationParam = synchronizer.add(IMU_CALIBRATION)

        timestampMask = (1 << (IMU_TIMESTAMP.byteSize * 8)) - 1

        print('waiting for connection... ')
        await synchronizer.waitForFullSync()

        file.write('time[ms],gyro x[rad/s],gyro y[rad/s],gyro z[rad/s], accel x[m/s^2], accel y[m/s^2],accel z[m/s^2],inv radius[1/m],direction[rad],\n')
        
        calibration = typing.cast(ImuCalibration, calibrationParam.getValue())
        file.write(f',{calibration.gyroXOffset},{calibration.gyroYOffset}\n')
        
        print('logging')

        lastTimestamp_ms = typing.cast(int, imuTimestampParam.getValue())
        consecutiveTimestamp_ms = 0

        while cancelToken is None or not cancelToken.triggered:
            currTimestamp_ms = await imuTimestampParam.getValueWait()
            if currTimestamp_ms == lastTimestamp_ms:
                continue

            deltaTime_ms = (currTimestamp_ms - lastTimestamp_ms) & timestampMask
            consecutiveTimestamp_ms += deltaTime_ms
            lastTimestamp_ms = currTimestamp_ms

            imuData: ImuData
            motionData: CircularMotionData
            direction: float

            imuData, motionData, direction = await asyncio.gather(
                imuDataParam.getValueWait(),
                motionDataParam.getValueWait(),
                directionParam.getValueWait(),
            )

            file.write(f'{consecutiveTimestamp_ms},{imuData.gyroX},{imuData.gyroY},{imuData.gyroZ},{imuData.accelX},{imuData.accelY},{imuData.accelZ},{motionData.invRadius},{direction}\n')

    finally:
        file.close()
        if synchronizer is not None:
            synchronizer.close()

def runLoggerForever(port: str, path: str)->None:
    import asyncio
    asyncio.get_event_loop().run_until_complete(runLogger(port, path))

if __name__ == '__main__':
    port = input('Run logger on port: ')
    path = input('Output file: ')
    runLoggerForever(port, path)