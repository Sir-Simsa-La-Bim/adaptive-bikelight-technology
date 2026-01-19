from serial import Serial
from serial.serialutil import SerialException
from math import pi, isfinite
import asyncio
import time
import struct
import typing

_BYTE_ORDER = 'little'
_IMU_GYRO_FACTOR = 250 * pi / 180.0 / (1 << 15)
_IMU_ACCEL_FACTOR = 9.81 * 2.0 / (1 << 15)

class _Codes:
    Null = 0

    Ping = 1
    ReadParameter = 2
    WriteParameter = 3

    ChecksumError = 4
    InvalidRequestError = 5
    InvalidParameterIdError = 6
    InvalidParameterLengthError = 7
    FatalError = 8


class Parameter:
    """
    Parameter
    ---
    Represents a variable in the firmware that can be read and written by the Driver class.
    """

    __slots__ = ('name', '_paramId', '_byteSize')

    name: str
    _paramId: int
    _byteSize: int

    def __init__(self, name: str, paramId: int, byteSize: int) -> None:
        self.name = name
        self._paramId = paramId
        self._byteSize = byteSize

    def isValid(self, value : typing.Any)->bool:
        """
        Tests if the given value can be assigned to this parameter.
        
        :param value: The value to test.
        :type value: typing.Any
        :return: True on success.
        :rtype: bool
        """
        return True

    def _encode(self, value : typing.Any)->bytes:
        raise RuntimeError('not implemented')
    
    def _decode(self, raw : bytes)->typing.Any:
        raise RuntimeError('not implemented')

class IntParameter(Parameter):
    __slots__ = ('_signed',)

    _signed: bool

    def __init__(self, name: str, paramId: int, byteSize: int, signed: bool) -> None:
        super().__init__(name, paramId, byteSize)
        self._signed = signed

    def isValid(self, value: typing.Any) -> bool:
        return isinstance(value, int)

    def _encode(self, value: int) -> bytes:
        return value.to_bytes(self._byteSize, _BYTE_ORDER, signed=self._signed)
    
    def _decode(self, raw: bytes) -> int:
        return int.from_bytes(raw, _BYTE_ORDER, signed=self._signed)

class FloatParameter(Parameter):
    __slots__ = ()

    def __init__(self, name: str, paramId: int) -> None:
        super().__init__(name, paramId, 4)

    def isValid(self, value: typing.Any) -> bool:
        return isinstance(value, float)

    def _encode(self, value: float) -> bytes:
        return struct.pack('<f', value)
    
    def _decode(self, raw: bytes) -> float:
        return struct.unpack('<f', raw)[0]

class EnumMember:
    __slots__ = ('name', 'value', '_bytes')

    name: str
    value: int
    _bytes: bytes|None

    def __init__(self, name: str, value: int) -> None:
        self.name = name
        self.value = value
        self._bytes = None

    def __repr__(self) -> str:
        return self.name

class EnumParameter(Parameter):
    _signed: bool
    _members: dict[bytes, EnumMember]

    def __init__(self, name: str, paramId: int, members : typing.Iterable[EnumMember], byteSize: int = 1, signed: bool = False) -> None:
        super().__init__(name, paramId, byteSize)
        self._signed = signed
        self._members = dict()

        for member in members:
            valueBytes = member.value.to_bytes(byteSize, _BYTE_ORDER, signed=signed)
            if valueBytes in self._members:
                raise ValueError('ambiguous values')
            member._bytes = valueBytes
            self._members[valueBytes] = member

    @property
    def members(self)->typing.Iterable[EnumMember]:
        return self._members.values()

    def isValid(self, value : typing.Any)->bool:
        if not isinstance(value, EnumMember):
            return False
        member : EnumMember|None = self._members.get(value._bytes, None) # type: ignore
        return member == value
    
    def _encode(self, value: EnumMember) -> bytes:
        if value._bytes not in self._members:
            raise ValueError('unknown member')
        return value._bytes
    
    def _decode(self, raw: bytes) -> EnumMember:
        return self._members[raw]

class ImuData:
    __slots__ = ('gyroX', 'gyroY', 'gyroZ', 'accelX', 'accelY', 'accelZ')
    gyroX: float
    gyroY: float
    gyroZ: float
    accelX: float
    accelY: float
    accelZ: float

    def __init__(self, gyroX: float = 0.0, gyroY: float = 0.0, gyroZ: float = 0.0, accelX: float = 0.0, accelY: float = 0.0, accelZ: float = 0.0) -> None:
        self.gyroX = gyroX
        self.gyroY = gyroY
        self.gyroZ = gyroZ
        self.accelX = accelX
        self.accelY = accelY
        self.accelZ = accelZ

    def __repr__(self) -> str:
        return f'ImuData( gyro = ({self.gyroX}, {self.gyroY}, {self.gyroZ}), accel = ({self.accelX}, {self.accelY}, {self.accelZ}) )'

class ImuDataParameter(Parameter):
    __slots__ = ()

    def __init__(self, name: str, paramId: int) -> None:
        super().__init__(name, paramId, 12)

    def isValid(self, value : typing.Any)->bool:
        return isinstance(value, ImuData)

    def _encode(self, value: ImuData) -> bytes:
        return struct.pack('<6h',
            value.gyroX / _IMU_GYRO_FACTOR,
            value.gyroY / _IMU_GYRO_FACTOR,
            value.gyroZ / _IMU_GYRO_FACTOR,
            value.accelX / _IMU_ACCEL_FACTOR,
            value.accelY / _IMU_ACCEL_FACTOR,
            value.accelZ / _IMU_ACCEL_FACTOR,
        )
    
    def _decode(self, raw: bytes) -> ImuData:
        values = struct.unpack('<6h', raw)
        return ImuData(
            values[0] * _IMU_GYRO_FACTOR,
            values[1] * _IMU_GYRO_FACTOR,
            values[2] * _IMU_GYRO_FACTOR,
            values[3] * _IMU_ACCEL_FACTOR,
            values[4] * _IMU_ACCEL_FACTOR,
            values[5] * _IMU_ACCEL_FACTOR,
        )

class ImuCalibration:
    __slots__ = ('gyroXOffset', 'gyroYOffset')
    gyroXOffset: float
    gyroYOffset: float

    def __init__(self, gyroXOffset : float = 0.0, gyroYOffset : float = 0.0) -> None:
        self.gyroXOffset = gyroXOffset
        self.gyroYOffset = gyroYOffset

    def __repr__(self) -> str:
        return f'ImuCalibration( gyroOffsets = (x: {self.gyroXOffset}, y: {self.gyroYOffset}) )'

class ImuCalibrationParameter(Parameter):
    __slots__ = ()

    def __init__(self, name: str, paramId: int) -> None:
        super().__init__(name, paramId, 4)

    def isValid(self, value : typing.Any)->bool:
        return isinstance(value, ImuCalibration)

    def _encode(self, value: ImuCalibration) -> bytes:
        return struct.pack('<2h',
            value.gyroXOffset / _IMU_GYRO_FACTOR,
            value.gyroYOffset / _IMU_GYRO_FACTOR,
        )
    
    def _decode(self, raw: bytes) -> ImuCalibration:
        values = struct.unpack('<2h', raw)
        return ImuCalibration(
            values[0] * _IMU_GYRO_FACTOR,
            values[1] * _IMU_GYRO_FACTOR,
        )

class CircularMotionData:
    __slots__ = ('invRadius',)

    invRadius: float

    def __init__(self, invRadius: float = 0.0) -> None:
        self.invRadius = invRadius

    def __repr__(self) -> str:
        return f'CircularMotionData( invRadius = {self.invRadius} )'

class CircularMotionDataParameter(Parameter):
    __slots__ = ()

    def __init__(self, name: str, paramId: int) -> None:
        super().__init__(name, paramId, 4)

    def isValid(self, value: typing.Any) -> bool:
        return isinstance(value, CircularMotionData)

    def _encode(self, value: CircularMotionData) -> bytes:
        return struct.pack('<f',
            value.invRadius,
        )
    
    def _decode(self, raw: bytes) -> CircularMotionData:
        values = struct.unpack('<f', raw)
        return CircularMotionData(
            values[0],
        )


OPERATING_MODE_AUTOMATIC = EnumMember('automatic', 0)
OPERATING_MODE_MANUAL = EnumMember('manual', 1)
OPERATING_MODE = EnumParameter('operating mode', paramId=0, byteSize=1, signed=False, members=(
    OPERATING_MODE_AUTOMATIC,
    OPERATING_MODE_MANUAL,
))

OPERATING_SUBMODE_MAIN = EnumMember('main', 0)
OPERATING_SUBMODE_HINT = EnumMember('hint', 1)
OPERATING_SUBMODE_FIXED = EnumMember('fixed', 2)
OPERATING_SUBMODE = EnumParameter('operating submode', paramId=1, byteSize=1, signed=False, members=(
    OPERATING_SUBMODE_MAIN,
    OPERATING_SUBMODE_HINT,
    OPERATING_SUBMODE_FIXED,
))

OPERATING_DIRECTION_LEFT = EnumMember('left', -1)
OPERATING_DIRECTION_NONE = EnumMember('none', 0)
OPERATING_DIRECTION_RIGHT = EnumMember('right', 1)
OPERATING_DIRECTION = EnumParameter('operating direction', paramId=2, byteSize=1, signed=True, members=(
    OPERATING_DIRECTION_LEFT,
    OPERATING_DIRECTION_NONE,
    OPERATING_DIRECTION_RIGHT,
))

MANUAL_STEPS = IntParameter('manual steps', paramId=3, byteSize=1, signed=True)
AUTOMATIC_DIRECTION = FloatParameter('automatic direction', paramId=4)

IMU_MODE_RUNNING = EnumMember('running', 0)
IMU_MODE_FREEZE_IMU_DATA = EnumMember('freeze imu data', 1)
IMU_MODE_FREEZE_MOTION_DATA = EnumMember('freeze motion data', 2)
IMU_MODE_FREEZE_AUTOMATIC_DIRECTION = EnumMember('freeze automatic direction', 3)
IMU_MODE = EnumParameter('imu mode', paramId=5, byteSize=1, signed=False, members=(
    IMU_MODE_RUNNING,
    IMU_MODE_FREEZE_IMU_DATA,
    IMU_MODE_FREEZE_MOTION_DATA,
    IMU_MODE_FREEZE_AUTOMATIC_DIRECTION,
))
IMU_TIMESTAMP = IntParameter('imu timestamp', paramId=6, byteSize=2, signed=False)
IMU_CALIBRATION = ImuCalibrationParameter('imu calibration', paramId=7)
IMU_DATA = ImuDataParameter('imu data', paramId=8)
MOTION_DATA = CircularMotionDataParameter('motion data', paramId=9)

PARAMETERS = (
    OPERATING_MODE,
    OPERATING_SUBMODE,
    OPERATING_DIRECTION,
    MANUAL_STEPS,
    AUTOMATIC_DIRECTION,
    IMU_MODE,
    IMU_TIMESTAMP,
    IMU_CALIBRATION,
    IMU_DATA,
    MOTION_DATA,
)
"""
All parameters of the firmware in no particular order.
"""


class DriverError(RuntimeError):
    pass

class DriverCommunicationError(DriverError):
    pass

class DriverTimeoutError(DriverError):
    pass

class Driver:
    """
    Driver
    ---

    This class handles allows communication with the firmware.
    All actions are triggered by the dedicated methods.
    """
    _connection: Serial

    def __init__(self, port: str, timeout: float = 0.05) -> None:
        self._connection = Serial(port, baudrate=115200, timeout=timeout)

    def interrupt(self)->None:
        """
        Attempt to reset the firmware's state machine when it got stuck for any reason.
        """
        self._connection.reset_input_buffer()
        self._connection.reset_output_buffer()
        self._connection.write(b'\0' * 256)

    def ping(self)->bool:
        """
        Check connection to firmware by sending a ping message.
        
        :return: True if on success
        :rtype: bool
        """

        self._connection.reset_input_buffer()
        self._connection.write(bytes((_Codes.Ping,)))

        replyCodeBytes = self._connection.read(1)
        if len(replyCodeBytes) < 1:
            return False
        if replyCodeBytes[0] != _Codes.Ping:
            raise DriverCommunicationError('invalid reply code', replyCodeBytes[0])
        return True

    def readParameter(self, parameter : Parameter)->typing.Any:
        """
        Reads the value of the given parameter and returns it.
        
        :param parameter: the parameter to be read
        :type parameter: Parameter
        :return: the parameter value
        :rtype: Any
        """

        self._connection.reset_input_buffer()
        self._connection.write(bytes((_Codes.ReadParameter, parameter._paramId)))

        replyCodeBytes = self._connection.read(1)
        if len(replyCodeBytes) < 1:
            raise DriverTimeoutError()
        if replyCodeBytes[0] != _Codes.ReadParameter:
            raise DriverCommunicationError('invalid reply code', replyCodeBytes[0])
        
        parameterIdBytes = self._connection.read(1)
        if len(parameterIdBytes) < 1:
            raise DriverTimeoutError()
        if parameterIdBytes[0] != parameter._paramId:
            raise DriverCommunicationError('invalid parameter id', parameterIdBytes[0])
        
        payloadLengthBytes = self._connection.read(1)
        if len(payloadLengthBytes) < 1:
            raise DriverTimeoutError()
        payloadLength = payloadLengthBytes[0]
        
        payloadBytes = self._connection.read(payloadLength)
        if len(payloadBytes) < payloadLength:
            raise DriverTimeoutError()
        
        checksumBytes = self._connection.read(1)
        if len(payloadLengthBytes) < 1:
            raise DriverTimeoutError()
        
        if payloadLength != parameter._byteSize:
            raise DriverCommunicationError('unexpected payload length', payloadLength)
        checksum = 0
        for byte in payloadBytes:
            checksum = (checksum + byte) & 0xFF
        checksum = (checksum + checksumBytes[0]) & 0xFF
        if checksum != 0:
            raise DriverCommunicationError('invalid checksum error', checksum)
        
        return parameter._decode(payloadBytes)
        
    def writeParameter(self, parameter : Parameter, value : typing.Any)->None:
        """
        Writes the given value into the specified parameter.

        :param parameter: the parameter to modify
        :type parameter: Parameter
        :param value: the new value of the parameter
        :type value: typing.Any
        """

        rawValue = parameter._encode(value)

        request = bytearray()
        request.append(_Codes.WriteParameter)
        request.append(parameter._paramId)
        request.append(len(rawValue))
        request.extend(rawValue)

        checksum = 0
        for byte in rawValue:
            checksum = (checksum + byte) & 0xFF
        request.append((-checksum) & 0xFF)

        self._connection.reset_input_buffer()
        self._connection.write(request)

        replyCodeBytes = self._connection.read(1)
        if len(replyCodeBytes) < 1:
            raise DriverTimeoutError()
        if replyCodeBytes[0] != _Codes.WriteParameter:
            raise DriverCommunicationError('invalid reply code', replyCodeBytes[0])
        
        parameterIdBytes = self._connection.read(1)
        if len(parameterIdBytes) < 1:
            raise DriverTimeoutError()
        if parameterIdBytes[0] != parameter._paramId:
            raise DriverCommunicationError('invalid parameter id', parameterIdBytes[0])

    def close(self)->None:
        """
        Closes the underlying serial connection.
        """
        self._connection.close()


class SyncedParameter:
    """
    SyncedParameter
    ---

    Provides a more conveniant way to deal with parameters by automatically synchronizing them.

    This class relies on async programming provided by the asyncio library but does not guarantee thread-safety.
    """

    __WAIT_NONE = 0
    __WAIT_FOR_READ = 1
    __WAIT_FOR_WRITE = 2

    __slots__ = ('_owner', '_parameter', '_value', '_syncTimestamp', '_waitHandle', '_waitType')

    _owner: 'ParameterSynchronizer'
    _parameter: Parameter
    _value: typing.Any|None
    _syncTimestamp: float

    _waitHandle: asyncio.Event|None
    _waitType: int

    def __init__(self, owner: 'ParameterSynchronizer', parameter: Parameter) -> None:
        self._owner = owner
        self._parameter = parameter
        self._value = None
        self._syncTimestamp = 0.0
        self._waitHandle = None
        self._waitType = SyncedParameter.__WAIT_NONE


    @property
    def name(self)->str:
        """
        The wrapped parameter's name.
        """
        return self._parameter.name

    @property
    def parameter(self)->Parameter:
        """
        The underlying parameter wrapped by this instance.
        """
        return self._parameter
    
    @property
    def synchronizer(self)->'ParameterSynchronizer':
        """
        The synchronizer object that handles this instance.
        """
        return self._owner

    @property
    def syncTimestamp(self)->float:
        """
        The last time this parameter was successfully synchronized.
        """
        return self._syncTimestamp


    async def _beginReadWait(self)->None:
        if self._waitType == SyncedParameter.__WAIT_NONE:
            self._waitType = SyncedParameter.__WAIT_FOR_READ
            self._waitHandle = asyncio.Event()
        elif self._waitType != SyncedParameter.__WAIT_FOR_READ:
            return
        
        await self._waitHandle.wait() # type: ignore

    async def _beginWriteWait(self)->None:
        if self._waitType == SyncedParameter.__WAIT_FOR_READ:
            self._waitHandle.set() # type: ignore
        if self._waitType != SyncedParameter.__WAIT_FOR_WRITE:
            self._waitType = SyncedParameter.__WAIT_FOR_WRITE
            self._waitHandle = asyncio.Event()
        
        await self._waitHandle.wait() # type: ignore

    def _realeseReadWait(self)->None:
        if self._waitType == SyncedParameter.__WAIT_FOR_READ:
            self._waitType = SyncedParameter.__WAIT_NONE
            self._waitHandle.set() # type: ignore
            self._waitHandle = None

    def _realeseWriteWait(self)->None:
        if self._waitType == SyncedParameter.__WAIT_FOR_WRITE:
            self._waitType = SyncedParameter.__WAIT_NONE
            self._waitHandle.set() # type: ignore
            self._waitHandle = None

    def _validateValue(self, value : typing.Any)->None:
        if not self._parameter.isValid(value):
            raise TypeError('invalid parameter value type')


    def isValid(self, value : typing.Any)->bool:
        """
        Tests if the given value can be assigned to the wrapped parameter.
        
        :param value: The value to test.
        :type value: typing.Any
        :return: True on success.
        :rtype: bool
        """
        return self._parameter.isValid(value)

    def getValue(self)->typing.Any|None:
        """
        Gets parameter most recent parameter value obtained by the last synchronization.
        If no synchronization has happened yet, None is returned.
        """
        return self._value
    
    def setValue(self, value : typing.Any)->None:
        """
        Set the parameter value and trigger a parameter synchronisation.
        
        :param value: The value to set.
        :type value: valid parameter value
        """
        self._validateValue(value)
        self._value = value
        self._realeseReadWait()
        self._owner._queueParameterWrite(self)

    async def getValueWait(self)->typing.Any:
        """
        Trigger a parameter synchronization and wait for its completion.
        Then return the obtained value.
        """

        if self._owner._queueParameterRead(self):
            await self._beginReadWait()
        return self._value

    async def setValueWait(self, value : typing.Any)->None:
        """
        Set the parameter value.
        Trigger a parameter synchronisation and return after its completion.
        
        :param value: The value to set.
        :type value: valid parameter value
        """

        self._validateValue(value)
        self._value = value
        if self._owner._queueParameterWrite(self):
            await self._beginWriteWait()
        
    def refresh(self)->None:
        """
        Triggers a parameter synchronization.
        """
        self._owner._queueParameterRead(self)

class ParameterSynchronizer:
    """
    ParameterSynchronizer
    ---

    This object automatically handles the synchronization of parameters via the Driver class.

    This class must be created while an async event loop is running or else an error will be thrown.
    """

    _CONNECTION_STATUS_PORT_UNAVAILABLE = 0
    _CONNECTION_NO_DEVICE = 1
    _CONNECTION_COMMUNICATION_ERROR = 2
    _CONNECTION_CONNECTED = 3
    _CONNECTION_CLOSED = 4

    __slots__ = ('_status', '_port', '_autoRefreshInterval', '_reconnectInterval', '_driver', '_parameters', '_readQueue', '_writeQueue', '_auxilaryList', '_isFullSync', '_syncTrigger', '_connectionWaitHandle', '_fullSyncWaitHandle')

    _status: int
    _port: str
    _autoRefreshInterval: float
    _reconnectInterval: float
    _driver: Driver|None

    _parameters: dict[Parameter, SyncedParameter]
    _readQueue: set[SyncedParameter]
    _writeQueue: set[SyncedParameter]
    _auxilaryList: list[SyncedParameter]

    _isFullSync : bool
    _syncTrigger: asyncio.Event
    _connectionWaitHandle: asyncio.Event
    _fullSyncWaitHandle: asyncio.Event

    @staticmethod
    def __validateInterval(interval : typing.Any):
        if not isinstance(interval, float):
            raise TypeError('autoRefreshInterval must be float')
        if not isfinite(interval) or interval <= 0.0:
            raise ValueError('autoRefreshInterval must be positive')

    def __init__(self, port: str, autoRefreshInterval : float = 1.0, reconnectInterval : float = 0.5) -> None:
        # This will raise an error if no event loop is running
        asyncio.get_running_loop()

        if not isinstance(port, str):
            raise TypeError('port must be str')
        ParameterSynchronizer.__validateInterval(autoRefreshInterval)
        ParameterSynchronizer.__validateInterval(reconnectInterval)

        self._status = ParameterSynchronizer._CONNECTION_STATUS_PORT_UNAVAILABLE
        self._port = port
        self._autoRefreshInterval = autoRefreshInterval
        self._reconnectInterval = reconnectInterval
        self._driver = None

        self._parameters = dict()
        self._readQueue = set()
        self._writeQueue = set()
        self._auxilaryList = list()

        self._isFullSync = False
        self._syncTrigger = asyncio.Event()
        self._connectionWaitHandle = asyncio.Event()
        self._fullSyncWaitHandle = asyncio.Event()

        asyncio.create_task(self._syncLoop())


    @property
    def port(self)->str:
        return self._port

    @property
    def isConnected(self)->bool:
        """
        Determines whether an active connection is available.
        """
        return self._status == ParameterSynchronizer._CONNECTION_CONNECTED
    
    @property
    def isClosed(self)->bool:
        """
        Determines whether this instance was closed.
        """
        return self._status == ParameterSynchronizer._CONNECTION_CLOSED


    def _queueParameterRead(self, parameter : SyncedParameter)->bool:
        if self._status == ParameterSynchronizer._CONNECTION_CLOSED:
            return False
        if parameter in self._writeQueue:
            return False
        
        self._readQueue.add(parameter)
        self._syncTrigger.set()
        return True

    def _queueParameterWrite(self, parameter : SyncedParameter)->bool:
        if self._status == ParameterSynchronizer._CONNECTION_CLOSED:
            return False

        self._readQueue.discard(parameter)
        self._writeQueue.add(parameter)
        self._syncTrigger.set()
        return True

    def _prepareFullSync(self)->None:
        for parameter in self._parameters.values():
            if parameter not in self._writeQueue:
                self._readQueue.add(parameter)

    def _readParameters(self)->None:
        self._auxilaryList.clear()

        try:
            for parameter in self._readQueue:
                value = self._driver.readParameter(parameter._parameter) # type: ignore
                parameter._value = value
                parameter._syncTimestamp = time.time()
                parameter._realeseReadWait()
                self._auxilaryList.append(parameter)
        finally:
            self._readQueue.difference_update(self._auxilaryList)
            self._auxilaryList.clear()

    def _writeParameters(self)->None:
        self._auxilaryList.clear()

        try:
            for parameter in self._writeQueue:
                self._driver.writeParameter(parameter._parameter, parameter._value) # type: ignore
                parameter._syncTimestamp = time.time()
                parameter._realeseWriteWait()
                self._auxilaryList.append(parameter)
        finally:
            self._writeQueue.difference_update(self._auxilaryList)
            self._auxilaryList.clear()
    
    async def _syncLoop(self)->None:
        lastFullSyncTime = 0.0

        while self._status != ParameterSynchronizer._CONNECTION_CLOSED:
            self._syncTrigger.clear()

            currSyncTime = time.time()
            if (currSyncTime - lastFullSyncTime) >= self._autoRefreshInterval:
                self._isFullSync = True
            if self._isFullSync:
                self._prepareFullSync()
            
            self._syncRoutine()

            if self._status == ParameterSynchronizer._CONNECTION_CONNECTED:
                if self._isFullSync:
                    self._fullSyncWaitHandle.set()
                    self._fullSyncWaitHandle.clear()
                    lastFullSyncTime = currSyncTime
                    self._isFullSync = False
                    timeout = self._autoRefreshInterval - time.time() + currSyncTime
                else:
                    timeout = self._autoRefreshInterval - time.time() + lastFullSyncTime
                self._connectionWaitHandle.set()
            else:
                self._connectionWaitHandle.clear()
                timeout = self._reconnectInterval

            if timeout > 0:
                try:
                    async with asyncio.timeout(timeout):
                        await self._syncTrigger.wait()
                except TimeoutError:
                    pass

    def _syncRoutine(self)->None:
        if self._status == ParameterSynchronizer._CONNECTION_CLOSED:
            return

        if self._status == ParameterSynchronizer._CONNECTION_STATUS_PORT_UNAVAILABLE:
            try:
                self._driver = Driver(self._port)
                self._status = ParameterSynchronizer._CONNECTION_NO_DEVICE
            except SerialException:
                return
            
        try:
            if self._status in (ParameterSynchronizer._CONNECTION_NO_DEVICE, ParameterSynchronizer._CONNECTION_COMMUNICATION_ERROR):
                if not self._driver.ping(): # type: ignore
                    self._status = ParameterSynchronizer._CONNECTION_NO_DEVICE
                    return
                
                self._status = ParameterSynchronizer._CONNECTION_CONNECTED
                

            if self._status == ParameterSynchronizer._CONNECTION_CONNECTED:
                self._writeParameters()
                self._readParameters()


        except DriverError:
            self._status = ParameterSynchronizer._CONNECTION_COMMUNICATION_ERROR
            self._driver.interrupt() # type: ignore

        except SerialException:
            if self._driver is not None:
                self._driver.close()
                self._driver = None
            self._status = ParameterSynchronizer._CONNECTION_STATUS_PORT_UNAVAILABLE

    
    def changeAutoRefreshInterval(self, newValue: float):
        """
        Sets the auto update interval to the given value. This triggers an immediate synchronization.

        :param newValue: Description
        :type newValue: float
        """
        ParameterSynchronizer.__validateInterval(newValue)
        self._autoRefreshInterval = newValue
        self._syncTrigger.set()


    async def waitForConnection(self)->None:
        """
        Blocks until a connection was established.
        """
        await self._connectionWaitHandle.wait()

    async def waitForFullSync(self)->None:
        """
        Blocks until a full synchronization completes successfully.
        """
        await self._fullSyncWaitHandle.wait()


    def add(self, parameter : Parameter)->SyncedParameter:
        """
        Adds the given parameter to the synchronization list and return a wrapper around it.
        
        :param parameter: The parameter to synchronize.
        :type parameter: Parameter
        :return: The synchronized wrapper object.
        :rtype: SyncedParameter
        """
        if parameter in self._parameters:
            return self._parameters[parameter]
        
        wrapper = SyncedParameter(self, parameter)
        self._parameters[parameter] = wrapper
        return wrapper
        
    def refresh(self)->None:
        """
        Manually triggers a refresh for all parameters managed by this instance.
        """

        if self._status == ParameterSynchronizer._CONNECTION_CLOSED:
            return
        
        self._isFullSync = True
        self._syncTrigger.set()

    def close(self)->None:
        """
        Closes this instance and releases the underlying driver.
        """

        self._status = ParameterSynchronizer._CONNECTION_CLOSED

        if self._driver is not None:
            self._driver.close()
            self._driver = None

        self._readQueue.clear()
        self._writeQueue.clear()
        
        self._syncTrigger.set()
        self._connectionWaitHandle.clear()
        self._fullSyncWaitHandle.clear()
