#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <Stream.h>

struct ParamDef
{
    void *location;
    uint8_t size;
};

template <class T>
constexpr ParamDef param(T *location)
{
    return (ParamDef){
        .location = location,
        .size = sizeof(T),
    };
}

enum class UserInterfaceState
{
    Ready,
    AwaitParameterIdForReading,
    AwaitParameterIdForWriting,
    AwaitPayloadLength,
    AwaitPayload,
    DiscardPayload,
    DiscardChecksum,
};

enum class UserInterfaceCode : uint8_t
{
    Null = 0,

    Ping = 1,
    ReadParameter = 2,
    WriteParameter = 3,

    ChecksumError = 4,
    InvalidRequestError = 5,
    InvalidParameterIdError = 6,
    InvalidParameterLengthError = 7,
    FatalError = 8,
};

class UserInterface
{
private:
    Stream &stream;
    const ParamDef *paramTable;
    uint8_t paramTableLength;

    UserInterfaceState state;
    uint8_t parameterId;
    uint8_t payloadLength;

public:
    UserInterface(Stream &stream, const ParamDef *paramTable, uint8_t paramTableLength)
        : stream(stream)
    {
        this->paramTable = paramTable;
        this->paramTableLength = paramTableLength;

        state = UserInterfaceState::Ready;
        parameterId = 0;
        payloadLength = 0;
    }

    void update();
};