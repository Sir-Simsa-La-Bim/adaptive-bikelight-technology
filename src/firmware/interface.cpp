#include "interface.hpp"
#include <string.h>

const uint8_t maxPayloadLength = 52;

void UserInterface::update()
{
    while (true)
    {
        switch (state)
        {
        default:
        case UserInterfaceState::Ready:
        {
            int data = stream.read();
            if (data < 0)
                return;

            switch ((UserInterfaceCode)data)
            {
            case UserInterfaceCode::Null:
                break;
            case UserInterfaceCode::Ping:
                stream.write((uint8_t)UserInterfaceCode::Ping);
                break;
            case UserInterfaceCode::ReadParameter:
                state = UserInterfaceState::AwaitParameterIdForReading;
                break;
            case UserInterfaceCode::WriteParameter:
                state = UserInterfaceState::AwaitParameterIdForWriting;
                break;
            default:
                stream.write((uint8_t)UserInterfaceCode::InvalidRequestError);
                break;
            }

            break;
        }
        case UserInterfaceState::AwaitParameterIdForReading:
        {
            int data = stream.read();
            if (data < 0)
                return;

            uint8_t parameterId = data;

            if (parameterId >= paramTableLength)
                stream.write((uint8_t)UserInterfaceCode::InvalidParameterIdError);
            else
            {
                stream.write((uint8_t)UserInterfaceCode::ReadParameter);
                stream.write(parameterId);

                const ParamDef &param = paramTable[parameterId];
                stream.write(param.size);

                uint8_t checksum = 0;
                uint8_t remainingBytes = param.size;
                uint8_t *paramDataPtr = param.location;
                while (remainingBytes > 0)
                {
                    uint8_t paramData = *paramDataPtr;
                    checksum += paramData;
                    stream.write(paramData);
                    remainingBytes--;
                    paramDataPtr++;
                }

                stream.write(-checksum);
            }

            state = UserInterfaceState::Ready;
            break;
        }
        case UserInterfaceState::AwaitParameterIdForWriting:
        {
            int data = stream.read();
            if (data < 0)
                return;

            parameterId = data;
            state = UserInterfaceState::AwaitPayloadLength;
            break;
        }
        case UserInterfaceState::AwaitPayloadLength:
        {
            int data = stream.read();
            if (data < 0)
                return;

            payloadLength = data;

            if (payloadLength > maxPayloadLength)
            {
                stream.write((uint8_t)UserInterfaceCode::InvalidRequestError);
                state = UserInterfaceState::DiscardPayload;
            }
            else
                state = UserInterfaceState::AwaitPayload;
            break;
        }
        case UserInterfaceState::AwaitPayload:
        {
            int available = stream.available();
            if (available < (int)payloadLength + 1)
                return;

            uint8_t payload[maxPayloadLength];

            uint8_t checksum = 0;
            uint8_t remainingBytes = payloadLength;
            uint8_t *payloadPtr = payload;
            while (remainingBytes > 0)
            {
                uint8_t paramData = stream.read();
                checksum += paramData;
                *payloadPtr = paramData;
                remainingBytes--;
                payloadPtr++;
            }

            checksum += stream.read();

            if (checksum != 0)
                stream.write((uint8_t)UserInterfaceCode::ChecksumError);
            else if (parameterId >= paramTableLength)
                stream.write((uint8_t)UserInterfaceCode::InvalidParameterIdError);
            else
            {
                const ParamDef &param = paramTable[parameterId];
                if (param.size != payloadLength)
                    stream.write((uint8_t)UserInterfaceCode::InvalidParameterLengthError);
                else
                {
                    stream.write((uint8_t)UserInterfaceCode::WriteParameter);
                    stream.write(parameterId);
                    memcpy(param.location, payload, payloadLength);
                }
            }

            state = UserInterfaceState::Ready;
            break;
        }
        case UserInterfaceState::DiscardPayload:
        {
            while (payloadLength > 0)
            {
                int data = stream.read();
                if (data < 0)
                    return;
                payloadLength--;
            }

            state = UserInterfaceState::DiscardChecksum;
            break;
        }
        case UserInterfaceState::DiscardChecksum:
        {
            int data = stream.read();
            if (data < 0)
                return;

            state = UserInterfaceState::Ready;
            break;
        }
        }
    }
}
