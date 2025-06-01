#ifndef PROTOCOL_BYTE_DEFINITIONS_H
#define PROTOCOL_BYTE_DEFINITIONS_H

#include <Arduino.h>

namespace CommandByte
{
    enum : uint8_t
    {
        PING = 0x01,
        HOME = 0x02,
        POS = 0x03,
        LOAD_TRAJ = 0x04,
        EXEC_TRAJ = 0x05,
        FINISHED = 0x06,
        STATUS = 0x07,
        ACK = 0xEE,
        NACK = 0xFF
    };
}

namespace StatusByte
{
    enum : uint8_t
    {
        IDLE = 0x01,
        HOMING = 0x02,
        EXECUTING_TRAJECTORY = 0x03,
    };
}

namespace AddressByte
{
    enum : uint8_t
    {
        BROADCAST = 0x00,
        MASTER = 0x01,
        ACTUATOR_1 = 0x02,
        ACTUATOR_2 = 0x03,
        ACTUATOR_3 = 0x04,
        ACTUATOR_4 = 0x05,
        ACTUATOR_5 = 0x06,
        ACTUATOR_6 = 0x07,
        TOOL = 0x08
    };
}

#endif