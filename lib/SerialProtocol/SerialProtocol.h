#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <Serialization.h>
#include <macros.h>

constexpr size_t MAX_PAYLOAD_SIZE = 1024;
constexpr size_t MAX_PACKET_SIZE = MAX_PAYLOAD_SIZE + 4;        // Max unescaped packet size = cmd (1) + len (2) + payload + checksum (1)
constexpr size_t MAX_ESCAPED_PACKET_SIZE = MAX_PACKET_SIZE * 2; // worst case
static constexpr size_t CMD_QUEUE_SIZE = 8;

enum class ParserState
{
    WAIT_START,
    READ_ADDR,
    READ_CMD,
    READ_LEN,
    READ_PAYLOAD,
    READ_CHECKSUM
};

struct Command
{
    uint8_t cmd;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    size_t payload_len;
};

namespace Byte
{
    namespace Protocol
    {
        enum : uint8_t
        {
            START = 0xAA,
            ESCAPE = 0xAB,
            ESCAPE_MASK = 0x20,
            CRC8_POLY = 0x07
        };
    }

    namespace Command
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

    namespace Status
    {
        enum : uint8_t
        {
            IDLE = 0x01,
            HOMING = 0x02,
            EXECUTING_TRAJECTORY = 0x03,
        };
    }

    namespace Address
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
}

class SerialParser
{
public:
    SerialParser(uint8_t addr)
        : address(addr) {}

    size_t available() const { return queue_count; }
    const Command *read();
    void parse(uint8_t byte);

private:
    ParserState state = ParserState::WAIT_START;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    size_t payload_len = 0;
    uint16_t len;
    uint8_t len_bytes_read = 0;
    uint8_t cmd, checksum;
    uint8_t crc8_acc = 0x00;
    bool escape_next = false;
    uint8_t address;

    Command queue[CMD_QUEUE_SIZE];
    size_t queue_head = 0;
    size_t queue_tail = 0;
    size_t queue_count = 0;

    void validate();
    void reset();
    void update_crc8(uint8_t byte);
    void enqueue_command(uint8_t cmd, const uint8_t *payload, size_t payload_len);
};

class SerialProtocol
{
public:
    SerialProtocol(Stream &serial, uint8_t addr)
        : serial_port(serial), address(addr), parser(addr) {}

    size_t available();
    const Command *read();
    void send_packet(uint8_t cmd, const uint8_t *payload = nullptr, uint16_t payload_len = 0);
    void send_packet(uint8_t cmd, const uint8_t payload_byte);

private:
    Stream &serial_port;
    uint8_t address;
    SerialParser parser;
    uint8_t packet[MAX_PACKET_SIZE];
    size_t escape_packet(const uint8_t *data, size_t len, uint8_t *escaped_packet) const;
    uint8_t crc8(const uint8_t *data, size_t len);
    void parse_serial();
};

#endif