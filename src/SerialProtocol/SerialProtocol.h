#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>

constexpr uint8_t START_BYTE = 0xAA;
constexpr uint8_t ESCAPE_BYTE = 0xAB;
constexpr uint8_t ESCAPE_MASK = 0x20;

constexpr size_t PAYLOAD_BUFFER_SIZE = 1024;
constexpr size_t MAX_PACKET_BUFFER_SIZE = PAYLOAD_BUFFER_SIZE + 4;

enum class ParserState {
    WAIT_START,
    READ_CMD,
    READ_LEN,
    READ_PAYLOAD,
    READ_CHECKSUM
};

class SerialParser {
public:
    using DispatchCallback = void (*)(uint8_t cmd, const uint8_t* payload, size_t payload_len);

    SerialParser(DispatchCallback cb = nullptr) : on_dispatch(cb) {}

    void parse(uint8_t byte);

private:
    ParserState state = ParserState::WAIT_START;
    uint8_t payload[PAYLOAD_BUFFER_SIZE];
    size_t payload_len = 0;
    uint16_t len;
    uint8_t len_bytes_read = 0;
    uint8_t cmd, checksum;
    uint8_t crc8_acc = 0x00;
    bool escape_next = false;

    DispatchCallback on_dispatch = nullptr;

    void validate();
    void dispatch();
    void reset();
    void update_crc8(uint8_t byte);

};

class SerialSender {
public:
    void send_packet(uint8_t cmd, const uint8_t* payload, uint16_t length);

private:
    size_t escape_buffer_index = 0;
    uint8_t packet_buffer[MAX_PACKET_BUFFER_SIZE];
    uint8_t escape_buffer[MAX_PACKET_BUFFER_SIZE*2];
    void escape_packet(uint8_t* data, size_t length);
    void writeUint16LE(uint8_t* buffer, uint16_t value);
    uint8_t crc8(const uint8_t* data, size_t length);
};

#endif