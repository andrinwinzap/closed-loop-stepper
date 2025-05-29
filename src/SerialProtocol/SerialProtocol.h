#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>

constexpr uint8_t START_BYTE = 0xAA;
constexpr uint8_t ESCAPE_BYTE = 0xAB;
constexpr uint8_t ESCAPE_MASK = 0x20;

constexpr size_t PAYLOAD_BUFFER_SIZE = 1024;

enum class ParserState {
    WAIT_START,
    READ_CMD,
    READ_LEN,
    READ_PAYLOAD,
    READ_CHECKSUM,
    VALIDATE,
    DISPATCH
};

void crc8(uint8_t& crc8, uint8_t byte);
uint8_t crc8(const uint8_t* data, size_t length);

size_t escape_data(uint8_t* data, size_t length, uint8_t* output, size_t max_output_len);

class SerialParser {
public:
    using DispatchCallback = void (*)(uint8_t cmd, const uint8_t* payload_buffer, size_t payload_buffer_len);

    SerialParser(DispatchCallback cb = nullptr) : on_dispatch(cb) {}

    void parse(uint8_t byte);

private:
    ParserState state = ParserState::WAIT_START;
    uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];
    size_t payload_buffer_len = 0;
    size_t payload_bytes_read = 0;
    uint16_t len;
    uint8_t len_bytes_read = 0;
    uint8_t cmd, checksum;
    uint8_t crc8_acc = 0x00;
    bool escapeNext = false;

    DispatchCallback on_dispatch = nullptr;

    void validate();
    void dispatch();
    void reset();

};

#endif