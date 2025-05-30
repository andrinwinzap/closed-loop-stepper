#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <Serialization/Serialization.h>

constexpr uint8_t START_BYTE = 0xAA;
constexpr uint8_t ESCAPE_BYTE = 0xAB;
constexpr uint8_t ESCAPE_MASK = 0x20;

constexpr size_t MAX_PAYLOAD_SIZE = 1024;
constexpr size_t MAX_PACKET_SIZE = MAX_PAYLOAD_SIZE + 4;

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
    uint8_t payload[MAX_PAYLOAD_SIZE];
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

class SerialProtocol {
public:
    SerialProtocol(Stream& serial, SerialParser::DispatchCallback cb = nullptr)
        : serial_port(serial), parser(cb) {}

    void read_input();
    void send_packet(uint8_t cmd, const uint8_t* payload = nullptr, uint16_t payload_len = 0);

private:
    Stream& serial_port;
    SerialParser parser;
    uint8_t packet[MAX_PACKET_SIZE];
    uint8_t escaped_packet[MAX_PACKET_SIZE*2];
    uint8_t escape_packet(uint8_t* data, size_t len);
    uint8_t crc8(const uint8_t* data, size_t len);
};

#endif