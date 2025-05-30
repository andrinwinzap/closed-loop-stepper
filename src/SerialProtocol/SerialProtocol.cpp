#include "SerialProtocol.h"

void SerialParser::update_crc8(uint8_t byte) {
        crc8_acc ^= byte;
        for (int i = 0; i < 8; ++i) {
            crc8_acc = (crc8_acc & 0x80)
                  ? (crc8_acc << 1) ^ 0x07
                  : (crc8_acc << 1);
        }
    }

uint8_t SerialSender::crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x80)
                  ? (crc << 1) ^ 0x07
                  : (crc << 1);
        }
    }
    return crc;
}

void SerialSender::writeUint16LE(uint8_t* buffer, uint16_t value) {
    buffer[0] = (uint8_t)(value & 0xFF);
    buffer[1] = (uint8_t)((value >> 8) & 0xFF);
}

void SerialSender::send_packet(uint8_t cmd, const uint8_t* payload, uint16_t length) {
    size_t index = 0;
    packet_buffer[index++] = cmd;
    writeUint16LE(&packet_buffer[index], length);
    index += 2;
    for (uint32_t i=0; i<length; i++) {
        packet_buffer[index++] = payload[i];
    }
    uint8_t crc = crc8(packet_buffer, index);
    packet_buffer[index++] = crc;
    escape_packet(packet_buffer, index);
    Serial.write(START_BYTE);
    Serial.write(escape_buffer, escape_buffer_index);
}

void SerialSender::escape_packet(uint8_t* data, size_t length) {
    escape_buffer_index = 0;
    for (size_t i = 0; i<length; i++) {
        uint8_t b = data[i];
        if (b == START_BYTE || b == ESCAPE_BYTE) {
            if (escape_buffer_index + 2 > sizeof(escape_buffer)) break;  // prevent overflow
            escape_buffer[escape_buffer_index++] = ESCAPE_BYTE;
            escape_buffer[escape_buffer_index++] = b ^ ESCAPE_MASK;
        } else {
            if (escape_buffer_index + 1 > sizeof(escape_buffer)) break;
            escape_buffer[escape_buffer_index++] = b;
        }
    }
}

void SerialParser::parse(uint8_t byte) {

        if (byte == START_BYTE) {
            reset();
            state = ParserState::READ_CMD;
            return;
        }

        if (state != ParserState::WAIT_START) {
            if (escape_next) {
                byte ^= ESCAPE_MASK;
                escape_next = false;
            } else if (byte == ESCAPE_BYTE) {
                escape_next = true;
                return;
            }
        }

        switch (state) {
            case ParserState::READ_CMD:
                cmd = byte;
                update_crc8(byte);
                state = ParserState::READ_LEN;
                break;

            case ParserState::READ_LEN:
                if (len_bytes_read == 0) {
                    len = byte;
                    update_crc8(byte);
                    len_bytes_read = 1;
                } else {
                    len |= (byte << 8);
                    update_crc8(byte);
                    len_bytes_read = 0;

                    if (len > 0 && len <= PAYLOAD_BUFFER_SIZE) {
                        state = ParserState::READ_PAYLOAD;
                    } else if (len == 0) {
                        state = ParserState::READ_CHECKSUM;
                    } else {
                        Serial.println("Payload too large!");
                        reset();
                    }
                }
                break;

            
            case ParserState::READ_PAYLOAD:
                update_crc8(byte);
                if (payload_len < PAYLOAD_BUFFER_SIZE) payload[payload_len++] = byte;
                if (payload_len >= len) {
                   state = ParserState::READ_CHECKSUM;
                }
                break;

            case ParserState::READ_CHECKSUM:
                checksum = byte;
                state = ParserState::WAIT_START;
                validate();
                break;

            default:
                reset();
        }
    }

void SerialParser::validate() {
        if (crc8_acc == checksum) {
            dispatch();
        } else {
            Serial.println("Checksum failed!");
        }
    }

void SerialParser::reset() {
        state = ParserState::WAIT_START;
        payload_len = 0;
        len = 0;
        len_bytes_read = 0;
        crc8_acc = 0x00;
        escape_next = false;
    }

void SerialParser::dispatch() {
    if (on_dispatch) {
        on_dispatch(cmd, payload, payload_len);
    }
}