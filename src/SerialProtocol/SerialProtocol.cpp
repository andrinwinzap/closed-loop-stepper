#include "SerialProtocol.h"

void crc8(uint8_t& crc8, uint8_t byte) {
        crc8 ^= byte;
        for (int i = 0; i < 8; ++i) {
            crc8 = (crc8 & 0x80)
                  ? (crc8 << 1) ^ 0x07
                  : (crc8 << 1);
        }
    }

uint8_t crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
        crc8(crc, data[i]);
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
    packet_buffer[index] = crc8(packet_buffer, index++);
    escape_packet(packet_buffer, index);
    Serial.write(START_BYTE);
    Serial.write(escape_buffer, escape_buffer_index);
}

void SerialSender::escape_packet(uint8_t* data, size_t length) {
    escape_buffer_index = 0;
    for (size_t i = 0; i<length; i++) {
        uint8_t b = data[i];
        if (b == START_BYTE || b == ESCAPE_BYTE) {
            if (escape_buffer_index + 2 > MAX_PACKET_BUFFER_SIZE*2) break;  // prevent overflow
            escape_buffer[escape_buffer_index++] = ESCAPE_BYTE;
            escape_buffer[escape_buffer_index++] = b ^ ESCAPE_MASK;
        } else {
            if (escape_buffer_index + 1 > MAX_PACKET_BUFFER_SIZE*2) break;
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
            if (escapeNext) {
                byte ^= ESCAPE_MASK;
                escapeNext = false;
            } else if (byte == ESCAPE_BYTE) {
                escapeNext = true;
                return;
            }
        }

        switch (state) {
            case ParserState::READ_CMD:
                cmd = byte;
                crc8(crc8_acc, byte);
                state = ParserState::READ_LEN;
                break;

            case ParserState::READ_LEN:
                if (len_bytes_read == 0) {
                    len = byte;
                    crc8(crc8_acc, byte);
                    len_bytes_read = 1;
                } else {
                    len |= (byte << 8);
                    crc8(crc8_acc, byte);
                    len_bytes_read = 0;
                    payload_bytes_read = 0;

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
                crc8(crc8_acc, byte);
                if (payload_buffer_len < PAYLOAD_BUFFER_SIZE) payload_buffer[payload_buffer_len++] = byte;
                payload_bytes_read++;
                if (payload_bytes_read >= len) {
                   state = ParserState::READ_CHECKSUM;
                }
                break;

            case ParserState::READ_CHECKSUM:
                checksum = byte;
                ParserState::VALIDATE;
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
        payload_buffer_len = 0;
        payload_bytes_read = 0;
        len = 0;
        len_bytes_read = 0;
        crc8_acc = 0x00;
        escapeNext = false;
    }

void SerialParser::dispatch() {
    if (on_dispatch) {
        on_dispatch(cmd, payload_buffer, payload_buffer_len);
    }
}