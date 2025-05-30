#include "SerialProtocol.h"

void SerialProtocol::parse_serial() {
    while (serial_port.available()) {
        parser.parse(serial_port.read());
    }
}

size_t SerialProtocol::available() {
    parse_serial();
    return parser.available();
}

const Command* SerialProtocol::read() {
    return parser.read();
}

void SerialParser::update_crc8(uint8_t byte) {
    crc8_acc ^= byte;
    for (int i = 0; i < 8; ++i) {
        crc8_acc = (crc8_acc & 0x80)
                ? (crc8_acc << 1) ^ CRC8_POLY
                : (crc8_acc << 1);
    }
}

uint8_t SerialProtocol::crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x80)
                  ? (crc << 1) ^ CRC8_POLY
                  : (crc << 1);
        }
    }
    return crc;
}

void SerialProtocol::send_packet(uint8_t cmd, const uint8_t* payload, uint16_t payload_len) {
    size_t index = 0;
    packet[index++] = cmd;
    writeUint16LE(&packet[index], payload_len);
    index += 2;

    if (payload != nullptr && payload_len > 0) {
        for (uint32_t i = 0; i < payload_len; i++) {
            packet[index++] = payload[i];
        }
    }

    uint8_t crc = crc8(packet, index);
    packet[index++] = crc;

    uint8_t escaped_packet[MAX_ESCAPED_PACKET_SIZE];
    size_t escaped_packet_len = escape_packet(packet, index, escaped_packet);
    serial_port.write(START_BYTE);
    serial_port.write(escaped_packet, escaped_packet_len);
}

size_t SerialProtocol::escape_packet(const uint8_t* data, size_t len, uint8_t* escaped_packet) const {
    size_t index = 0;
    for (size_t i = 0; i<len; i++) {
        uint8_t b = data[i];
        if (b == START_BYTE || b == ESCAPE_BYTE) {
            if (index + 1 >= MAX_ESCAPED_PACKET_SIZE) break;  // prevent overflow
            escaped_packet[index++] = ESCAPE_BYTE;
            escaped_packet[index++] = b ^ ESCAPE_MASK;
        } else {
            if (index >= MAX_ESCAPED_PACKET_SIZE) break;
            escaped_packet[index++] = b;
        }
    }
    return index;
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

                if (len > 0 && len <= MAX_PAYLOAD_SIZE) {
                    state = ParserState::READ_PAYLOAD;
                } else if (len == 0) {
                    state = ParserState::READ_CHECKSUM;
                } else {
                    DBG_PRINTLN("Payload too large!");
                    reset();
                }
            }
            break;

        
        case ParserState::READ_PAYLOAD:
            update_crc8(byte);
            if (payload_len < len && payload_len < MAX_PAYLOAD_SIZE) {
                payload[payload_len++] = byte;
            }
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
        enqueue_command(cmd, payload, payload_len);
    } else {
        DBG_PRINTLN("Checksum failed!");
    }
    crc8_acc = 0x00;
}

void SerialParser::reset() {
    state = ParserState::WAIT_START;
    payload_len = 0;
    len = 0;
    len_bytes_read = 0;
    crc8_acc = 0x00;
    escape_next = false;
}

void SerialParser::enqueue_command(uint8_t cmd, const uint8_t* payload, size_t payload_len) {
    if (queue_count < CMD_QUEUE_SIZE) {
        queue[queue_tail].cmd = cmd;
        queue[queue_tail].payload_len = payload_len;
        memcpy(queue[queue_tail].payload, payload, payload_len);
        queue_tail = (queue_tail + 1) % CMD_QUEUE_SIZE;
        queue_count++;
    } else {
        DBG_PRINTLN("Command queue full, dropping command");
    }
}

const Command* SerialParser::read() {
    if (queue_count == 0) return nullptr;
    const Command* cmd = &queue[queue_head];
    queue_head = (queue_head + 1) % CMD_QUEUE_SIZE;
    queue_count--;
    return cmd;
}


