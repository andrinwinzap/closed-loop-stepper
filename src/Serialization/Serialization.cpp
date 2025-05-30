#include "Serialization.h"

uint16_t readUint16LE(const uint8_t *buffer)
{
    uint16_t val;
    memcpy(&val, buffer, 2);
    return val;
}

uint32_t readUint32LE(const uint8_t *buffer)
{
    uint32_t val;
    memcpy(&val, buffer, 4);
    return val;
}

float readFloatLE(const uint8_t *buffer)
{
    float val;
    memcpy(&val, buffer, 4);
    return val;
}

void writeUint16LE(uint8_t *buffer, uint16_t value)
{
    memcpy(buffer, &value, 2);
}

void writeUint32LE(uint8_t *buffer, uint32_t value)
{
    memcpy(buffer, &value, 4);
}

void writeFloatLE(uint8_t *buffer, float value)
{
    memcpy(buffer, &value, 4);
}