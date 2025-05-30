#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <Arduino.h>

uint16_t readUint16LE(const uint8_t* buffer);
uint32_t readUint32LE(const uint8_t* buffer);
float readFloatLE(const uint8_t* buffer);

void writeUint16LE(uint8_t* buffer, uint16_t value);
void writeUint32LE(uint8_t* buffer, uint32_t value);
void writeFloatLE(uint8_t* buffer, float value);

#endif