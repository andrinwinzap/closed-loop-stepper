#include "AS5600.h"

AS5600::AS5600(TwoWire &wirePort, uint8_t address)
    : _wire(&wirePort), _address(address) {}

bool AS5600::begin() {
    _wire->begin();
    _lastUpdate = millis();
    _lastPosition = getPosition();
    return true;
}

uint8_t AS5600::read8(uint8_t reg) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_address, (uint8_t)1);
    if (_wire->available()) {
        return _wire->read();
    }
    return 0;
}

uint16_t AS5600::read12bit(uint8_t regHigh) {
    _wire->beginTransmission(_address);
    _wire->write(regHigh);
    _wire->endTransmission(false);
    _wire->requestFrom(_address, (uint8_t)2);
    if (_wire->available() < 2) {
        return 0;
    }
    uint8_t high = _wire->read();
    uint8_t low = _wire->read();
    return ((high << 8) | low) & 0x0FFF;
}

uint16_t AS5600::getRawPosition() {
    return read12bit(0x0E);
}

float AS5600::rawToRadians(uint16_t raw) {
    return (raw * 2*PI) / 4096.0;
}

float AS5600::getPosition() {
    return rawToRadians(getRawPosition());
}

void AS5600::update() {
    float current = getPosition();
    unsigned long now = millis();
    float delta = current - _lastPosition;

    // Handle rollover
    if (delta > PI) {
        delta -= 2*PI;
    } else if (delta < -PI) {
        delta += 2*PI;
    }

    _cumulativePosition += delta;

    float dt = (now - _lastUpdate) / 1000.0;  // seconds
    if (dt > 0) {
        _speed = delta / dt;
    }

    _lastPosition = current;
    _lastUpdate = now;
}

float AS5600::getCumulativePosition() {
    update();
    return _cumulativePosition;
}

float AS5600::getSpeed() {
    return _speed;
}

bool AS5600::magnetDetected() {
    uint8_t status = read8(0x0B);
    return (status & (1 << 5)) != 0;
}

void AS5600::setCumulativePosition(float angle) {
    _cumulativePosition = angle;
    _lastPosition = getPosition();
}

