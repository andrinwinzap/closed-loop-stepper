#include <AS5600.h>

AS5600::AS5600(float gear_ratio, TwoWire &wirePort, uint8_t address)
    : _gear_ratio(gear_ratio), _wire(&wirePort), _address(address) {}

bool AS5600::begin()
{
    _wire->begin();

    uint8_t status = read8(0x0B);

    if (status == 0xFF || (status & (1 << 5)) == 0)
    {
        return false;
    }

    _lastUpdate = millis();
    _lastRawAngle = getRawAngle();
    return true;
}

uint8_t AS5600::read8(uint8_t reg)
{
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0)
        return 0xFF;

    if (_wire->requestFrom(_address, (uint8_t)1) != 1)
        return 0xFF;

    return _wire->read();
}

uint16_t AS5600::read12bit(uint8_t regHigh)
{
    _wire->beginTransmission(_address);
    _wire->write(regHigh);
    if (_wire->endTransmission(false) != 0)
        return 0;

    if (_wire->requestFrom(_address, (uint8_t)2) != 2)
        return 0;

    uint8_t high = _wire->read();
    uint8_t low = _wire->read();
    return ((high << 8) | low) & 0x0FFF;
}

float AS5600::getRawAngle()
{
    return (read12bit(0x0E) * 2 * PI) / 4096.0;
}

void AS5600::update()
{
    float current = getRawAngle();
    unsigned long now = millis();
    float delta = current - _lastRawAngle;

    if (delta > PI)
        delta -= 2 * PI;
    else if (delta < -PI)
        delta += 2 * PI;

    _position += delta;
    float dt = (now - _lastUpdate) / 1000.0;

    if (dt > 0)
        _speed = delta / dt;

    _lastRawAngle = current;
    _lastUpdate = now;
}

float AS5600::getPosition()
{
    return _position / _gear_ratio;
}

float AS5600::getSpeed()
{
    return _speed / _gear_ratio;
}

bool AS5600::magnetDetected()
{
    uint8_t status = read8(0x0B);
    return (status & (1 << 5)) != 0;
}

void AS5600::setPosition(float angle)
{
    _position = angle * _gear_ratio;
    _lastRawAngle = getRawAngle();
}