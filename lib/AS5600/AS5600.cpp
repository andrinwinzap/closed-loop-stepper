#include <AS5600.h>

AS5600 *AS5600::_instance = nullptr;

AS5600::AS5600(float gear_ratio, TwoWire &wirePort, uint8_t address)
    : _gear_ratio(gear_ratio), _wire(&wirePort), _address(address) {}

bool AS5600::begin()
{
    _wire->begin();
    _lastUpdate = millis();
    _lastRawAngle = getRawAngle();

    _instance = this;

    _timer = timerBegin(1, 80, true); // prescaler 80 = 1 tick = 1 us
    timerAttachInterrupt(_timer, &AS5600::onTimerISR, true);

    uint64_t ticks = (uint64_t)(1000000.0f / _updateFrequencyHz); // microseconds per tick
    timerAlarmWrite(_timer, ticks, true);
    timerAlarmEnable(_timer);

    return true;
}

void IRAM_ATTR AS5600::onTimerISR()
{
    if (_instance)
    {
        portENTER_CRITICAL_ISR(&_instance->_timerMux);
        _instance->setUpdateFlag();
        portEXIT_CRITICAL_ISR(&_instance->_timerMux);
    }
}

void AS5600::setUpdateFlag()
{
    _updateFlag = true;
}

uint8_t AS5600::read8(uint8_t reg)
{
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_address, (uint8_t)1);
    if (_wire->available())
    {
        return _wire->read();
    }
    return 0;
}

uint16_t AS5600::read12bit(uint8_t regHigh)
{
    _wire->beginTransmission(_address);
    _wire->write(regHigh);
    _wire->endTransmission(false);
    _wire->requestFrom(_address, (uint8_t)2);
    if (_wire->available() < 2)
    {
        return 0;
    }
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
    bool shouldUpdate = false;

    portENTER_CRITICAL(&_timerMux);
    if (_updateFlag)
    {
        shouldUpdate = true;
        _updateFlag = false;
    }
    portEXIT_CRITICAL(&_timerMux);

    if (!shouldUpdate)
        return;

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
    update();
    return _position / _gear_ratio;
}

float AS5600::getSpeed()
{
    update();
    return _speed / _gear_ratio;
}

bool AS5600::magnetDetected()
{
    uint8_t status = read8(0x0B);
    return (status & (1 << 5)) != 0;
}

void AS5600::setPosition(float angle)
{
    _position = angle;
    _lastRawAngle = getRawAngle();
}

void AS5600::setUpdateFrequency(float freqHz)
{
    if (freqHz <= 0)
        return; // ignore invalid frequencies
    _updateFrequencyHz = freqHz;

    if (_timer)
    {
        timerAlarmDisable(_timer);
        uint64_t ticks = (uint64_t)(1000000.0f / _updateFrequencyHz); // microseconds per tick
        timerAlarmWrite(_timer, ticks, true);
        timerAlarmEnable(_timer);
    }
}