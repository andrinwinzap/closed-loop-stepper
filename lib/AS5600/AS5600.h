#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>

class AS5600
{
public:
    AS5600(float gear_ratio, TwoWire &wirePort = Wire, uint8_t address = 0x36);
    bool begin();
    void update();
    float getPosition();
    void setPosition(float position);
    float getSpeed();
    bool magnetDetected();
    void setUpdateFrequency(float freqHz);

private:
    TwoWire *_wire;
    uint8_t _address;

    float _gear_ratio;
    float _lastRawAngle = 0.0;
    float _position = 0.0;
    float _speed = 0.0;
    unsigned long _lastUpdate = 0;

    hw_timer_t *_timer = nullptr;
    portMUX_TYPE _timerMux = portMUX_INITIALIZER_UNLOCKED;

    volatile bool _updateFlag = false;
    float _updateFrequencyHz = 10000.0;

    static void IRAM_ATTR onTimerISR();
    void setUpdateFlag();

    uint8_t read8(uint8_t reg);
    uint16_t read12bit(uint8_t regHigh);
    float getRawAngle();

    static AS5600 *_instance;
};

#endif
