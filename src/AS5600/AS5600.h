#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>

class AS5600 {
public:
    AS5600(TwoWire &wirePort = Wire, uint8_t address = 0x36);
    bool begin();
    void update();  // call regularly to update angle and speed

    float getRadians();           // current angle in radians
    float getCumulativeAngle();   // angle including rollover
    void setCumulativeAngle(float angle); // set cumulative angle
    float getVelocity();             // speed in rad/s
    bool magnetDetected();

private:
    TwoWire *_wire;
    uint8_t _address;

    float _lastAngle = 0.0;
    float _cumulativeAngle = 0.0;
    float _velocity = 0.0;
    unsigned long _lastUpdate = 0;

    uint8_t read8(uint8_t reg);
    uint16_t read12bit(uint8_t regHigh);
    uint16_t getRawAngle();
    float rawToRadians(uint16_t raw);
};

#endif
