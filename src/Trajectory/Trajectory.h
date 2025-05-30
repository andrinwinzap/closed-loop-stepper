#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>
#include <Serialization/Serialization.h>

#define MAX_WAYPOINTS 64

struct Waypoint {
    float position;             // 4 bytes
    float velocity;             // 4 bytes
    uint32_t timestamp;         // 4 bytes
};

class Trajectory {
public:
    Waypoint waypoints[MAX_WAYPOINTS];
    size_t length;              // usually 2 or 4 bytes (depends on platform)

    Trajectory() : length(0) {};
    Trajectory(const Waypoint* wps, size_t count);
    Trajectory(const uint8_t* data, size_t len);

private:
    size_t serialize(uint8_t* outBuffer, size_t maxLen);

};

#endif