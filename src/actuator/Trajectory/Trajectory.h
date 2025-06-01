#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>
#include <Serialization/Serialization.h>

#define MAX_WAYPOINTS 64

struct Waypoint
{
    float position;     // 4 bytes
    float velocity;     // 4 bytes
    uint32_t timestamp; // 4 bytes
};

struct ActuatorTrajectory
{
    Waypoint waypoints[MAX_WAYPOINTS];
    size_t length; // usually 2 or 4 bytes (depends on platform)

    ActuatorTrajectory() : length(0) {};
    ActuatorTrajectory(const Waypoint *wps, size_t count);
    ActuatorTrajectory(const uint8_t *data, size_t len);

    size_t serialize(uint8_t *outBuffer, size_t maxLen);
};

struct RobotTrajectory
{
    // NOT IMPLEMENTED
};

#endif