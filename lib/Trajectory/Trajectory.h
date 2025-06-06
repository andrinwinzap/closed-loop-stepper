#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>
#include <Serialization.h>

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

    size_t serialize(uint8_t *outBuffer, size_t maxLen) const;
};

class RobotTrajectory
{
public:
    ActuatorTrajectory actuator_1;
    ActuatorTrajectory actuator_2;
    ActuatorTrajectory actuator_3;
    ActuatorTrajectory actuator_4;
    ActuatorTrajectory actuator_5;
    ActuatorTrajectory actuator_6;

    RobotTrajectory() = default;

    // Constructor from serialized data
    RobotTrajectory(const uint8_t *data, size_t len);

    // Constructor from six actuator trajectories
    RobotTrajectory(const ActuatorTrajectory &a1,
                    const ActuatorTrajectory &a2,
                    const ActuatorTrajectory &a3,
                    const ActuatorTrajectory &a4,
                    const ActuatorTrajectory &a5,
                    const ActuatorTrajectory &a6);

    size_t serialize(uint8_t *outBuffer, size_t maxLen) const;
};

struct RobotPosition
{
    float theta_1;
    float theta_2;
    float theta_3;
    float theta_4;
    float theta_5;
    float theta_6;
};

float hermite_interpolate(const Waypoint &wp1,
                          const Waypoint &wp2,
                          unsigned long elapsed);

float hermite_velocity(const Waypoint &wp1,
                       const Waypoint &wp2,
                       unsigned long elapsed);

#endif