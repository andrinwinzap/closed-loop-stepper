#ifndef CLOSEDLOOP_H
#define CLOSEDLOOP_H

#include <Arduino.h>
#include <AS5600.h>
#include <Stepper.h>
#include <Trajectory.h>
#include <debug_macro.h>
#include <actuator_configuration_macro.h>

namespace ControlLoop
{
    enum class Flag
    {
        NOTHING,
        IDLE,
        POSITION,
        TRAJECTORY
    };

    enum class State
    {
        IDLE,
        HOMING,
        POSITION,
        TRAJECTORY
    };

    enum class HomingState
    {
        WAITING_FOR_HOMING,
        PREPARING,
        CW_EDGE,
        CCW_EDGE,
        HOMED
    };

    struct Params
    {
        Stepper *stepper;
        AS5600 *encoder;
        volatile State *state;
        volatile Flag *flag;
        ActuatorTrajectory **trajectory;
        float *target_position;
    };

    struct TrajectoryContext
    {
        size_t segment_index = 0;
        unsigned long segment_start;
        Waypoint *wp1 = nullptr;
        Waypoint *wp2 = nullptr;
    };

    void task(void *param);
}

#endif
