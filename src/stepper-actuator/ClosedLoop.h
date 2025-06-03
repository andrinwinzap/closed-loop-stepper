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

    struct Params
    {
        Stepper *stepper;
        AS5600 *encoder;
        volatile State *state;
        volatile Flag *flag;
        ActuatorTrajectory **trajectory;
        float *target_position;
    };

    void task(void *param);
}

#endif
