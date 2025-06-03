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
        HOME,
        TRAJECTORY
    };

    enum class State
    {
        IDLE,
        HOMING,
        TRAJECTORY
    };

    struct Params
    {
        Stepper *stepper;
        AS5600 *encoder;
        volatile State *state;
        volatile Flag *flag;
        ActuatorTrajectory **trajectory;
    };

    void task(void *param);

    void home(Stepper &stepper, AS5600 &encoder);

    void move_to(float position, AS5600 &encoder, Stepper &stepper);
}

#endif
