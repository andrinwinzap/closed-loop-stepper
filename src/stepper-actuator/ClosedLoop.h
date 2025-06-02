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
        EXECUTE_TRAJECTORY
    };

    enum class State
    {
        IDLE,
        HOMING,
        EXECUTING_TRAJECTORY
    };

    struct Params
    {
        AS5600 *encoder;
        Stepper *stepper;
        volatile Flag *flag;
        ActuatorTrajectory **trajectory;
    };

    extern State state;

    void control_loop_task(void *param);

    void home(Stepper &stepper, AS5600 &encoder);

    void execute_trajectory_segment(Waypoint &wp1, Waypoint &wp2, AS5600 &encoder, Stepper &stepper);

    void execute_trajectory(ActuatorTrajectory *trajectory, AS5600 &encoder, Stepper &stepper);

    void move_to(float position, AS5600 &encoder, Stepper &stepper);
}

#endif
