#ifndef CLOSEDLOOP_H
#define CLOSEDLOOP_H

#include <Arduino.h>
#include <AS5600/AS5600.h>
#include <Stepper/Stepper.h>
#include <Trajectory/Trajectory.h>
#include <Macros.h>

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
        Trajectory **trajectory;
    };

    State state;

    float hermite_interpolate(const Waypoint &wp1,
                              const Waypoint &wp2,
                              unsigned long elapsed);

    float hermite_velocity(const Waypoint &wp1,
                           const Waypoint &wp2,
                           unsigned long elapsed);

    void control_loop_task(void *param);

    void home(Stepper &stepper, AS5600 &encoder);

    void execute_trajectory_segment(Waypoint &wp1, Waypoint &wp2, AS5600 &encoder, Stepper &stepper);

    void execute_trajectory(Trajectory *trajectory, AS5600 &encoder, Stepper &stepper);

    void move_to(float position, AS5600 &encoder, Stepper &stepper);
}

#endif
