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

    struct TrajectoryContext
    {
        size_t segment_index = 0;
        unsigned long segment_start;
        Waypoint *wp1 = nullptr;
        Waypoint *wp2 = nullptr;
        float filtered_vel;
        float last_control_speed = 0;
        unsigned long stall_start_time = 0;
    };

    extern State state;

    void control_loop_task(void *param);

    void home(Stepper &stepper, AS5600 &encoder);

    void move_to(float position, AS5600 &encoder, Stepper &stepper);
}

#endif
