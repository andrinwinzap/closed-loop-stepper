#ifndef CLOSEDLOOP_H
#define CLOSEDLOOP_H

#include <Arduino.h>
#include <AS5600/AS5600.h>
#include <Stepper/Stepper.h>
#include <Trajectory/Trajectory.h>
#include <Macros.h>

enum class ControlLoopFlag
{
    IDLE,
    HOME,
    EXECUTE_TRAJECTORY
};

float hermiteInterpolate(const Waypoint &wp1,
                         const Waypoint &wp2,
                         unsigned long elapsed);

float hermiteVelocity(const Waypoint &wp1,
                      const Waypoint &wp2,
                      unsigned long elapsed);

void home(Stepper &stepper, AS5600 &encoder);

void execute_trajectory_segment(Waypoint &wp1, Waypoint &wp2, AS5600 &encoder, Stepper &stepper);

void execute_trajectory(Trajectory *trajectory, AS5600 &encoder, Stepper &stepper);

void move_to(float position, AS5600 &encoder, Stepper &stepper);

#endif
