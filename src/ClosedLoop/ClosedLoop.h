#ifndef CLOSEDLOOP_H
#define CLOSEDLOOP_H

#include <Arduino.h>
#include <AS5600/AS5600.h>
#include <Stepper/Stepper.h>
#include <macros.h>

struct Waypoint
{
  float position;          // radians
  float velocity;          // radians/second
  unsigned long timestamp; // milliseconds
};

float hermiteInterpolate(const Waypoint &wp1,
                         const Waypoint &wp2,
                         unsigned long elapsed);

float hermiteVelocity(const Waypoint &wp1,
                      const Waypoint &wp2,
                      unsigned long elapsed);

void execute_trajectory_segment(Waypoint &wp1, Waypoint &wp2, AS5600 &encoder, Stepper &stepper);

void execute_trajectory(Waypoint *arr, size_t length, AS5600 &encoder, Stepper &stepper);

void move_to(float position, AS5600 &encoder, Stepper &stepper);

#endif
