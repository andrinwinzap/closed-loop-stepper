#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>
#include <AS5600/AS5600.h>
#include <Stepper/Stepper.h>

struct Waypoint {
  float position;             // radians
  float velocity;             // radians/second
  unsigned long timestamp;   // milliseconds
};

float hermiteInterpolate(const Waypoint &wp1,
                         const Waypoint &wp2,
                         unsigned long elapsed);

float hermiteVelocity(const Waypoint &wp1,
                      const Waypoint &wp2,
                      unsigned long elapsed);

void execute_trajectory_segment(const Waypoint &wp1, const Waypoint &wp2, AS5600 &encoder, Stepper &stepper, float gear_ratio);

void execute_trajectory(const Waypoint* arr, size_t length, AS5600 &encoder, Stepper &stepper, float gear_ratio);

void move_to(float position, AS5600 &encoder, Stepper &stepper, float gear_ratio);

#endif
