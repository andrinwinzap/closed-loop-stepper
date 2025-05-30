#include "ClosedLoop.h"

// Hermite interpolation taking elapsed time (ms) directly
float hermiteInterpolate(const Waypoint &wp1,
                         const Waypoint &wp2,
                         unsigned long elapsed)
{
  float dt_ms = float(wp2.timestamp - wp1.timestamp);
  if (dt_ms <= 0)
    return wp1.position;

  // Normalized parameter u in [0,1]
  float u = float(elapsed) / dt_ms;
  u = constrain(u, 0.0f, 1.0f);

  float u2 = u * u;
  float u3 = u2 * u;

  // Hermite basis
  float h00 = 2 * u3 - 3 * u2 + 1;
  float h10 = u3 - 2 * u2 + u;
  float h01 = -2 * u3 + 3 * u2;
  float h11 = u3 - u2;

  // Scale velocities over the interval
  float dt_s = dt_ms * 0.001f;
  float m0 = wp1.velocity * dt_s;
  float m1 = wp2.velocity * dt_s;

  return h00 * wp1.position + h10 * m0 + h01 * wp2.position + h11 * m1;
}

// Hermite derivative: instantaneous velocity (rad/s) from elapsed time
float hermiteVelocity(const Waypoint &wp1,
                      const Waypoint &wp2,
                      unsigned long elapsed)
{
  float dt_ms = float(wp2.timestamp - wp1.timestamp);
  if (dt_ms <= 0)
    return 0;

  float u = float(elapsed) / dt_ms;
  if (u <= 0.0f || u >= 1.0f)
    return 0;

  float u2 = u * u;

  // Basis derivatives
  float dh00 = 6 * u2 - 6 * u;
  float dh10 = 3 * u2 - 4 * u + 1;
  float dh01 = -6 * u2 + 6 * u;
  float dh11 = 3 * u2 - 2 * u;

  float dt_s = dt_ms * 0.001f;
  float m0 = wp1.velocity * dt_s;
  float m1 = wp2.velocity * dt_s;

  // d(position)/du
  float dPu = dh00 * wp1.position + dh10 * m0 + dh01 * wp2.position + dh11 * m1;

  // du/dt = 1 / dt_s
  return dPu / dt_s;
}

void execute_trajectory_segment(Waypoint &wp1, Waypoint &wp2, AS5600 &encoder, Stepper &stepper)
{
  DBG_PRINT("Waypoint | Position: ");
  DBG_PRINT(wp1.position);
  DBG_PRINT(" | Speed: ");
  DBG_PRINT(wp1.velocity);
  DBG_PRINT(" | Timestamp: ");
  DBG_PRINTLN(wp1.timestamp);

  unsigned long now = millis();
  unsigned long motion_segment_start_time = now;

  unsigned long last_control_loop_timestamp = now;
  unsigned long last_serial_print_timestamp = now;
  float last_control_speed = 0;
  float filtered_vel = encoder.getSpeed();

  bool stalled = false;
  unsigned long stall_start_time = 0;
  Waypoint new_start = wp1;

  unsigned long motion_duration = wp2.timestamp - wp1.timestamp;

  stepper.start();

  while (true)
  {
    unsigned long now = millis();
    unsigned long elapsed = now - motion_segment_start_time;

    if (elapsed >= motion_duration)
    {
      stepper.setSpeed(wp2.velocity);
      break;
    }

    if (now - last_control_loop_timestamp >= CONTROL_LOOP_INTERVAL)
    {

      float desired_pos = hermiteInterpolate(wp1, wp2, elapsed);
      float desired_vel = hermiteVelocity(wp1, wp2, elapsed);

      float measured_pos = encoder.getPosition();
      float measured_vel = encoder.getSpeed();
      filtered_vel = CONTROL_LOOP_DEBUG_INTERVAL * measured_vel + (1 - CONTROL_LOOP_DEBUG_INTERVAL) * filtered_vel;

      float pos_error = desired_pos - measured_pos;
      float vel_error = desired_vel - filtered_vel;

      if (filtered_vel < STALL_VELOCITY_THRESHOLD && pos_error > STALL_POSITION_ERROR_THRESHOLD)
      {
        if (stall_start_time == 0)
        {
          stall_start_time = millis();
        }
        else
        {
          if (millis() - stall_start_time > STALL_TIME_THRESHOLD)
          {
            stalled = true;
            DBG_PRINTLN("STALLED!");
            new_start = {
                encoder.getPosition(),
                0.0f,
                elapsed};
            break;
          }
        }
      }
      else
      {
        stall_start_time = 0;
      }

      float control_speed = KF * desired_vel + KP * pos_error + KV * vel_error;

      if (CONTROL_LOOP_DEBUG_OUTPUT && millis() - last_serial_print_timestamp > CONTROL_LOOP_DEBUG_INTERVAL)
      {
        DBG_PRINT("Position: ");
        DBG_PRINT(measured_pos);
        DBG_PRINT(" | ");
        DBG_PRINT("Position Error: ");
        DBG_PRINT(pos_error);
        DBG_PRINT(" | ");
        DBG_PRINT("Velocity: ");
        DBG_PRINT(filtered_vel);
        DBG_PRINT(" | ");
        DBG_PRINT("Velocity Error: ");
        DBG_PRINT(vel_error);
        DBG_PRINT(" | ");
        DBG_PRINT("Control Speed: ");
        DBG_PRINTLN(control_speed);

        last_serial_print_timestamp = millis();
      }

      if (control_speed != last_control_speed)
      {
        stepper.setSpeed(control_speed);
        last_control_speed = control_speed;
      }

      last_control_loop_timestamp = now;
    }
  }

  DBG_PRINT("Motion Segment Duration: ");
  DBG_PRINTLN(millis() - motion_segment_start_time);

  if (stalled)
  {
    DBG_PRINTLN("New motion profile started");
    execute_trajectory_segment(new_start, wp2, encoder, stepper);
  }

  wp2.position = encoder.getPosition();
  wp2.velocity = encoder.getSpeed();
  DBG_PRINT("Updated Waypoint | Position: ");
  DBG_PRINT(wp2.position);
  DBG_PRINT(" | Speed: ");
  DBG_PRINT(wp2.velocity);
  DBG_PRINT(" | Timestamp: ");
  DBG_PRINTLN(wp2.timestamp);
}

void execute_trajectory(Trajectory *trajectory, AS5600 &encoder, Stepper &stepper)
{
  if (trajectory == nullptr)
    return;
  for (size_t i = 0; i < trajectory->length - 1; ++i)
  {
    execute_trajectory_segment(trajectory->waypoints[i], trajectory->waypoints[i + 1], encoder, stepper);
  }
  stepper.stop(); // Ensure motor stops
}

void move_to(float target_position, AS5600 &encoder, Stepper &stepper)
{
  stepper.start();

  unsigned long last_control_time = millis();
  float last_speed_command = 0.0f;

  while (true)
  {
    unsigned long now = millis();
    if (now - last_control_time >= CONTROL_LOOP_INTERVAL)
    {
      float dt = (now - last_control_time) * 0.001f; // convert ms to seconds

      float current_position = encoder.getPosition();
      float pos_error = target_position - current_position;

      // If within tolerance, stop and break loop
      if (fabs(pos_error) <= POSITION_TOLERANCE)
      {
        stepper.setSpeed(0);
        DBG_PRINTLN("Position reached within tolerance. Stopping.");
        break;
      }

      // Proportional control speed command
      float desired_speed = KP_MOVE_TO * pos_error;

      // Clamp desired speed to max limits
      if (desired_speed > MAX_SPEED)
        desired_speed = MAX_SPEED;
      else if (desired_speed < -MAX_SPEED)
        desired_speed = -MAX_SPEED;

      // Limit acceleration
      float max_speed_change = MAX_ACCELERATION * dt;
      float speed_diff = desired_speed - last_speed_command;

      if (speed_diff > max_speed_change)
      {
        desired_speed = last_speed_command + max_speed_change;
        speed_diff = max_speed_change;
      }
      else if (speed_diff < -max_speed_change)
      {
        desired_speed = last_speed_command - max_speed_change;
        speed_diff = -max_speed_change;
      }

      // Apply speed command scaled by gear ratio
      stepper.setSpeed(desired_speed);

      // Debug output
      DBG_PRINT("Pos Error: ");
      DBG_PRINT(pos_error, 4);
      DBG_PRINT(" | Speed Cmd: ");
      DBG_PRINT(desired_speed, 4);
      DBG_PRINT(" | Accel Applied: ");
      DBG_PRINTLN(speed_diff / dt, 4); // acceleration in units/sec²

      last_speed_command = desired_speed;
      last_control_time = now;
    }
  }

  stepper.stop();
}
