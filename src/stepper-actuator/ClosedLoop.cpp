#include "ClosedLoop.h"

namespace ControlLoop
{
  State state = State::IDLE;
  TrajectoryContext trajectory_context;

  void control_loop_task(void *param)
  {
    Params *params = static_cast<Params *>(param);
    AS5600 &encoder = *params->encoder;
    Stepper &stepper = *params->stepper;
    volatile Flag &flag = *params->flag;
    ActuatorTrajectory **trajectory = params->trajectory;

    for (;;)
    {
      unsigned long now = millis();

      if (flag != Flag::NOTHING)
      {
        switch (flag)
        {
        case Flag::IDLE:
          state = State::IDLE;
          flag = Flag::NOTHING;
          break;

        case Flag::HOME:
          state = State::HOMING;
          flag = Flag::NOTHING;
          break;

        case Flag::EXECUTE_TRAJECTORY:
          if (trajectory != nullptr && *trajectory != nullptr)
          {
            state = State::EXECUTING_TRAJECTORY;
            trajectory_context = TrajectoryContext{}; // Reset context
            trajectory_context.wp1 = &(*trajectory)->waypoints[0];
            trajectory_context.wp2 = &(*trajectory)->waypoints[1];
            trajectory_context.filtered_vel = encoder.getSpeed();
            trajectory_context.segment_start = now;
            stepper.start();
            DBG_PRINTLN("[CONTROL] Start trajectory execution");
          }
          flag = Flag::NOTHING;
          break;

        default:
          break;
        }
      }

      switch (state)
      {
      case State::EXECUTING_TRAJECTORY:
      {
        unsigned long delta_time = now - trajectory_context.segment_start;

        if (delta_time >= trajectory_context.wp2->timestamp - trajectory_context.wp1->timestamp)
        {
          trajectory_context.segment_index++;
          if (trajectory_context.segment_index >= (*trajectory)->length - 1)
          {
            stepper.stop();
            state = State::IDLE;
            DBG_PRINTLN("[CONTROL] Finished trajectory");
            continue;
          }
          else
          {
            (*trajectory)->waypoints[trajectory_context.segment_index].position = encoder.getPosition();
            (*trajectory)->waypoints[trajectory_context.segment_index].velocity = encoder.getSpeed();
            trajectory_context.wp1 = &(*trajectory)->waypoints[trajectory_context.segment_index];
            trajectory_context.wp2 = &(*trajectory)->waypoints[trajectory_context.segment_index + 1];
            trajectory_context.segment_start = now;
          }
          break;
        }

        float desired_pos = hermite_interpolate(*trajectory_context.wp1, *trajectory_context.wp2, delta_time);
        float desired_vel = hermite_velocity(*trajectory_context.wp1, *trajectory_context.wp2, delta_time);

        float measured_pos = encoder.getPosition();
        float measured_vel = encoder.getSpeed();
        trajectory_context.filtered_vel = VELOCITY_FILTER_APHA * measured_vel + (1 - VELOCITY_FILTER_APHA) * trajectory_context.filtered_vel;

        float pos_error = desired_pos - measured_pos;
        float vel_error = desired_vel - trajectory_context.filtered_vel;

        if (trajectory_context.filtered_vel < STALL_VELOCITY_THRESHOLD && pos_error > STALL_POSITION_ERROR_THRESHOLD)
        {
          if (trajectory_context.stall_start_time == 0)
          {
            trajectory_context.stall_start_time = millis();
          }
          else if (millis() - trajectory_context.stall_start_time > STALL_TIME_THRESHOLD)
          {
            DBG_PRINT("[CONTROL][TRAJ][STALL] PosErr=");
            DBG_PRINT(pos_error, 4);
            DBG_PRINT("\tFilteredVel=");
            DBG_PRINTLN(trajectory_context.filtered_vel, 4);

            *trajectory_context.wp1 = {
                encoder.getPosition(),
                0.0f,
                trajectory_context.wp1->timestamp + delta_time};
            trajectory_context.segment_start = now;
            break;
          }
        }
        else
        {
          trajectory_context.stall_start_time = 0;
        }

        float control_speed = KF * desired_vel + KP * pos_error + KV * vel_error;

        if (control_speed != trajectory_context.last_control_speed)
        {
          stepper.setSpeed(control_speed);
          trajectory_context.last_control_speed = control_speed;
        }
      }
      break;

      default:
        break;
      }

      encoder.update();
      vTaskDelay(pdMS_TO_TICKS(1)); // Run at ~1kHz
    }
  }

  void execute_trajectory_segment(Waypoint &wp1, Waypoint &wp2, AS5600 &encoder, Stepper &stepper)
  {
    DBG_PRINT("[CONTROL][TRAJ] Start Waypoint | Pos: ");
    DBG_PRINT(wp1.position, 4);
    DBG_PRINT(" | Vel: ");
    DBG_PRINT(wp1.velocity, 4);
    DBG_PRINT(" | Time: ");
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
        float desired_pos = hermite_interpolate(wp1, wp2, elapsed);
        float desired_vel = hermite_velocity(wp1, wp2, elapsed);

        float measured_pos = encoder.getPosition();
        float measured_vel = encoder.getSpeed();
        filtered_vel = VELOCITY_FILTER_APHA * measured_vel + (1 - VELOCITY_FILTER_APHA) * filtered_vel;

        float pos_error = desired_pos - measured_pos;
        float vel_error = desired_vel - filtered_vel;

        if (filtered_vel < STALL_VELOCITY_THRESHOLD && pos_error > STALL_POSITION_ERROR_THRESHOLD)
        {
          if (stall_start_time == 0)
          {
            stall_start_time = millis();
          }
          else if (millis() - stall_start_time > STALL_TIME_THRESHOLD)
          {
            stalled = true;
            DBG_PRINT("[CONTROL][TRAJ][STALL] PosErr=");
            DBG_PRINT(pos_error, 4);
            DBG_PRINT("\tFilteredVel=");
            DBG_PRINTLN(filtered_vel, 4);

            new_start = {
                encoder.getPosition(),
                0.0f,
                elapsed};
            break;
          }
        }
        else
        {
          stall_start_time = 0;
        }

        float control_speed = KF * desired_vel + KP * pos_error + KV * vel_error;

#if CONTROL_LOOP_DEBUG_OUTPUT
        if (millis() - last_serial_print_timestamp > CONTROL_LOOP_DEBUG_INTERVAL)
        {
          DBG_PRINT("[CONTROL] t=");
          DBG_PRINT(elapsed);
          DBG_PRINT("ms\tPos=");
          DBG_PRINT(measured_pos, 4);
          DBG_PRINT("\tDesPos=");
          DBG_PRINT(desired_pos, 4);
          DBG_PRINT("\tErr=");
          DBG_PRINT(pos_error, 4);
          DBG_PRINT("\tVel=");
          DBG_PRINT(filtered_vel, 4);
          DBG_PRINT("\tDesVel=");
          DBG_PRINT(desired_vel, 4);
          DBG_PRINT("\tVelErr=");
          DBG_PRINT(vel_error, 4);
          DBG_PRINT("\tCmdSpeed=");
          DBG_PRINTLN(control_speed, 4);

          last_serial_print_timestamp = millis();
        }
#endif

        if (control_speed != last_control_speed)
        {
          stepper.setSpeed(control_speed);
          last_control_speed = control_speed;
        }

        last_control_loop_timestamp = now;
      }
    }

    DBG_PRINT("[CONTROL][TRAJ] Segment Duration: ");
    DBG_PRINTLN(millis() - motion_segment_start_time);

    if (stalled)
    {
      DBG_PRINTLN("[CONTROL][TRAJ] Stalled. Restarting segment from current position.");
      execute_trajectory_segment(new_start, wp2, encoder, stepper);
    }

    wp2.position = encoder.getPosition();
    wp2.velocity = encoder.getSpeed();
    DBG_PRINT("[CONTROL][TRAJ] Updated End Waypoint | Pos: ");
    DBG_PRINT(wp2.position, 4);
    DBG_PRINT(" | Vel: ");
    DBG_PRINT(wp2.velocity, 4);
    DBG_PRINT(" | Time: ");
    DBG_PRINTLN(wp2.timestamp);
  }

  void execute_trajectory(ActuatorTrajectory *trajectory, AS5600 &encoder, Stepper &stepper)
  {
    if (trajectory == nullptr)
      return;

    for (size_t i = 0; i < trajectory->length - 1; ++i)
    {
      DBG_PRINT("[CONTROL][TRAJ] Executing Segment ");
      DBG_PRINTLN(i);
      execute_trajectory_segment(trajectory->waypoints[i], trajectory->waypoints[i + 1], encoder, stepper);
    }

    stepper.stop();
  }

  void move_to(float target_position, AS5600 &encoder, Stepper &stepper)
  {
    stepper.start();

    unsigned long now = millis();
    unsigned long last_control_time = now;
    unsigned long last_serial_print_timestamp = now;
    float last_speed_command = 0.0f;

    while (true)
    {
      unsigned long now = millis();
      if (now - last_control_time >= CONTROL_LOOP_INTERVAL)
      {
        float dt = (now - last_control_time) * 0.001f; // ms to s

        float current_position = encoder.getPosition();
        float pos_error = target_position - current_position;

        if (fabs(pos_error) <= POSITION_TOLERANCE)
        {
          stepper.setSpeed(0);
          DBG_PRINTLN("[CONTROL][MOVETO] Target reached within tolerance.");
          break;
        }

        float desired_speed = KP_MOVE_TO * pos_error;

        if (desired_speed > MAX_SPEED)
          desired_speed = MAX_SPEED;
        else if (desired_speed < -MAX_SPEED)
          desired_speed = -MAX_SPEED;

        float max_speed_change = MAX_ACCELERATION * dt;
        float speed_diff = desired_speed - last_speed_command;

        if (speed_diff > max_speed_change)
          desired_speed = last_speed_command + max_speed_change;
        else if (speed_diff < -max_speed_change)
          desired_speed = last_speed_command - max_speed_change;

        stepper.setSpeed(desired_speed);

#if CONTROL_LOOP_DEBUG_OUTPUT
        if (millis() - last_serial_print_timestamp > CONTROL_LOOP_DEBUG_INTERVAL)
        {
          DBG_PRINT("[CONTROL][MOVETO] t=");
          DBG_PRINT(now - last_control_time);
          DBG_PRINT("ms\tPosErr=");
          DBG_PRINT(pos_error, 4);
          DBG_PRINT("\tSpeedCmd=");
          DBG_PRINT(desired_speed, 4);
          DBG_PRINT("\tAccel=");
          DBG_PRINTLN(speed_diff / dt, 4);

          last_serial_print_timestamp = millis();
        }
#endif

        last_speed_command = desired_speed;
        last_control_time = now;
      }
    }

    stepper.stop();
  }

  void home(Stepper &stepper, AS5600 &encoder)
  {
    DBG_PRINTLN("[CONTROL][HOMING] Starting homing process...");

    stepper.setSpeed(-HOMING_SPEED);
    stepper.move(-PI / 8.0, HOMING_SPEED, HOMING_ACCELERATION);
    DBG_PRINTLN("[CONTROL][HOMING] Stepper move initiated");

    float first_bound = 0;

    stepper.start();
    stepper.accelerate(0, HOMING_SPEED, HOMING_ACCELERATION);
    DBG_PRINTLN("[CONTROL][HOMING] Stepper acceleration started");

    float filteredHallSensorValue = analogRead(HALL_EFFECT_SENSOR_PIN);

    while (true)
    {
      stepper.updateAcceleration();
      int raw = analogRead(HALL_EFFECT_SENSOR_PIN);
      filteredHallSensorValue = HALL_EFFECT_SENSOR_ALPHA * raw + (1 - HALL_EFFECT_SENSOR_ALPHA) * filteredHallSensorValue;

#if PRINT_HALL_SENSOR_VALUE
      DBG_PRINT("[CONTROL][HOMING] Hall sensor value: ");
      DBG_PRINTLN(filteredHallSensorValue);
#endif

      if (filteredHallSensorValue < 1)
      {
        DBG_PRINTLN("[CONTROL][HOMING] Detected falling edge - Hall sensor triggered (first bound)");
        break;
      }

      delayMicroseconds(HALL_EFFECT_SENSOR_UPDATE_PERIOD);
    }

    first_bound = encoder.getPosition();
    DBG_PRINT("[CONTROL][HOMING] First bound position: ");
    DBG_PRINTLN(first_bound);

    while (true)
    {
      int raw = analogRead(HALL_EFFECT_SENSOR_PIN);
      filteredHallSensorValue = HALL_EFFECT_SENSOR_ALPHA * raw + (1 - HALL_EFFECT_SENSOR_ALPHA) * filteredHallSensorValue;

      if (filteredHallSensorValue > 1)
      {
        DBG_PRINTLN("[CONTROL][HOMING] Detected rising edge - leaving Hall sensor");
        break;
      }

      delayMicroseconds(HALL_EFFECT_SENSOR_UPDATE_PERIOD);
    }

    float current_pos = encoder.getPosition();
    float home = (current_pos - first_bound) / 2;

    DBG_PRINT("[CONTROL][HOMING] Current position: ");
    DBG_PRINTLN(current_pos);
    DBG_PRINT("[CONTROL][HOMING] Calculated home offset: ");
    DBG_PRINTLN(home);

    encoder.setPosition(current_pos - home);

    stepper.accelerate(HOMING_SPEED, 0, HOMING_ACCELERATION);
    while (stepper.updateAcceleration())
    {
      unsigned long start = micros();
      encoder.update();
    }

    move_to(0, encoder, stepper);

    DBG_PRINTLN("[CONTROL][HOMING] Finished homing");
    DBG_PRINT("[CONTROL][HOMING] Encoder position after homing: ");
    DBG_PRINTLN(encoder.getPosition());

    delay(1000);
  }
}