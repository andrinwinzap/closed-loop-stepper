#include "ClosedLoop.h"

namespace ControlLoop
{
  State state = State::IDLE;

  // Hermite interpolation taking elapsed time (ms) directly
  float hermite_interpolate(const Waypoint &wp1, const Waypoint &wp2, unsigned long elapsed)
  {
    float dt_ms = float(wp2.timestamp - wp1.timestamp);
    if (dt_ms <= 0)
      return wp1.position;

    float u = float(elapsed) / dt_ms;
    u = constrain(u, 0.0f, 1.0f);

    float u2 = u * u;
    float u3 = u2 * u;

    float h00 = 2 * u3 - 3 * u2 + 1;
    float h10 = u3 - 2 * u2 + u;
    float h01 = -2 * u3 + 3 * u2;
    float h11 = u3 - u2;

    float dt_s = dt_ms * 0.001f;
    float m0 = wp1.velocity * dt_s;
    float m1 = wp2.velocity * dt_s;

    return h00 * wp1.position + h10 * m0 + h01 * wp2.position + h11 * m1;
  }

  float hermite_velocity(const Waypoint &wp1, const Waypoint &wp2, unsigned long elapsed)
  {
    float dt_ms = float(wp2.timestamp - wp1.timestamp);
    if (dt_ms <= 0)
      return 0;

    float u = float(elapsed) / dt_ms;
    if (u <= 0.0f || u >= 1.0f)
      return 0;

    float u2 = u * u;

    float dh00 = 6 * u2 - 6 * u;
    float dh10 = 3 * u2 - 4 * u + 1;
    float dh01 = -6 * u2 + 6 * u;
    float dh11 = 3 * u2 - 2 * u;

    float dt_s = dt_ms * 0.001f;
    float m0 = wp1.velocity * dt_s;
    float m1 = wp2.velocity * dt_s;

    float dPu = dh00 * wp1.position + dh10 * m0 + dh01 * wp2.position + dh11 * m1;

    return dPu / dt_s;
  }

  void control_loop_task(void *param)
  {
    Params *params = static_cast<Params *>(param);
    AS5600 &encoder = *params->encoder;
    Stepper &stepper = *params->stepper;
    volatile Flag &flag = *params->flag;
    Trajectory **trajectory = params->trajectory;

    for (;;)
    {
      if (flag == Flag::EXECUTE_TRAJECTORY && *trajectory != nullptr)
      {
        flag = Flag::NOTHING;
        state = State::EXECUTING_TRAJECTORY;
        DBG_PRINTLN("[CONTROL] Starting trajectory execution...");
        execute_trajectory(*trajectory, encoder, stepper);
        DBG_PRINTLN("[CONTROL] Finished trajectory execution");
        state = State::IDLE;
      }

      if (flag == Flag::HOME)
      {
        flag = Flag::NOTHING;
        state = State::HOMING;
        DBG_PRINTLN("[CONTROL] Starting homing...");
        home(stepper, encoder);
        DBG_PRINTLN("[CONTROL] Finished homing");
        state = State::IDLE;
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

  void execute_trajectory(Trajectory *trajectory, AS5600 &encoder, Stepper &stepper)
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