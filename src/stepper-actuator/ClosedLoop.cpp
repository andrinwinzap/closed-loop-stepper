#include "ClosedLoop.h"

namespace ControlLoop
{
  void task(void *param)
  {
    DBG_PRINTLN("[CONTROL] Loop started");

    Params *params = static_cast<Params *>(param);

    Stepper &stepper = *params->stepper;
    AS5600 &encoder = *params->encoder;
    volatile State &state = *params->state;
    volatile Flag &flag = *params->flag;
    ActuatorTrajectory **trajectory_ptr = params->trajectory;
    ActuatorTrajectory *trajectory = nullptr;
    float *target_position = params->target_position;

    state = State::HOMING;
    flag = Flag::NOTHING;

    float filtered_vel = encoder.getSpeed();
    float filtered_hall_sensor_value = analogRead(HALL_EFFECT_SENSOR_PIN);
    float last_control_speed = 0;
    unsigned long stall_start_time = 0;

    HomingState homing_state = HomingState::WAITING_FOR_HOMING;
    unsigned long initial_home_position_start;
    float cw_edge, ccw_edge;

    TrajectoryContext trajectory_context;

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;)
    {
      unsigned long now = millis();

      if (flag != Flag::NOTHING)
      {
        switch (flag)
        {

        case Flag::ESTOP:
          state = State::ESTOP;
          stepper.stop();
          stepper.disable();
          DBG_PRINTLN("[CONTROL] ESTOP TRIGGERED!");
          flag = Flag::NOTHING;
          break;

        case Flag::IDLE:
          state = State::IDLE;
          flag = Flag::NOTHING;
          break;

        case Flag::POSITION:
          if (target_position != nullptr)
          {
            state = State::POSITION;
            filtered_vel = encoder.getSpeed();
            last_control_speed = 0;
            stall_start_time = 0;
            stepper.start();
            DBG_PRINTLN("[CONTROL] Start position mode");
          }
          flag = Flag::NOTHING;
          break;

        case Flag::TRAJECTORY:
          if (trajectory_ptr != nullptr && *trajectory_ptr != nullptr)
          {
            state = State::TRAJECTORY;
            trajectory = *trajectory_ptr;
            trajectory_context.segment_index = 0;
            trajectory_context.segment_start = now;
            trajectory_context.wp1 = &trajectory->waypoints[0];
            trajectory_context.wp2 = &trajectory->waypoints[1];
            filtered_vel = encoder.getSpeed();
            last_control_speed = 0;
            stall_start_time = 0;
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

      case State::HOMING:
      {
        int raw = analogRead(HALL_EFFECT_SENSOR_PIN);
        filtered_hall_sensor_value = HALL_EFFECT_SENSOR_ALPHA * raw + (1 - HALL_EFFECT_SENSOR_ALPHA) * filtered_hall_sensor_value;
        switch (homing_state)
        {
        case HomingState::WAITING_FOR_HOMING:
        {
          if (filtered_hall_sensor_value < 1)
          {
            if (initial_home_position_start == 0)
            {
              initial_home_position_start = now;
            }
            else if (now - initial_home_position_start >= INITIAL_HOME_POSITION_DETECTION_DELAY + BEEP_DURATION + HOMING_SEQUENCE_DELAY)
            {
              encoder.setPosition(0);
              stepper.setSpeed(-HOMING_SPEED);
              stepper.start();
              homing_state = HomingState::PREPARING;
            }
            else if (now - initial_home_position_start >= INITIAL_HOME_POSITION_DETECTION_DELAY + BEEP_DURATION)
            {
              stepper.disableBeeperMode();
            }
            else if (now - initial_home_position_start >= INITIAL_HOME_POSITION_DETECTION_DELAY)
            {
              DBG_PRINTLN("[Control] Initial home position found");
              stepper.enable();
              stepper.activateBeeperMode();
            }
          }
          else
          {
            stepper.disableBeeperMode();
            stepper.disable();
            initial_home_position_start = 0;
          }
          break;
        }

        case HomingState::PREPARING:
        {
          if (fabs(encoder.getPosition()) >= 0.2)
          {
            stepper.setSpeed(HOMING_SPEED);
            homing_state = HomingState::CW_EDGE;
            DBG_PRINTLN("[Control] Preperation finished");
          }
          break;
        }

        case HomingState::CW_EDGE:
        {
          if (filtered_hall_sensor_value < 1)
          {
            cw_edge = encoder.getPosition();
            DBG_PRINT("[Control] CW edge position found: ");
            DBG_PRINTLN(cw_edge);
            homing_state = HomingState::CCW_EDGE;
          }
          break;
        }

        case HomingState::CCW_EDGE:
        {
          if (filtered_hall_sensor_value > 4000)
          {
            ccw_edge = encoder.getPosition();
            encoder.setPosition(((ccw_edge - cw_edge) / 2.0f));
            DBG_PRINT("[Control] CCW edge position found: ");
            DBG_PRINTLN(ccw_edge);
            DBG_PRINTLN(encoder.getPosition());
            homing_state = HomingState::HOMED;
            *target_position = 0;
            state = State::POSITION;
          }
          break;
        }

        default:
          break;
        }
        break;
      }

      case State::POSITION:
      {
        float dt = CONTROL_LOOP_INTERVAL * 0.001f; // ms to s

        float current_position = encoder.getPosition();
        float pos_error = *target_position - current_position;

        if (fabs(pos_error) <= POSITION_TOLERANCE)
        {
          stepper.setSpeed(0);
          stepper.stop();
          state = State::IDLE;
          DBG_PRINTLN("[CONTROL][MOVETO] Target reached within tolerance.");
          break;
        }

        float control_speed = KP * pos_error;

        if (control_speed > MAX_SPEED)
          control_speed = MAX_SPEED;
        else if (control_speed < -MAX_SPEED)
          control_speed = -MAX_SPEED;

        float max_speed_change = MAX_ACCELERATION * dt;
        float speed_diff = control_speed - last_control_speed;

        if (speed_diff > max_speed_change)
          control_speed = last_control_speed + max_speed_change;
        else if (speed_diff < -max_speed_change)
          control_speed = last_control_speed - max_speed_change;

        stepper.setSpeed(control_speed);

        last_control_speed = control_speed;
        break;
      }

      case State::TRAJECTORY:
      {
        unsigned long delta_time = now - trajectory_context.segment_start;

        if (delta_time >= trajectory_context.wp2->timestamp - trajectory_context.wp1->timestamp)
        {
          trajectory_context.segment_index++;
          if (trajectory_context.segment_index >= trajectory->length - 1)
          {
            DBG_PRINTLN("[CONTROL] Finished trajectory");
            *target_position = trajectory_context.wp2->position;
            state = State::POSITION;
            break;
          }
          else
          {
            trajectory->waypoints[trajectory_context.segment_index].position = encoder.getPosition();
            trajectory->waypoints[trajectory_context.segment_index].velocity = encoder.getSpeed();
            trajectory_context.wp1 = &trajectory->waypoints[trajectory_context.segment_index];
            trajectory_context.wp2 = &trajectory->waypoints[trajectory_context.segment_index + 1];
            trajectory_context.segment_start = now;
          }
          break;
        }

        float desired_pos = hermite_interpolate(*trajectory_context.wp1, *trajectory_context.wp2, delta_time);
        float desired_vel = hermite_velocity(*trajectory_context.wp1, *trajectory_context.wp2, delta_time);

        float measured_pos = encoder.getPosition();
        float measured_vel = encoder.getSpeed();
        filtered_vel = VELOCITY_FILTER_APHA * measured_vel + (1 - VELOCITY_FILTER_APHA) * filtered_vel;

        float pos_error = desired_pos - measured_pos;
        float vel_error = desired_vel - filtered_vel;

        if (filtered_vel < STALL_VELOCITY_THRESHOLD && pos_error > STALL_POSITION_ERROR_THRESHOLD)
        {
          if (stall_start_time == 0)
          {
            stall_start_time = now;
          }
          else if (now - stall_start_time > STALL_TIME_THRESHOLD)
          {
            DBG_PRINT("[CONTROL][TRAJ][STALL] PosErr=");
            DBG_PRINT(pos_error, 4);
            DBG_PRINT("\tFilteredVel=");
            DBG_PRINTLN(filtered_vel, 4);

            *trajectory_context.wp1 = {
                measured_pos,
                0.0f,
                trajectory_context.wp1->timestamp + delta_time};
            trajectory_context.segment_start = now;
            break;
          }
        }
        else
        {
          stall_start_time = 0;
        }

        float control_speed = KF * desired_vel + KP * pos_error + KV * vel_error;

        if (control_speed != last_control_speed)
        {
          stepper.setSpeed(control_speed);
          last_control_speed = control_speed;
        }
        break;
      }

      default:
        break;
      }

      encoder.update();
      vTaskDelayUntil(&last_wake_time, CONTROL_LOOP_PERIOD_TICKS);
    }
  }
}