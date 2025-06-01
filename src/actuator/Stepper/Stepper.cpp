#include "Stepper.h"

Stepper *Stepper::instance = nullptr;

Stepper::Stepper(uint8_t stepPin, uint8_t dirPin, uint8_t enPin, float gear_ratio,
                 uint16_t stepsPerRevolution, uint8_t microsteps)
    : stepPin(stepPin), dirPin(dirPin), enPin(enPin), gear_ratio(gear_ratio),
      stepsPerRev(stepsPerRevolution), microsteps(microsteps) {}

void Stepper::begin()
{
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  digitalWrite(enPin, HIGH);

  instance = this;
  timer = timerBegin(0, 80, true); // 1 µs resolution
  timerAttachInterrupt(timer, &Stepper::onTimerISR, true);
}

void Stepper::setSpeed(float radPerSec)
{
  // Clamp to max speed
  if (radPerSec > maxSpeedRadPerSec)
  {
    radPerSec = maxSpeedRadPerSec;
  }
  else if (radPerSec < -maxSpeedRadPerSec)
  {
    radPerSec = -maxSpeedRadPerSec;
  }

  currentSpeed = radPerSec;

  if (radPerSec == 0.0f)
  {
    digitalWrite(stepPin, LOW);
    if (timerAlarmIsEnabled)
    {
      timerAlarmDisable(timer);
      timerAlarmIsEnabled = false;
    }
    return;
  }
  else
  {
    if (isRunning && !timerAlarmIsEnabled)
    {
      timerAlarmEnable(timer);
      timerAlarmIsEnabled = true;
    }
  }

  if (radPerSec > 0)
  {
    digitalWrite(dirPin, HIGH);
    direction = true;
  }
  else
  {
    digitalWrite(dirPin, LOW);
    direction = false;
  }

  currentFrequency = float(abs(radPerSec) * stepsPerRev * microsteps * gear_ratio) / float(2 * PI);
  updateTimer();
}

void Stepper::enable()
{
  digitalWrite(enPin, LOW);
}

void Stepper::disable()
{
  digitalWrite(enPin, HIGH);
}

void IRAM_ATTR Stepper::onTimerISR()
{
  if (instance)
  {
    portENTER_CRITICAL_ISR(&instance->timerMux);
    instance->toggleStepPin();
    portEXIT_CRITICAL_ISR(&instance->timerMux);
  }
}

void Stepper::toggleStepPin()
{
  digitalWrite(stepPin, !digitalRead(stepPin));
}

void Stepper::updateTimer()
{
  uint64_t ticks = (uint64_t)(500000.0 / currentFrequency); // half-period in µs
  timerAlarmWrite(timer, ticks, true);
}

void Stepper::setMicrosteps(uint8_t microsteps)
{
  this->microsteps = microsteps;
  updateTimer();
}

void Stepper::setStepsPerRevolution(uint16_t stepsPerRevolution)
{
  this->stepsPerRev = stepsPerRevolution;
  updateTimer();
}

void Stepper::setMaxSpeed(float radPerSec)
{
  maxSpeedRadPerSec = fabs(radPerSec);
}

void Stepper::move(float angleRad, float max_speed_rads, float max_accel_rads2)
{
  if (angleRad == 0.0f)
    return;

  // Set direction
  digitalWrite(dirPin, angleRad > 0 ? HIGH : LOW);

  // Calculate total steps needed
  const float absAngle = fabs(angleRad);
  const uint32_t steps = static_cast<uint32_t>((absAngle / (2.0f * PI)) * stepsPerRev * microsteps * gear_ratio);
  if (steps == 0)
    return;

  // Convert radian units to step units
  const float steps_per_rad = (stepsPerRev * microsteps * gear_ratio) / (2.0f * PI);
  const float max_speed = max_speed_rads * steps_per_rad;  // steps/s
  const float max_accel = max_accel_rads2 * steps_per_rad; // steps/s²

  // Calculate acceleration profile
  const float accel_time = max_speed / max_accel;
  const float accel_steps = 0.5f * max_accel * accel_time * accel_time;

  uint32_t steps_accel = static_cast<uint32_t>(accel_steps);
  uint32_t steps_decel = steps_accel;
  uint32_t steps_coast = 0;

  // Determine motion profile
  bool triangular = false;
  if (steps_accel + steps_decel >= steps)
  {
    // Triangular profile (no coast phase)
    triangular = true;
    steps_accel = steps / 2;
    steps_decel = steps - steps_accel;
  }
  else
  {
    // Trapezoidal profile (with coast phase)
    steps_coast = steps - steps_accel - steps_decel;
  }

  // Calculate initial parameters
  float current_speed = 0;
  unsigned long last_step_time = micros();

  // Acceleration phase
  for (uint32_t s = 1; s <= steps_accel; s++)
  {
    current_speed = sqrt(2.0f * max_accel * s);
    if (current_speed > max_speed)
      current_speed = max_speed;

    const unsigned long step_delay = static_cast<unsigned long>(500000.0f / current_speed);

    while (micros() - last_step_time < step_delay)
    {
    }
    digitalWrite(stepPin, HIGH);
    last_step_time = micros();

    while (micros() - last_step_time < step_delay)
    {
    }
    digitalWrite(stepPin, LOW);
    last_step_time = micros();
  }

  // Coast phase (trapezoidal only)
  if (!triangular)
  {
    const unsigned long coast_delay = static_cast<unsigned long>(500000.0f / max_speed);

    for (uint32_t s = 0; s < steps_coast; s++)
    {
      while (micros() - last_step_time < coast_delay)
      {
      }
      digitalWrite(stepPin, HIGH);
      last_step_time = micros();

      while (micros() - last_step_time < coast_delay)
      {
      }
      digitalWrite(stepPin, LOW);
      last_step_time = micros();
    }
  }

  // Deceleration phase
  for (uint32_t s = steps_decel; s > 0; s--)
  {
    current_speed = sqrt(2.0f * max_accel * s);
    if (current_speed > max_speed)
      current_speed = max_speed;

    const unsigned long step_delay = static_cast<unsigned long>(500000.0f / current_speed);

    while (micros() - last_step_time < step_delay)
    {
    }
    digitalWrite(stepPin, HIGH);
    last_step_time = micros();

    while (micros() - last_step_time < step_delay)
    {
    }
    digitalWrite(stepPin, LOW);
    last_step_time = micros();
  }
}

void Stepper::start()
{
  isRunning = true;
  if (!timerAlarmIsEnabled && currentFrequency > 0.0f)
  {
    timerAlarmEnable(timer);
    timerAlarmIsEnabled = true;
  }
}

void Stepper::stop()
{
  isRunning = false;
  if (timerAlarmIsEnabled)
  {
    timerAlarmDisable(timer);
    timerAlarmIsEnabled = false;
    digitalWrite(stepPin, LOW); // Ensure step pin is low
  }
}

void Stepper::accelerate(float start_speed, float target_speed, float acceleration)
{
  // Clamp target speed
  if (target_speed > maxSpeedRadPerSec)
    target_speed = maxSpeedRadPerSec;
  if (target_speed < -maxSpeedRadPerSec)
    target_speed = -maxSpeedRadPerSec;

  acceleration_target_speed = target_speed;

  // Determine direction of acceleration
  float speed_diff = target_speed - start_speed;
  if (speed_diff == 0)
  {
    accelerating = false;
    return;
  }

  acceleration_target = (speed_diff > 0 ? 1 : -1) * fabs(acceleration);
  accelerating = true;

  setSpeed(start_speed);
  last_acceleration_update = micros();
}

bool Stepper::updateAcceleration()
{
  if (!accelerating)
    return false;

  unsigned long time_d = micros() - last_acceleration_update;
  float delta_t_sec = time_d / 1e6;

  // Compute new speed
  float updated_speed = currentSpeed + acceleration_target * delta_t_sec;

  // Check if we've reached or passed the target
  bool reached_target =
      (acceleration_target > 0 && updated_speed >= acceleration_target_speed) ||
      (acceleration_target < 0 && updated_speed <= acceleration_target_speed);

  if (reached_target)
  {
    setSpeed(acceleration_target_speed); // Snap to target speed
    accelerating = false;
  }
  else
  {
    setSpeed(updated_speed);
    last_acceleration_update = micros();
  }

  return accelerating;
}
