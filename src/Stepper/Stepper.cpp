#include "Stepper.h"

Stepper* Stepper::instance = nullptr;

Stepper::Stepper(uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                 uint16_t stepsPerRevolution, uint8_t microsteps)
  : stepPin(stepPin), dirPin(dirPin), enPin(enPin),
    stepsPerRev(stepsPerRevolution), microsteps(microsteps) {}

void Stepper::begin() {
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

void Stepper::setSpeed(float radPerSec) {
  // Clamp to max speed
  if (radPerSec > maxSpeedRadPerSec) {
    radPerSec = maxSpeedRadPerSec;
  } else if (radPerSec < -maxSpeedRadPerSec) {
    radPerSec = -maxSpeedRadPerSec;
  }

  currentSpeed = radPerSec;

  if (radPerSec == 0.0f) {
    digitalWrite(stepPin, LOW);
    if (timerAlarmIsEnabled) {
      timerAlarmDisable(timer);
      timerAlarmIsEnabled = false;
    }
    return;
  } else {
    if (isRunning && !timerAlarmIsEnabled) {
      timerAlarmEnable(timer);
      timerAlarmIsEnabled = true;
    }
  }

  if (radPerSec > 0) {
    digitalWrite(dirPin, HIGH);
    direction = true;
  } else {
    digitalWrite(dirPin, LOW);
    direction = false;
  }

  currentFrequency = float(abs(radPerSec) * stepsPerRev * microsteps) / float(2 * PI);
  updateTimer();
}



void Stepper::enable() {
  digitalWrite(enPin, LOW);
}

void Stepper::disable() {
  digitalWrite(enPin, HIGH);
}

void IRAM_ATTR Stepper::onTimerISR() {
  if (instance) {
    portENTER_CRITICAL_ISR(&instance->timerMux);
    instance->toggleStepPin();
    portEXIT_CRITICAL_ISR(&instance->timerMux);
  }
}

void Stepper::toggleStepPin() {
  digitalWrite(stepPin, !digitalRead(stepPin));
}

void Stepper::updateTimer() {
  uint64_t ticks = (uint64_t)(500000.0 / currentFrequency); // half-period in µs
  timerAlarmWrite(timer, ticks, true);
}

void Stepper::setMicrosteps(uint8_t microsteps) {
  this->microsteps = microsteps;
  updateTimer();
}

void Stepper::setStepsPerRevolution(uint16_t stepsPerRevolution) {
  this->stepsPerRev = stepsPerRevolution;
  updateTimer();
}

void Stepper::setMaxSpeed(float radPerSec) {
  maxSpeedRadPerSec = fabs(radPerSec);
}

void Stepper::move(float angleRad) {
  if (angleRad == 0.0f) return;

  float absAngle = fabs(angleRad);

  // Calculate number of steps
  uint32_t steps = (uint32_t)((absAngle / (2.0f * PI)) * stepsPerRev * microsteps);

  // Calculate delay per half-cycle (i.e., time between step HIGH-LOW toggles)
  if (currentFrequency <= 0.0f) {
    Serial.println("Speed not set. Use setSpeed() before move().");
    return;
  }
  float halfPeriod_us = 500000.0 / currentFrequency;

  if (isRunning) {
      stop();
    }

  // Perform the steps
  for (uint32_t i = 0; i < steps; ++i) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds((uint32_t)halfPeriod_us);
    digitalWrite(stepPin, LOW);
    delayMicroseconds((uint32_t)halfPeriod_us);
  }
}

void Stepper::start() {
  isRunning = true;
  if (!timerAlarmIsEnabled && currentFrequency > 0.0f) {
    timerAlarmEnable(timer);
    timerAlarmIsEnabled = true;
  }
}


void Stepper::stop() {
  isRunning = false;
  if (timerAlarmIsEnabled) {
    timerAlarmDisable(timer);
    timerAlarmIsEnabled = false;
    digitalWrite(stepPin, LOW);  // Ensure step pin is low
  }
}

void Stepper::accelerate(float start_speed, float target_speed, float acceleration) {
  // Clamp target speed
  if (target_speed > maxSpeedRadPerSec) target_speed = maxSpeedRadPerSec;
  if (target_speed < -maxSpeedRadPerSec) target_speed = -maxSpeedRadPerSec;

  acceleration_target_speed = target_speed;

  // Determine direction of acceleration
  float speed_diff = target_speed - start_speed;
  if (speed_diff == 0) {
    accelerating = false;
    return;
  }

  acceleration_target = (speed_diff > 0 ? 1 : -1) * fabs(acceleration);
  accelerating = true;

  setSpeed(start_speed);
  last_acceleration_update = micros();
}

bool Stepper::updateAcceleration() {
  if (!accelerating) return false;

  unsigned long time_d = micros() - last_acceleration_update;
  float delta_t_sec = time_d / 1e6;

  // Compute new speed
  float updated_speed = currentSpeed + acceleration_target * delta_t_sec;

  // Check if we've reached or passed the target
  bool reached_target = 
      (acceleration_target > 0 && updated_speed >= acceleration_target_speed) ||
      (acceleration_target < 0 && updated_speed <= acceleration_target_speed);

  if (reached_target) {
    setSpeed(acceleration_target_speed);  // Snap to target speed
    accelerating = false;
  } else {
    setSpeed(updated_speed);
    last_acceleration_update = micros();
  }

  return accelerating;
}

