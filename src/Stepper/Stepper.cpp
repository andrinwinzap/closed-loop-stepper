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

  if (radPerSec == 0.0f) {
    digitalWrite(stepPin, LOW);
    currentFrequency = 0;
    if (isRunning) {
      stop();
    }
    return;
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
  if (currentFrequency > 0.0f && !isRunning) {
    timerAlarmEnable(timer);
    isRunning = true;
  }
}


void Stepper::stop() {
  if (isRunning) {
    timerAlarmDisable(timer);
    digitalWrite(stepPin, LOW);  // Ensure step pin is low
    isRunning = false;
  }
}

