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
  digitalWrite(enPin, LOW);

  instance = this;
  timer = timerBegin(0, 80, true); // 1 µs resolution
  timerAttachInterrupt(timer, &Stepper::onTimerISR, true);
}

void Stepper::setSpeed(float radPerSec) {
  if (radPerSec == 0.0f) {
    timerAlarmDisable(timer);
    digitalWrite(stepPin, LOW); // optional: ensure step pin is low
    return;
  }

  if (radPerSec > 0) {
    digitalWrite(dirPin, HIGH);  // forward direction
  } else {
    digitalWrite(dirPin, LOW);   // reverse direction
    radPerSec = -radPerSec;      // take absolute value for frequency calculation
  }

  currentFrequency = float(radPerSec * stepsPerRev * microsteps) / float(2 * PI);
  updateTimer();
  timerAlarmEnable(timer);
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
  if (currentFrequency > 0) {
    updateTimer();
  }
}

void Stepper::setStepsPerRevolution(uint16_t stepsPerRevolution) {
  this->stepsPerRev = stepsPerRevolution;
  if (currentFrequency > 0) {
    updateTimer();
  }
}