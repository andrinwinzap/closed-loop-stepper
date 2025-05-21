#pragma once
#include <Arduino.h>

class Stepper {
public:
  Stepper(uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
          uint16_t stepsPerRevolution = 200, uint8_t microsteps = 1);

  void begin();
  void setSpeed(float radPerSec); // Set angular speed in radians/sec
  void enable();                  // Enable driver
  void disable();                 // Disable driver
  void setMicrosteps(uint8_t microsteps);
  void setStepsPerRevolution(uint16_t stepsPerRevolution);

private:
  uint8_t stepPin, dirPin, enPin;
  uint16_t stepsPerRev;
  uint8_t microsteps;
  float currentFrequency = 0;

  hw_timer_t* timer = nullptr;
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

  static Stepper* instance;
  static void IRAM_ATTR onTimerISR();
  void toggleStepPin();
  void updateTimer();
};
