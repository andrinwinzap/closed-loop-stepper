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
  void setMaxSpeed(float radPerSec);           // Set max angular speed
  void setMaxAcceleration(float radPerSec2);   // Set max angular acceleration

private:
  uint8_t stepPin, dirPin, enPin;
  uint16_t stepsPerRev;
  uint8_t microsteps;
  float currentFrequency = 0;
  float maxSpeedRadPerSec = 100.0f;           // Default max speed
  float maxAccelRadPerSec2 = 100.0f;          // Default max acceleration

  hw_timer_t* timer = nullptr;
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

  static Stepper* instance;
  static void IRAM_ATTR onTimerISR();
  void toggleStepPin();
  void updateTimer();
};
