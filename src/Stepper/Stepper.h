#pragma once
#include <Arduino.h>

class Stepper {
public:
  Stepper(uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
          uint16_t stepsPerRevolution = 200, uint8_t microsteps = 1);

  void begin();
  void enable();                  // Enable driver
  void disable();                 // Disable driver
  void setSpeed(float radPerSec); // Set angular speed in radians/sec
  void setMicrosteps(uint8_t microsteps);
  void setStepsPerRevolution(uint16_t stepsPerRevolution);
  void setMaxSpeed(float radPerSec);           // Set max angular speed
  void move(float angleRad);
  void start();  // Start continuous stepping at current speed
  void stop();   // Stop continuous stepping



private:
  uint8_t stepPin, dirPin, enPin;
  uint16_t stepsPerRev;
  uint8_t microsteps;
  float currentFrequency = 0;
  float maxSpeedRadPerSec = 100.0f;           // Default max speed
  bool isRunning = false;
  bool timerAlarmIsEnabled = false;
  bool direction = true; // true = HIGH (forward), false = LOW (reverse)


  hw_timer_t* timer = nullptr;
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

  static Stepper* instance;
  static void IRAM_ATTR onTimerISR();
  void toggleStepPin();
  void updateTimer();
};
