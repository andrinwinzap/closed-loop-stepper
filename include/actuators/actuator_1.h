#include <Arduino.h>

constexpr bool DUMMY_MODE = true;

// Stepper Motor
constexpr int STEPPER_STEP_PIN = 17;
constexpr int STEPPER_DIR_PIN = 16;
constexpr int STEPPER_EN_PIN = 4;
constexpr int STEPPER_MICROSTEPS = 8;
constexpr int STEPPER_STEPS_PER_REVOLUTION = 200;

// Hall Effect Sensor
constexpr int HALL_EFFECT_SENSOR_PIN = 15;
constexpr float HALL_EFFECT_SENSOR_ALPHA = 0.6;
constexpr int HALL_EFFECT_SENSOR_HIGH_THRESHOLD = 4000;
constexpr int HALL_EFFECT_SENSOR_LOW_THRESHOLD = 100;

// Homing
constexpr int BEEP_DURATION = 300;                        // ms
constexpr int INITIAL_HOME_POSITION_DETECTION_DELAY = 50; // ms
constexpr int HOMING_SEQUENCE_DELAY = 500;                // ms
constexpr float HOMING_SEQUENCE_OFFSET = 0.2;             // rad500

// Gearbox
constexpr float GEAR_RATIO = 15.0;

// SPEED & ACCELERATION
constexpr float HOMING_SPEED = 0.1;      // rad/s
constexpr float MAX_SPEED = 6.0f;        // rad/s
constexpr float MAX_ACCELERATION = 0.3f; // rad/s²

// Closed Loop
constexpr float KF = 1.0f;                             // Velocity feed forward gain
constexpr float KP = 10.0f;                            // Proportional gain
constexpr float KV = 1.0f;                             // Derivative gain
constexpr float VELOCITY_FILTER_APHA = 0.1f;           // Velocity low pass filter alpha
constexpr float STALL_VELOCITY_THRESHOLD = 0.1f;       // Stalling velocity threshold
constexpr float STALL_POSITION_ERROR_THRESHOLD = 0.1f; // Stalling position-error threshold
constexpr int STALL_TIME_THRESHOLD = 50;               // Minimal duration for stalling detection
constexpr int CONTROL_LOOP_FREQUENCY = 500;            // hz
constexpr float POSITION_TOLERANCE = 0.0001f;          // rad
constexpr TickType_t CONTROL_LOOP_PERIOD_TICKS = configTICK_RATE_HZ / CONTROL_LOOP_FREQUENCY;
constexpr float CONTROL_LOOP_INTERVAL = 1000.0f / CONTROL_LOOP_FREQUENCY;

// Serial Communication Protocol
constexpr int PROTOCOL_ADDRESS = 0x02;
constexpr int RXD2 = 26;
constexpr int TXD2 = 27;