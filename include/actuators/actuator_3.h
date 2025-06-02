#include <Arduino.h>

// Stepper Motor
#define STEPPER_STEP_PIN 17
#define STEPPER_DIR_PIN 16
#define STEPPER_EN_PIN 4
#define STEPPER_MICROSTEPS 8
#define STEPPER_STEPS_PER_REVOLUTION 200

// Hall Effect Sensor
#define HALL_EFFECT_SENSOR_PIN 15
constexpr float HALL_EFFECT_SENSOR_UPDATE_FREQUENCY = 1000;
constexpr float HALL_EFFECT_SENSOR_ALPHA = 0.2;
constexpr float HALL_EFFECT_SENSOR_UPDATE_PERIOD = 1e6 / HALL_EFFECT_SENSOR_UPDATE_FREQUENCY;
#define PRINT_HALL_SENSOR_VALUE false

// Encoder
constexpr float ENCODER_UPDATE_FREQUENCY = 10000.0f;

// Gearbox
constexpr float GEAR_RATIO = 15.0;

// SPEED & ACCELERATION
constexpr float HOMING_SPEED = 0.3;
constexpr float HOMING_ACCELERATION = 0.5;
constexpr float MAX_SPEED = 6.0f;
constexpr float MAX_ACCELERATION = 0.3f;

// Closed Loop Trajectory
constexpr float KF = 1.0f;  // Velocity feed forward gain
constexpr float KP = 10.0f; // Proportional gain
constexpr float KV = 1.0f;  // Derivative gain
constexpr int CONTROL_LOOP_FREQUENCY = 500;
constexpr TickType_t CONTROL_LOOP_PERIOD_TICKS = configTICK_RATE_HZ / CONTROL_LOOP_FREQUENCY;
constexpr float CONTROL_LOOP_INTERVAL = 1000.0f / CONTROL_LOOP_FREQUENCY;
constexpr float CONTROL_LOOP_DEBUG_FREQUENCY = 100.0f;
constexpr float CONTROL_LOOP_DEBUG_INTERVAL = 1000.0f / CONTROL_LOOP_DEBUG_FREQUENCY;
constexpr float VELOCITY_FILTER_APHA = 0.1f;           // Velocity low pass filter alpha
constexpr float STALL_VELOCITY_THRESHOLD = 0.1f;       // Stalling velocity threshold
constexpr float STALL_POSITION_ERROR_THRESHOLD = 0.1f; // Stalling position-error threshold
constexpr int STALL_TIME_THRESHOLD = 50;               // Minimal duration for stalling detection
#define CONTROL_LOOP_DEBUG_OUTPUT false

// Closed Loop Move To
constexpr float KP_MOVE_TO = 1.0f;
constexpr float POSITION_TOLERANCE = 0.01f;

// Serial Communication Protocol
#define PROTOCOL_ADDRESS 0x04
#define RXD2 26
#define TXD2 27