#include <Arduino.h>
#include "AS5600/AS5600.h"
#include "Stepper/Stepper.h"

#define HALL_PIN 15
#define GEAR_RATIO 13

AS5600 encoder;
Stepper stepper(17, 16, 4); // STEP, DIR, EN

struct Waypoint {
  float position;             // radians
  float velocity;             // radians/second
  unsigned long timestamp;   // milliseconds
};

// Hermite interpolation taking elapsed time (ms) directly
float hermiteInterpolate(const Waypoint &wp1,
                         const Waypoint &wp2,
                         unsigned long elapsed)
{
    float dt_ms = float(wp2.timestamp - wp1.timestamp);
    if (dt_ms <= 0) return wp1.position;

    // Normalized parameter u in [0,1]
    float u = float(elapsed) / dt_ms;
    u = constrain(u, 0.0f, 1.0f);

    float u2 = u * u;
    float u3 = u2 * u;

    // Hermite basis
    float h00 =  2*u3 - 3*u2 + 1;
    float h10 =      u3 - 2*u2 + u;
    float h01 = -2*u3 + 3*u2;
    float h11 =      u3 -   u2;

    // Scale velocities over the interval
    float dt_s = dt_ms * 0.001f;
    float m0 = wp1.velocity * dt_s;
    float m1 = wp2.velocity * dt_s;

    return h00 * wp1.position
         + h10 * m0
         + h01 * wp2.position
         + h11 * m1;
}

// Hermite derivative: instantaneous velocity (rad/s) from elapsed time
float hermiteVelocity(const Waypoint &wp1,
                      const Waypoint &wp2,
                      unsigned long elapsed)
{
    float dt_ms = float(wp2.timestamp - wp1.timestamp);
    if (dt_ms <= 0) return 0;

    float u = float(elapsed) / dt_ms;
    if (u <= 0.0f || u >= 1.0f) return 0;

    float u2 = u * u;

    // Basis derivatives
    float dh00 =  6*u2 - 6*u;
    float dh10 =  3*u2 - 4*u + 1;
    float dh01 = -6*u2 + 6*u;
    float dh11 =  3*u2 - 2*u;

    float dt_s = dt_ms * 0.001f;
    float m0 = wp1.velocity * dt_s;
    float m1 = wp2.velocity * dt_s;

    // d(position)/du
    float dPu = dh00 * wp1.position
              + dh10 * m0
              + dh01 * wp2.position
              + dh11 * m1;

    // du/dt = 1 / dt_s
    return dPu / dt_s;
}

void home() {

  stepper.setSpeed(-2*PI);
  delay(1000);

  stepper.setSpeed(2*PI);
  int counter = 0;
  const int threshold = 10;
  const int interval = 1;
  unsigned long last_check = millis();

  while (counter < threshold) {
    if (millis() - last_check > interval) {
      if (analogRead(HALL_PIN) <= 10) {
        counter++;
      } else {
        counter = 0;
      }
      last_check = millis();
    }
  }

  stepper.setSpeed(0);
  encoder.resetCumulativeAngle();
}

void execute_motion_profile_segment(const Waypoint &wp1, const Waypoint &wp2) {

  Serial.print("Position before motion: ");
  Serial.println(encoder.getCumulativeAngle());

  // PID Parameters
  const float Kp = 10;
  const float Kv = 10;

  const int control_loop_frequency = 10000; // Control loop frequency in Hz
  const int serial_print_frequency = 5; // Serial print frequency in Hz

  unsigned long start_time = millis();
  unsigned long last_control = 0;
  unsigned long last_serial_print = 0;
  float last_control_speed = 0;
  
  const int control_interval = 1000/control_loop_frequency;
  const int serial_print_interval = 1000/serial_print_frequency;

  unsigned long motion_duration = wp2.timestamp - wp1.timestamp;

  while (true) {
    unsigned long now = millis();
    unsigned long elapsed = now - start_time;

    if (elapsed > motion_duration) break;

    if (now - last_control >= control_interval) {
      float desired_pos = hermiteInterpolate(wp1, wp2, elapsed);
      float desired_vel = hermiteVelocity   (wp1, wp2, elapsed);

      float measured_pos = encoder.getCumulativeAngle() / float(GEAR_RATIO);
      float measured_vel = encoder.getVelocity() / float(GEAR_RATIO);
      

      float pos_error = desired_pos - measured_pos;
      float vel_error = desired_vel - measured_vel;

      float control_speed = desired_vel + Kp * pos_error;


      if (millis() - last_serial_print > serial_print_interval) {
        Serial.print("Position: ");
        Serial.print(measured_pos);
        Serial.print(" | ");
        Serial.print("Position Error: ");
        Serial.print(pos_error);
        Serial.print(" | ");
        Serial.print("Velocity: ");
        Serial.print(measured_vel);
        Serial.print(" | ");
        Serial.print("Velocity Error: ");
        Serial.print(vel_error);
        Serial.print(" | ");
        Serial.print("Control Speed: ");
        Serial.println(control_speed);

        last_serial_print = millis();
      }

      if (control_speed != last_control_speed){
        stepper.setSpeed(control_speed * GEAR_RATIO);
        last_control_speed = control_speed;
      }

      last_control = now;
    }
  }
  
  stepper.setSpeed(wp2.velocity);

  Serial.print("Position after motion: ");
  Serial.println(encoder.getCumulativeAngle());
}

void execute_motion_profile(const Waypoint* arr, size_t length) {
  for (size_t i = 0; i < length-1; ++i) {
    execute_motion_profile_segment(arr[i], arr[i+1]);
  }
} 

void setup() {

  Serial.begin(115200);

  Wire.begin();  // Default SDA = 21, SCL = 22 on ESP32

  if (!encoder.begin()) {
    Serial.println("AS5600 not found!");
    while (1);
  }

  Serial.println("AS5600 ready.");

  stepper.begin();
  stepper.enable(); // keep driver enabled
  stepper.setMicrosteps(32);
  stepper.setStepsPerRevolution(200);

  pinMode(HALL_PIN, INPUT_PULLUP);
  
  home();

  Serial.println("Homing done");

  delay(2000);

  Waypoint motion[] = {
  Waypoint {
                      0,
                      0,
                      0
                    },
  Waypoint {
                      2*PI,
                      1,
                      10000
                    },
  Waypoint {
                      4*PI,
                      0,
                      20000
                    }
                  };
  execute_motion_profile(motion, 3);
}

void loop() {
}

