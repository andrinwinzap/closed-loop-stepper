#include <Arduino.h>
#include <AS5600/AS5600.h>
#include <Stepper/Stepper.h>
#include <Trajectory/Trajectory.h>

#define HALL_PIN 15
#define GEAR_RATIO 13

AS5600 encoder;
Stepper stepper(17, 16, 4); // STEP, DIR, EN

void home() {

  const float homing_speed = 2*PI;

  Serial.println("Starting homing process...");

  stepper.setSpeed(-homing_speed);
  stepper.move((PI/4.0) * GEAR_RATIO);
  
  stepper.setSpeed(homing_speed);
  stepper.start();
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

  stepper.stop();
  encoder.resetCumulativeAngle();

  delay(1000);

  Serial.println("Finished homing");
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
  stepper.setMaxSpeed(50);

  pinMode(HALL_PIN, INPUT_PULLUP);
  
  home();

  Waypoint trajectory[] = {
  Waypoint {
                      0,
                      0,
                      0
                    },
  Waypoint {
                      2*PI,
                      0,
                      5000
                    }
                  };
  unsigned long motion_start_time = millis();
  execute_trajectory(trajectory, 2, encoder, stepper, GEAR_RATIO);
  Serial.print("Motion Duration: ");
  Serial.println(millis() - motion_start_time);
}

void loop() {
}

