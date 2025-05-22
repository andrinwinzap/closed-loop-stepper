#include <Arduino.h>
#include <AS5600/AS5600.h>
#include <Stepper/Stepper.h>
#include <Trajectory/Trajectory.h>

#define HALL_PIN 15
#define GEAR_RATIO 13

AS5600 encoder;
Stepper stepper(17, 16, 4); // STEP, DIR, EN

void home() {

  const float homing_speed = 4*PI;

  Serial.println("Starting homing process...");

  stepper.setSpeed(-homing_speed);
  stepper.move((PI/4.0) * GEAR_RATIO);
  
  float first_bound = 0;

  stepper.setSpeed(homing_speed);
  stepper.start();

  float filteredHallSensorValue = analogRead(HALL_PIN);
  float hallSensorAlpha = 0.2;
  float hallSensorUpdateFrequency = 1000;

  float hallSensorUpdatePeriod = 1e6 / hallSensorUpdateFrequency;

  while (true) {
    
    int raw = analogRead(HALL_PIN);
    filteredHallSensorValue = hallSensorAlpha * raw + (1 - hallSensorAlpha) * filteredHallSensorValue;

    if (filteredHallSensorValue < 1) {

      first_bound = encoder.getCumulativeAngle();
      break;

    }

    delayMicroseconds(hallSensorUpdatePeriod);
  }

  while (true) {
    
    int raw = analogRead(HALL_PIN);
    filteredHallSensorValue = hallSensorAlpha * raw + (1 - hallSensorAlpha) * filteredHallSensorValue;

    if (filteredHallSensorValue > 1) {

      stepper.stop();
      float current_pos = encoder.getCumulativeAngle();
      float home = (current_pos - first_bound) / 2;
      encoder.setCumulativeAngle(current_pos - home);

      Serial.println("Finished homing");

      delay(1000);
      break;

    }

    delayMicroseconds(hallSensorUpdatePeriod);
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
  stepper.setMaxSpeed(50);

  pinMode(HALL_PIN, INPUT_PULLUP);
  
  home();

Waypoint trajectory[] = {
  {0,          0,      0},        // start at 0 rad, velocity 0 at t=0 ms
  {PI,     0,   8000},
  {-2*PI,     0,   16000},
};


  unsigned long motion_start_time = millis();
  execute_trajectory(trajectory, 3, encoder, stepper, GEAR_RATIO);
  Serial.print("Motion Duration: ");
  Serial.println(millis() - motion_start_time);
}

void loop() {
}

