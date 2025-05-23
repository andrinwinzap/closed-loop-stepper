#include <Arduino.h>
#include <AS5600/AS5600.h>
#include <Stepper/Stepper.h>
#include <Trajectory/Trajectory.h>

#define HALL_PIN 15
#define GEAR_RATIO 15

AS5600 encoder;
Stepper stepper(17, 16, 4); // STEP, DIR, EN

void home() {

  const float homing_speed = 1*PI;
  const float homing_acceleration = 1*PI;

  Serial.println("Starting homing process...");

  stepper.setSpeed(-homing_speed);
  stepper.move(-PI/4.0 * GEAR_RATIO, homing_speed, homing_acceleration);

  Serial.println("BlaBla");
  
  float first_bound = 0;

  stepper.start();
  stepper.accelerate(0, homing_speed, PI);

  float filteredHallSensorValue = analogRead(HALL_PIN);
  float hallSensorAlpha = 0.2;
  float hallSensorUpdateFrequency = 1000;

  float hallSensorUpdatePeriod = 1e6 / hallSensorUpdateFrequency;

  while (true) {
    
    stepper.updateAcceleration();
    int raw = analogRead(HALL_PIN);
    filteredHallSensorValue = hallSensorAlpha * raw + (1 - hallSensorAlpha) * filteredHallSensorValue;

    if (filteredHallSensorValue < 1) {

      //first_bound = encoder.getCumulativeAngle();
      encoder.setCumulativePosition(0);
      stepper.accelerate(homing_speed, 0, homing_acceleration);
      while (stepper.updateAcceleration())
      Serial.println("Finished homing");
      Serial.println(encoder.getCumulativePosition() / GEAR_RATIO);
      break;

    }

    delayMicroseconds(hallSensorUpdatePeriod);
  }

  // while (true) {
    
  //   int raw = analogRead(HALL_PIN);
  //   filteredHallSensorValue = hallSensorAlpha * raw + (1 - hallSensorAlpha) * filteredHallSensorValue;

  //   if (filteredHallSensorValue > 1) {

  //     stepper.stop();
  //     float current_pos = encoder.getCumulativeAngle();
  //     float home = (current_pos - first_bound) / 2;
  //     encoder.setCumulativeAngle(current_pos - home);

  //     move_to(0, encoder, stepper, GEAR_RATIO);

  //     Serial.println("Finished homing");

  //     delay(1000);
  //     break;

  //   }

  //   delayMicroseconds(hallSensorUpdatePeriod);
  // }

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
  stepper.setMicrosteps(16);
  stepper.setStepsPerRevolution(200);
  stepper.setMaxSpeed(1000000);

  pinMode(HALL_PIN, INPUT_PULLUP);
  
  home();
  
  // Serial.println(encoder.getCumulativePosition()/GEAR_RATIO);

  // Waypoint trajectory[] = {
  //   {0,          0,      0},        // start at 0 rad, velocity 0 at t=0 ms
  //   {PI/2.0,     0,   5000},
  //   {0,     0,   10000}
  // };


  // unsigned long motion_start_time = millis();
  // execute_trajectory(trajectory, 3, encoder, stepper, GEAR_RATIO);
  // Serial.print("Motion Duration: ");
  // Serial.println(millis() - motion_start_time);

  // stepper.disable();
}

void loop() {
}

