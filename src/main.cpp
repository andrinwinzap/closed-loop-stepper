// #include <Arduino.h>
// #include <AS5600/AS5600.h>
// #include <Stepper/Stepper.h>
// #include <Trajectory/Trajectory.h>

// #define HALL_PIN 15
// #define GEAR_RATIO 15.0

// AS5600 encoder(GEAR_RATIO);
// Stepper stepper(17, 16, 4, GEAR_RATIO); // STEP, DIR, EN

// void home() {

//   const float homing_speed = 0.3;
//   const float homing_acceleration = 0.5;

//   Serial.println("Starting homing process...");

//   stepper.setSpeed(-homing_speed);
//   stepper.move(-PI/8.0, homing_speed, homing_acceleration);

//   Serial.println("BlaBla");
  
//   float first_bound = 0;

//   stepper.start();
//   stepper.accelerate(0, homing_speed, homing_acceleration);

//   float filteredHallSensorValue = analogRead(HALL_PIN);
//   float hallSensorAlpha = 0.2;
//   float hallSensorUpdateFrequency = 1000;

//   float hallSensorUpdatePeriod = 1e6 / hallSensorUpdateFrequency;

//   while (true) {
    
//     stepper.updateAcceleration();
//     int raw = analogRead(HALL_PIN);
//     filteredHallSensorValue = hallSensorAlpha * raw + (1 - hallSensorAlpha) * filteredHallSensorValue;

//     if (filteredHallSensorValue < 1) break;

//     delayMicroseconds(hallSensorUpdatePeriod);
//   }

//   first_bound = encoder.getPosition();

//   while (true) {
    
//     int raw = analogRead(HALL_PIN);
//     filteredHallSensorValue = hallSensorAlpha * raw + (1 - hallSensorAlpha) * filteredHallSensorValue;

//     if (filteredHallSensorValue > 1) break;

//     delayMicroseconds(hallSensorUpdatePeriod);
//   }

//   float current_pos = encoder.getPosition();
//     float home = (current_pos - first_bound) / 2;
//     encoder.setPosition(current_pos - home);

//     stepper.accelerate(homing_speed, 0, homing_acceleration);
//     while (stepper.updateAcceleration()) {
//       unsigned long start = micros();
//       encoder.update();
//       Serial.println(micros()-start);
//     }
//     move_to(0, encoder, stepper);
//     Serial.println("Finished homing");
//     Serial.println(encoder.getPosition());

//     delay(1000);
// } 

// void setup() {

//   Serial.begin(115200);

//   Wire.begin();  // Default SDA = 21, SCL = 22 on ESP32

//   if (!encoder.begin()) {
//     Serial.println("AS5600 not found!");
//     while (1);
//   }

//   Serial.println("AS5600 ready.");

//   stepper.begin();
//   stepper.enable(); // keep driver enabled
//   stepper.setMicrosteps(8);
//   stepper.setStepsPerRevolution(200);
//   stepper.setMaxSpeed(1000000);

//   pinMode(HALL_PIN, INPUT_PULLUP);
  
//   home();

//   Waypoint trajectory[] = {
//     {0,          0,      0},        // start at 0 rad, velocity 0 at t=0 ms
//     {PI/2.0,     0,   3000},
//     {0,     0,   6000}
//   };


//   unsigned long motion_start_time = millis();
//   execute_trajectory(trajectory, 3, encoder, stepper);
//   Serial.print("Motion Duration: ");
//   Serial.println(millis() - motion_start_time);

//   stepper.disable();

//   // move_to(PI/2.0, encoder, stepper);
//   // delay(1000);
//   // move_to(0, encoder, stepper);
// }

// void loop() {
// }

#include <Arduino.h>
#include <Trajectory/Trajectory.h>
#include <SerialProtocol/SerialProtocol.h>
#include <Serialization/Serialization.h>

void parse_cmd(uint8_t cmd, const uint8_t* payload, size_t payload_len);

SerialProtocol com(Serial, parse_cmd);

enum Command : uint8_t {
    PING = 0x01,
    HOME = 0x02,
    POS  = 0x03,
    TRAJ = 0x04,
    ACK  = 0xEE,
    NACK = 0xFF
};

void printTrajectory(Trajectory& traj) {
    Serial.println(traj.length);
                for (size_t i = 0; i < traj.length; ++i) {
                    Serial.print("Waypoint ");
                    Serial.print(i);
                    Serial.print(": pos=");
                    Serial.print(traj.waypoints[i].position, 6);
                    Serial.print(", vel=");
                    Serial.print(traj.waypoints[i].velocity, 6);
                    Serial.print(", time=");
                    Serial.println(traj.waypoints[i].timestamp);
                }
}

void parse_cmd(uint8_t cmd, const uint8_t* payload, size_t payload_len) {

        switch (cmd) {
            
            case PING: {
                com.send_packet(ACK);
            }

            case HOME: {
                //Implement homing
                com.send_packet(ACK);
            }

            case POS: {
                float pos = 1.1; // Implement encoder position
                uint8_t payload[4];
                writeFloatLE(payload, pos);
                com.send_packet(POS, payload, 4);
                break;
            }

            case TRAJ: {
                Trajectory traj(payload, payload_len);
                if (traj.length>0) {
                    Serial.println("Successfully deserialized trajectory.");
                    printTrajectory(traj); // Implement trajectory
                } else {
                    Serial.println("Failed to deserialize trajectory.");
                }
                break;
            }

            default: {
                Serial.print("Command: 0x");
                Serial.println(cmd, HEX);
                Serial.print("Payload: ");
                for (int i = 0; i < payload_len; i++) {
                    Serial.print("0x");
                    Serial.print(payload[i], HEX);
                    Serial.print(" ");
                }
                Serial.println("");
                break;
            }
        };
    }

void setup() {
    Serial.begin(115200);
}

void loop() {
    com.read_input();
}