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
#include <macros.h>

SerialProtocol com(Serial);

Trajectory *trajectory = nullptr;

enum cmdByte : uint8_t {
    PING = 0x01,
    HOME = 0x02,
    POS  = 0x03,
    LOAD_TRAJ = 0x04,
    EXEC_TRAJ = 0x05,
    ACK  = 0xEE,
    NACK = 0xFF
};

void printTrajectory(Trajectory& traj) {
    DBG_PRINTLN(traj.length);
                for (size_t i = 0; i < traj.length; ++i) {
                    DBG_PRINT("Waypoint ");
                    DBG_PRINT(i);
                    DBG_PRINT(": pos=");
                    DBG_PRINT(traj.waypoints[i].position, 6);
                    DBG_PRINT(", vel=");
                    DBG_PRINT(traj.waypoints[i].velocity, 6);
                    DBG_PRINT(", time=");
                    DBG_PRINTLN(traj.waypoints[i].timestamp);
                }
}

void parse_cmd(uint8_t cmd, const uint8_t* payload, size_t payload_len) {

        switch (cmd) {
            
            case PING: {
                com.send_packet(ACK);
                break;
            }

            case HOME: {
                //Implement homing
                com.send_packet(ACK);
                break;
            }

            case POS: {
                float pos = 1.1; // Implement encoder position
                uint8_t payload[4];
                writeFloatLE(payload, pos);
                com.send_packet(POS, payload, 4);
                break;
            }

            case LOAD_TRAJ: {

                if (trajectory != nullptr) {
                    delete trajectory;
                    trajectory = nullptr;
                }

                trajectory = new Trajectory(payload, payload_len);

                if (trajectory->length == 0) {
                    delete trajectory;
                    trajectory = nullptr;
                }
                
                com.send_packet(ACK);
                break;

            }

            case EXEC_TRAJ: {
                if (trajectory == nullptr) break;
                printTrajectory(*trajectory); // Implement trajectory execution
                break;

            }

            default: {
                DBG_PRINT("Command: 0x");
                DBG_PRINTLN(cmd, HEX);
                DBG_PRINT("Payload: ");
                for (int i = 0; i < payload_len; i++) {
                    DBG_PRINT("0x");
                    DBG_PRINT(payload[i], HEX);
                    DBG_PRINT(" ");
                }
                DBG_PRINTLN("");
                break;
            }
        };
    }

void setup() {
    Serial.begin(115200);
}

void loop() {
    if (com.available() > 0) {
        const Command* cmd = com.read();
        parse_cmd(cmd->cmd, cmd->payload, cmd->payload_len);
    }
}