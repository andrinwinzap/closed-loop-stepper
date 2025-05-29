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

constexpr uint8_t START_BYTE = 0xAA;
constexpr uint8_t ESCAPE_BYTE = 0xAB;
constexpr uint8_t ESCAPE_MASK = 0x20;

constexpr size_t PAYLOAD_BUFFER_SIZE = 1024;

enum Command : uint8_t {
    PING = 0x01,
    HOME = 0x02,
    POS  = 0x03,
    TRAJ = 0x04,
    ACK  = 0xEE,
    NACK = 0xFF
};

enum class State {
    WAIT_START,
    READ_CMD,
    READ_LEN,
    READ_PAYLOAD,
    READ_CHECKSUM,
    VALIDATE,
    DISPATCH
};

void updateCrc8(uint8_t& crc8, uint8_t byte) {
        crc8 ^= byte;
        for (int i = 0; i < 8; ++i) {
            crc8 = (crc8 & 0x80)
                  ? (crc8 << 1) ^ 0x07
                  : (crc8 << 1);
        }
    }

uint8_t crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
        updateCrc8(crc, data[i]);
    }
    return crc;
}

size_t escape_data(uint8_t* data, size_t length, uint8_t* output, size_t max_output_len) {
    size_t output_index = 0;
    for (size_t i = 0; i<length; i++) {
        uint8_t b = data[i];
        if (b == START_BYTE || b == ESCAPE_BYTE) {
            if (output_index + 2 > max_output_len) break;  // prevent overflow
            output[output_index++] = ESCAPE_BYTE;
            output[output_index++] = b ^ ESCAPE_MASK;
        } else {
            if (output_index + 1 > max_output_len) break;
            output[output_index++] = b;
        }
    }
    return output_index; 
}

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

class SerialParser {
public:
    State state = State::WAIT_START;
    void parse(uint8_t byte) {

        if (byte == START_BYTE) {
            reset();
            state = State::READ_CMD;
            return;
        }

        if (state != State::WAIT_START) {
            if (escapeNext) {
                byte ^= ESCAPE_MASK;
                escapeNext = false;
            } else if (byte == ESCAPE_BYTE) {
                escapeNext = true;
                return;
            }
        }

        switch (state) {
            case State::READ_CMD:
                cmd = byte;
                updateCrc8(crc8_acc, byte);
                state = State::READ_LEN;
                break;

            case State::READ_LEN:
                if (len_bytes_read == 0) {
                    len = byte;
                    updateCrc8(crc8_acc, byte);
                    len_bytes_read = 1;
                } else {
                    len |= (byte << 8);
                    updateCrc8(crc8_acc, byte);
                    len_bytes_read = 0;
                    payload_bytes_read = 0;

                    if (len > 0 && len <= PAYLOAD_BUFFER_SIZE) {
                        state = State::READ_PAYLOAD;
                    } else if (len == 0) {
                        state = State::READ_CHECKSUM;
                    } else {
                        Serial.println("Payload too large!");
                        reset();
                    }
                }
                break;

            
            case State::READ_PAYLOAD:
                updateCrc8(crc8_acc, byte);
                if (payload_buffer_len < PAYLOAD_BUFFER_SIZE) payload_buffer[payload_buffer_len++] = byte;
                payload_bytes_read++;
                if (payload_bytes_read >= len) {
                   state = State::READ_CHECKSUM;
                }
                break;

            case State::READ_CHECKSUM:
                checksum = byte;
                State::VALIDATE;
                validate();
                break;

            default:
                reset();
        }
    }
private:
    uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];
    size_t payload_buffer_len = 0;
    size_t payload_bytes_read = 0;
    uint16_t len;
    uint8_t len_bytes_read = 0;

    uint8_t cmd, checksum;
    uint8_t crc8_acc = 0x00;
    bool escapeNext = false;

    void validate() {
        if (crc8_acc == checksum) {
            dispatch();
        } else {
            Serial.println("Checksum failed!");
        }
    }

    void dispatch() {
        if (cmd == TRAJ) {
            Trajectory traj(payload_buffer, payload_buffer_len);
            if (traj.length>0) {
                Serial.println("Successfully deserialized trajectory.");
                printTrajectory(traj);
            } else {
                Serial.println("Failed to deserialize trajectory.");
            }
        } else {
            Serial.print("Command: 0x");
            Serial.println(cmd, HEX);
            Serial.print("Payload: ");
            for (int i = 0; i < payload_buffer_len; i++) {
                Serial.print("0x");
                Serial.print(payload_buffer[i], HEX);
                Serial.print(" ");
            }
            Serial.println("");
        }
    }

    void reset() {
        state = State::WAIT_START;
        payload_buffer_len = 0;
        payload_bytes_read = 0;
        len = 0;
        len_bytes_read = 0;
        crc8_acc = 0x00;
        escapeNext = false;
    }

};

SerialParser parser;

void setup() {


    Serial.begin(115200);
}

void loop() {
    while (Serial.available() > 0) {
    uint8_t byte = Serial.read();
    parser.parse(byte);
  }
}