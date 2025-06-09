#include <Arduino.h>
#include <AS5600.h>
#include "ClosedLoop.h"
#include <Serialization.h>
#include <SerialProtocol.h>
#include <Stepper.h>
#include <Trajectory.h>
#include <debug_macro.h>
#include <actuator_configuration_macro.h>

HardwareSerial master_com_serial(MASTER_COM_PORT);

void master_com_callback(const uint8_t *data, size_t len);
SerialProtocol master_com(PROTOCOL_ADDRESS, master_com_callback);

AS5600 encoder(GEAR_RATIO);
Stepper stepper(STEPPER_STEP_PIN, STEPPER_DIR_PIN, STEPPER_EN_PIN, GEAR_RATIO);
volatile ControlLoop::State control_loop_state;
volatile ControlLoop::Flag control_loop_flag;

ActuatorTrajectory *trajectory = nullptr;
float target_position;

ControlLoop::Params control_loop_params = {
    .stepper = &stepper,
    .encoder = &encoder,
    .state = &control_loop_state,
    .flag = &control_loop_flag,
    .trajectory = &trajectory,
    .target_position = &target_position};

TaskHandle_t controlTaskHandle = nullptr;

void master_com_callback(const uint8_t *data, size_t len)
{
    master_com_serial.write(data, len);
}

void parse_cmd(uint8_t cmd, const uint8_t *payload, size_t payload_len)
{
    switch (cmd)
    {
    case Byte::Command::PING:
    {
        DBG_PRINTLN("[CMD] PING");
        master_com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }

    case Byte::Command::ESTOP:
    {
        DBG_PRINTLN("[CMD] ESTOP");
        if (!DUMMY_MODE)
            control_loop_flag = ControlLoop::Flag::ESTOP;
        master_com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }

    case Byte::Command::POS:
    {
        DBG_PRINTLN("[CMD] POS");
        float pos;
        if (DUMMY_MODE)
            pos = 123.456f;
        else
            pos = encoder.getPosition();
        DBG_PRINT("[CMD] Encoder position: ");
        DBG_PRINTLN(pos);
        uint8_t payload[4];
        writeFloatLE(payload, pos);
        master_com.send_packet(Byte::Address::MASTER, Byte::Command::POS, payload, 4);
        break;
    }
    case Byte::Command::LOAD_TRAJ:
    {
        DBG_PRINTLN("[CMD] LOAD_TRAJ");

        if (trajectory != nullptr)
        {
            DBG_PRINTLN("[CMD] Deleting existing trajectory");
            delete trajectory;
            trajectory = nullptr;
        }

        trajectory = new ActuatorTrajectory(payload, payload_len);
        DBG_PRINT("[CMD] New trajectory length: ");
        DBG_PRINTLN(trajectory->length);

        if (trajectory->length == 0)
        {
            DBG_PRINTLN("[CMD] Trajectory length is 0, discarding");
            delete trajectory;
            trajectory = nullptr;
        }
        else
        {
            DBG_PRINTLN("[CMD] Trajectory:");
            for (size_t i = 0; i < trajectory->length; ++i)
            {
                DBG_PRINT("[CMD] Waypoint ");
                DBG_PRINT(i);
                DBG_PRINT(": pos=");
                DBG_PRINT(trajectory->waypoints[i].position, 6);
                DBG_PRINT(", vel=");
                DBG_PRINT(trajectory->waypoints[i].velocity, 6);
                DBG_PRINT(", time=");
                DBG_PRINTLN(trajectory->waypoints[i].timestamp);
            }
        }

        master_com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }
    case Byte::Command::EXEC_TRAJ:
    {
        DBG_PRINTLN("[CMD] EXEC_TRAJ");

        if (trajectory == nullptr)
        {
            DBG_PRINTLN("[CMD] No trajectory loaded. Skipping execution.");
            break;
        }

        if (!DUMMY_MODE)
            control_loop_flag = ControlLoop::Flag::TRAJECTORY;
        DBG_PRINTLN("[CMD] Trajectory execution triggered");
        master_com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }
    case Byte::Command::STATUS:
    {
        uint8_t payload[5];
        writeFloatLE(&payload[1], encoder.getPosition());
        switch (control_loop_state)
        {
        case ControlLoop::State::IDLE:
        {

            payload[0] = Byte::Status::IDLE;
            master_com.send_packet(Byte::Address::MASTER, Byte::Command::STATUS, payload, 5);
            break;
        }
        case ControlLoop::State::HOMING:
        {
            payload[0] = Byte::Status::HOMING;
            master_com.send_packet(Byte::Address::MASTER, Byte::Command::STATUS, payload, 5);
            break;
        }
        case ControlLoop::State::TRAJECTORY:
        {
            payload[0] = Byte::Status::TRAJECTORY;
            master_com.send_packet(Byte::Address::MASTER, Byte::Command::STATUS, payload, 5);
            break;
        }
        case ControlLoop::State::POSITION:
        {
            payload[0] = Byte::Status::POSITION;
            master_com.send_packet(Byte::Address::MASTER, Byte::Command::STATUS, payload, 5);
            break;
        }
        case ControlLoop::State::ESTOP:
        {
            payload[0] = Byte::Status::ESTOP;
            master_com.send_packet(Byte::Address::MASTER, Byte::Command::STATUS, payload, 5);
            break;
        }
        default:
            break;
        }
        break;
    }
    default:
        DBG_PRINT("[CMD] Unknown command: 0x");
        DBG_PRINTLN(cmd, HEX);
        DBG_PRINT("[CMD] Payload: ");
        for (int i = 0; i < payload_len; i++)
        {
            DBG_PRINT("0x");
            DBG_PRINT(payload[i], HEX);
            DBG_PRINT(" ");
        }
        DBG_PRINTLN("");
        break;
    };
}

void setup()
{
#ifdef DEBUG_OUTPUT
    Serial.begin(DEBUG_SERIAL_BAUD);
#endif
    master_com_serial.begin(MASTER_COM_BAUD, SERIAL_8N1, MASTER_COM_RX, MASTER_COM_TX);

    DBG_PRINTLN("[SETUP] Starting setup");

    if (!DUMMY_MODE)
    {
        Wire.begin();

        if (!encoder.begin())
        {
            DBG_PRINTLN("[SETUP] Failed to start AS5600!!");
            while (1)
                ;
        }

        DBG_PRINTLN("[SETUP] AS5600 ready");

        stepper.begin();
        stepper.disable();
        stepper.setMicrosteps(STEPPER_MICROSTEPS);
        stepper.setStepsPerRevolution(STEPPER_STEPS_PER_REVOLUTION);
        stepper.setMaxSpeed(1000000);

        DBG_PRINTLN("[SETUP] Stepper initialized");

        pinMode(HALL_EFFECT_SENSOR_PIN, INPUT_PULLUP);

        xTaskCreatePinnedToCore(
            ControlLoop::task,
            "ControlLoopTask",
            4096,
            &control_loop_params,
            1,
            &controlTaskHandle,
            0 // Core 0
        );
    }
    control_loop_flag = ControlLoop::Flag::IDLE;
    DBG_PRINTLN("[SETUP] Setup complete");
}

void loop()
{
    while (master_com_serial.available())
    {
        master_com.feed(master_com_serial.read());
    }
    if (master_com.available() > 0)
    {
        const Command *packet = master_com.read();
        if (packet)
        {
            parse_cmd(packet->cmd, packet->payload, packet->payload_len);
        }
    }
}