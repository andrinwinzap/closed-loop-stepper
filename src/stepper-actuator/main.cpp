#include <Arduino.h>
#include <AS5600.h>
#include "ClosedLoop.h"
#include <Serialization.h>
#include <SerialProtocol.h>
#include <Stepper.h>
#include <Trajectory.h>
#include <debug_macro.h>
#include <actuator_configuration_macro.h>

HardwareSerial com_serial(2);
SerialProtocol com(com_serial, PROTOCOL_ADDRESS);

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

void parse_cmd(uint8_t cmd, const uint8_t *payload, size_t payload_len)
{
    switch (cmd)
    {
    case Byte::Command::PING:
    {
        DBG_PRINTLN("[CMD] PING");
        com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }
    case Byte::Command::HOME:
    {
        DBG_PRINTLN("[CMD] HOME");
        com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }
    case Byte::Command::POS:
    {
        DBG_PRINTLN("[CMD] POS");
        float pos = encoder.getPosition();
        DBG_PRINT("[CMD] Encoder position: ");
        DBG_PRINTLN(pos);
        uint8_t payload[4];
        writeFloatLE(payload, pos);
        com.send_packet(Byte::Address::MASTER, Byte::Command::POS, payload, 4);
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

        com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
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

        control_loop_flag = ControlLoop::Flag::TRAJECTORY;
        DBG_PRINTLN("[CMD] Trajectory execution triggered");
        com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }
    case Byte::Command::STATUS:
    {

        switch (control_loop_state)
        {
        case ControlLoop::State::IDLE:
        {
            com.send_packet(Byte::Address::MASTER, Byte::Command::STATUS, Byte::Status::IDLE);
            break;
        }
        case ControlLoop::State::HOMING:
        {
            com.send_packet(Byte::Address::MASTER, Byte::Command::STATUS, Byte::Status::HOMING);
            break;
        }
        case ControlLoop::State::TRAJECTORY:
        {
            com.send_packet(Byte::Address::MASTER, Byte::Command::STATUS, Byte::Status::EXECUTING_TRAJECTORY);
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
    Serial.begin(115200);
    com_serial.begin(115200, SERIAL_8N1, RXD2, TXD2);

    DBG_PRINTLN("[SETUP] Starting setup");

    Wire.begin();

    if (!encoder.begin())
    {
        DBG_PRINTLN("[SETUP] AS5600 not found!");
        while (1)
            ;
    }

    DBG_PRINTLN("[SETUP] AS5600 ready");

    stepper.begin();
    stepper.enable();
    stepper.setMicrosteps(STEPPER_MICROSTEPS);
    stepper.setStepsPerRevolution(STEPPER_STEPS_PER_REVOLUTION);
    stepper.setMaxSpeed(1000000);

    DBG_PRINTLN("[SETUP] Stepper initialized");

    pinMode(HALL_EFFECT_SENSOR_PIN, INPUT_PULLUP);

    control_loop_flag = ControlLoop::Flag::IDLE;

    xTaskCreatePinnedToCore(
        ControlLoop::task,
        "ControlLoopTask",
        4096,
        &control_loop_params,
        1,
        &controlTaskHandle,
        0 // Core 0
    );

    DBG_PRINTLN("[SETUP] Setup complete");
}

void loop()
{
    if (com.available() > 0)
    {
        const Command *cmd = com.read();
        if (cmd)
        {
            parse_cmd(cmd->cmd, cmd->payload, cmd->payload_len);
        }
    }
}
