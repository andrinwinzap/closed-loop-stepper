#include <Arduino.h>
#include <AS5600/AS5600.h>
#include <ClosedLoop/ClosedLoop.h>
#include <Serialization/Serialization.h>
#include <SerialProtocol/SerialProtocol.h>
#include <Stepper/Stepper.h>
#include <Trajectory/Trajectory.h>
#include <Macros.h>
#include <ProtocolByteDefinitions.h>

HardwareSerial com_serial(2);
SerialProtocol com(com_serial);

AS5600 encoder(GEAR_RATIO);
Stepper stepper(STEPPER_STEP_PIN, STEPPER_DIR_PIN, STEPPER_EN_PIN, GEAR_RATIO);
volatile ControlLoop::Flag control_loop_flag;
Trajectory *trajectory = nullptr;

ControlLoop::Params control_loop_params = {
    .encoder = &encoder,
    .stepper = &stepper,
    .flag = &control_loop_flag,
    .trajectory = &trajectory};

TaskHandle_t controlTaskHandle = nullptr;

void parse_cmd(uint8_t cmd, const uint8_t *payload, size_t payload_len)
{
    switch (cmd)
    {
    case CommandByte::PING:
    {
        DBG_PRINTLN("[CMD] PING");
        com.send_packet(CommandByte::ACK);
        break;
    }
    case CommandByte::HOME:
    {
        DBG_PRINTLN("[CMD] HOME");
        control_loop_flag = ControlLoop::Flag::HOME;
        com.send_packet(CommandByte::ACK);
        break;
    }
    case CommandByte::POS:
    {
        DBG_PRINTLN("[CMD] POS");
        float pos = encoder.getPosition();
        DBG_PRINT("[CMD] Encoder position: ");
        DBG_PRINTLN(pos);
        uint8_t payload[4];
        writeFloatLE(payload, pos);
        com.send_packet(CommandByte::POS, payload, 4);
        break;
    }
    case CommandByte::LOAD_TRAJ:
    {
        DBG_PRINTLN("[CMD] LOAD_TRAJ");

        if (trajectory != nullptr)
        {
            DBG_PRINTLN("[CMD] Deleting existing trajectory");
            delete trajectory;
            trajectory = nullptr;
        }

        trajectory = new Trajectory(payload, payload_len);
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

        com.send_packet(CommandByte::ACK);
        break;
    }
    case CommandByte::EXEC_TRAJ:
    {
        DBG_PRINTLN("[CMD] EXEC_TRAJ");

        if (trajectory == nullptr)
        {
            DBG_PRINTLN("[CMD] No trajectory loaded. Skipping execution.");
            break;
        }

        control_loop_flag = ControlLoop::Flag::EXECUTE_TRAJECTORY;
        DBG_PRINTLN("[CMD] Trajectory execution triggered");
        com.send_packet(CommandByte::ACK);
        break;
    }
    case CommandByte::STATUS:
    {

        switch (ControlLoop::state)
        {
        case ControlLoop::State::IDLE:
        {
            com.send_packet(CommandByte::STATUS, StatusByte::IDLE);
            break;
        }
        case ControlLoop::State::HOMING:
        {
            com.send_packet(CommandByte::STATUS, StatusByte::HOMING);
            break;
        }
        case ControlLoop::State::EXECUTING_TRAJECTORY:
        {
            com.send_packet(CommandByte::STATUS, StatusByte::EXECUTING_TRAJECTORY);
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

    encoder.setUpdateFrequency(ENCODER_UPDATE_FREQUENCY);
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
        ControlLoop::control_loop_task,
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
