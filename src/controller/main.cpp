#include <Arduino.h>
#include <Serialization.h>
#include <SerialProtocol.h>
#include <Trajectory.h>
#include <debug_macro.h>
#include <controller.h>

constexpr unsigned long TIMEOUT = 1000;

HardwareSerial com_serial(2);
SerialProtocol com(com_serial, PROTOCOL_ADDRESS);

struct Pos
{
    float theta_1;
    float theta_2;
    float theta_3;
    float theta_4;
    float theta_5;
    float theta_6;
};

float get_pos(uint8_t addr)
{
    com.send_packet(addr, Byte::Command::POS);
    unsigned long start = millis();
    while (1)
    {
        if (millis() - start > TIMEOUT)
            return 0.0f;
        if (com.available())
        {
            const Command *cmd = com.read();
            if (cmd)
            {
                if (cmd->cmd == Byte::Command::POS)
                    return readFloatLE(cmd->payload);
            }
        }
    }
}

float get_pos()
{
    Pos pos = {
        .theta_1 = get_pos(Byte::Address::ACTUATOR_1),
        .theta_2 = get_pos(Byte::Address::ACTUATOR_2),
        .theta_3 = get_pos(Byte::Address::ACTUATOR_3),
        .theta_4 = get_pos(Byte::Address::ACTUATOR_4),
        .theta_5 = get_pos(Byte::Address::ACTUATOR_5),
        .theta_6 = get_pos(Byte::Address::ACTUATOR_6)};
}

float ping(uint8_t addr)
{
    com.send_packet(addr, Byte::Command::PING);
    unsigned long start = millis();
    while (1)
    {
        if (millis() - start > TIMEOUT)
            return false;
        if (com.available())
        {
            const Command *cmd = com.read();
            if (cmd)
            {
                if (cmd->cmd == Byte::Command::POS)
                    return true;
            }
        }
    }
}

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
        // NOT IMPLEMENTED
        com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }
    case Byte::Command::POS:
    {
        DBG_PRINTLN("[CMD] POS");
        // NOT IMPLEMENTED
        com.send_packet(Byte::Address::MASTER, Byte::Command::POS, payload, 4);
        break;
    }
    case Byte::Command::LOAD_TRAJ:
    {
        DBG_PRINTLN("[CMD] LOAD_TRAJ");
        // NOT IMPLEMENTED
        com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }
    case Byte::Command::EXEC_TRAJ:
    {
        DBG_PRINTLN("[CMD] EXEC_TRAJ");
        // NOT IMPLEMENTED
        com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
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
