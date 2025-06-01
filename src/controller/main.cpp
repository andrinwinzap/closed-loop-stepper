#include <Arduino.h>
#include <Serialization.h>
#include <SerialProtocol.h>
#include <Trajectory.h>
#include <debug_macro.h>
#include <controller.h>

HardwareSerial com_serial(2);
SerialProtocol com(com_serial, PROTOCOL_ADDRESS);

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
