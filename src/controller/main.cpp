#include <Arduino.h>
#include <WiFi.h>
#include <CD74HC4067.h>
#include <Serialization.h>
#include <SerialProtocol.h>
#include <Trajectory.h>
#include <debug_macro.h>
#include <controller.h>
#include <credentials.h>

HardwareSerial com_serial(2);
void actuator_com_write_callback(const uint8_t *data, size_t len);
void client_com_write_callback(const uint8_t *data, size_t len);

SerialProtocol actuator_com(PROTOCOL_ADDRESS, actuator_com_write_callback);
SerialProtocol client_com(PROTOCOL_ADDRESS, client_com_write_callback);

CD74HC4067 mux(MUX_S0, MUX_S1, MUX_S2, MUX_S3);

WiFiServer tcpServer(TCP_LISTEN_PORT);
WiFiClient tcpClient;

void actuator_com_write_callback(const uint8_t *data, size_t len)
{
    com_serial.write(data, len);
}

void client_com_write_callback(const uint8_t *data, size_t len)
{
    if (tcpClient && tcpClient.connected())
    {
        tcpClient.write(data, len);
    }
}

float get_pos(uint8_t addr)
{
    mux.channel(Byte::mux_channel(addr));
    actuator_com.send_packet(addr, Byte::Command::POS);
    unsigned long start = millis();
    while (1)
    {
        if (millis() - start > SERIAL_PROTOCOL_TIMEOUT)
            return 0.0f;
        if (actuator_com.available())
        {
            const Command *cmd = actuator_com.read();
            if (cmd)
            {
                if (cmd->cmd == Byte::Command::POS)
                    return readFloatLE(cmd->payload);
            }
        }
    }
}

RobotPosition get_pos()
{
    RobotPosition pos = {
        .theta_1 = get_pos(Byte::Address::ACTUATOR_1),
        .theta_2 = get_pos(Byte::Address::ACTUATOR_2),
        .theta_3 = get_pos(Byte::Address::ACTUATOR_3),
        .theta_4 = get_pos(Byte::Address::ACTUATOR_4),
        .theta_5 = get_pos(Byte::Address::ACTUATOR_5),
        .theta_6 = get_pos(Byte::Address::ACTUATOR_6)};
    return pos;
}

float ping(uint8_t addr)
{
    mux.channel(Byte::mux_channel(addr));
    actuator_com.send_packet(addr, Byte::Command::PING);
    unsigned long start = millis();
    while (1)
    {
        if (millis() - start > SERIAL_PROTOCOL_TIMEOUT)
            return false;
        if (actuator_com.available())
        {
            const Command *cmd = actuator_com.read();
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
        client_com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }
    case Byte::Command::HOME:
    {
        DBG_PRINTLN("[CMD] HOME");
        // NOT IMPLEMENTED
        client_com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }
    case Byte::Command::POS:
    {
        DBG_PRINTLN("[CMD] POS");
        // NOT IMPLEMENTED
        client_com.send_packet(Byte::Address::MASTER, Byte::Command::POS, payload, 4);
        break;
    }
    case Byte::Command::LOAD_TRAJ:
    {
        DBG_PRINTLN("[CMD] LOAD_TRAJ");
        // NOT IMPLEMENTED
        client_com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
        break;
    }
    case Byte::Command::EXEC_TRAJ:
    {
        DBG_PRINTLN("[CMD] EXEC_TRAJ");
        // NOT IMPLEMENTED
        client_com.send_packet(Byte::Address::MASTER, Byte::Command::ACK);
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

    pinMode(MUX_SIG, OUTPUT);
    mux.channel(0);

    DBG_PRINTLN("[SETUP] Connecting to Wi-Fi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - wifiStart > WIFI_CONNECT_TIMEOUT)
        {
            DBG_PRINTLN("[ERROR] Wi-Fi connection failed!");
            break;
        }
        delay(200);
        DBG_PRINT(".");
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        DBG_PRINTLN("\n[SETUP] Wi-Fi connected.");
        DBG_PRINT("        IP = ");
        DBG_PRINTLN(WiFi.localIP());
    }
    else
    {
        DBG_PRINTLN("\n[SETUP] Continuing without Wi-Fi...");
    }

    tcpServer.begin();
    tcpServer.setNoDelay(true);
    DBG_PRINT("[SETUP] TCP Server started on port ");
    DBG_PRINTLN(TCP_LISTEN_PORT);

    DBG_PRINTLN("[SETUP] Setup complete");
}

void loop()
{
    if (!tcpClient || !tcpClient.connected())
    {
        WiFiClient newClient = tcpServer.available();
        if (newClient)
        {
            if (tcpClient && tcpClient.connected())
            {
                tcpClient.stop();
                DBG_PRINTLN("[LOOP] Dropped previous client.");
            }
            tcpClient = newClient;
            tcpClient.setNoDelay(true);
            DBG_PRINT("[LOOP] Client connected from ");
            DBG_PRINT(tcpClient.remoteIP());
            DBG_PRINT(":");
            DBG_PRINTLN(tcpClient.remotePort());
        }
        else
        {
            delay(10);
            return;
        }
    }

    while (tcpClient && tcpClient.connected() && tcpClient.available())
    {
        uint8_t byteReceived = tcpClient.read();
        client_com.feed(byteReceived);
    }

    if (client_com.available() > 0)
    {
        const Command *cmd = client_com.read();
        if (cmd)
        {
            parse_cmd(cmd->cmd, cmd->payload, cmd->payload_len);
        }
    }

    while (com_serial.available())
    {
        uint8_t c = com_serial.read();
        actuator_com.feed(c);
    }
}