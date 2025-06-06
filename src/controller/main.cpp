#include <Arduino.h>
#include <WiFi.h>
#include <CD74HC4067.h>
#include <Serialization.h>
#include <SerialProtocol.h>
#include <Trajectory.h>
#include <debug_macro.h>
#include <controller.h>
#include <credentials.h>

HardwareSerial actuator_com_serial(2);
void actuator_com_write_callback(const uint8_t *data, size_t len);
void client_com_write_callback(const uint8_t *data, size_t len);

SerialProtocol actuator_com(PROTOCOL_ADDRESS, actuator_com_write_callback);
SerialProtocol client_com(PROTOCOL_ADDRESS, client_com_write_callback);

CD74HC4067 mux(MUX_S0, MUX_S1, MUX_S2, MUX_S3);

WiFiServer tcp_server(TCP_LISTEN_PORT);
WiFiClient tcp_client;

void actuator_com_write_callback(const uint8_t *data, size_t len)
{
    actuator_com_serial.write(data, len);
}

void client_com_write_callback(const uint8_t *data, size_t len)
{
    if (tcp_client && tcp_client.connected())
    {
        tcp_client.write(data, len);
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
    case Byte::Command::ESTOP:
    {
        DBG_PRINTLN("[CMD] ESTOP");
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
    actuator_com_serial.begin(115200, SERIAL_8N1, RXD2, TXD2);

    pinMode(MUX_SIG, OUTPUT);
    mux.channel(0);

    DBG_PRINT("[SETUP] Connecting to Wi-Fi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long wifi_start = millis();

    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - wifi_start > WIFI_CONNECT_TIMEOUT)
        {
            DBG_PRINTLN("\n[ERROR] Wi-Fi connection failed.");
            break;
        }
        delay(200);
        DBG_PRINT(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        DBG_PRINTLN("\n[SETUP] Wi-Fi connected.");
        DBG_PRINT("[SETUP] IP = ");
        DBG_PRINTLN(WiFi.localIP());
    }
    else
    {
        DBG_PRINTLN("[SETUP] Starting AP mode...");
        WiFi.mode(WIFI_AP);

        if (WiFi.softAP(AP_SSID, AP_PASS))
        {
            IPAddress IP = WiFi.softAPIP();
            DBG_PRINT("[SETUP] AP IP address: ");
            DBG_PRINTLN(IP);
        }
        else
        {
            DBG_PRINTLN("[ERROR] Failed to start AP mode.");
        }
    }

    tcp_server.begin();
    tcp_server.setNoDelay(true);
    DBG_PRINT("[SETUP] TCP Server started on port ");
    DBG_PRINTLN(TCP_LISTEN_PORT);

    DBG_PRINTLN("[SETUP] Setup complete");
}

void loop()
{
    if (!tcp_client || !tcp_client.connected())
    {
        WiFiClient newClient = tcp_server.available();
        if (newClient)
        {
            if (tcp_client && tcp_client.connected())
            {
                tcp_client.stop();
                DBG_PRINTLN("[LOOP] Dropped previous client.");
            }
            tcp_client = newClient;
            tcp_client.setNoDelay(true);
            DBG_PRINT("[LOOP] Client connected from ");
            DBG_PRINT(tcp_client.remoteIP());
            DBG_PRINT(":");
            DBG_PRINTLN(tcp_client.remotePort());
        }
        else
        {
            delay(10);
            return;
        }
    }

    while (tcp_client && tcp_client.connected() && tcp_client.available())
    {
        uint8_t byteReceived = tcp_client.read();
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

    while (actuator_com_serial.available())
    {
        uint8_t c = actuator_com_serial.read();
        actuator_com.feed(c);
    }
}