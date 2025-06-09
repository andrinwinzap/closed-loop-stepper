#include <Arduino.h>
#include <WiFi.h>
#include <CD74HC4067.h>
#include <Serialization.h>
#include <SerialProtocol.h>
#include <Trajectory.h>
#include <debug_macro.h>
#include <controller.h>
#include <credentials.h>

HardwareSerial actuator_com_serial(ACTUATOR_COM_PORT);
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

SerialProtocol actuator_com(PROTOCOL_ADDRESS, actuator_com_write_callback);
SerialProtocol client_com(PROTOCOL_ADDRESS, client_com_write_callback);
struct ActuatorStatus
{
    uint8_t status;
    float position;
};

struct RobotStatus
{
    ActuatorStatus actuator_1;
    ActuatorStatus actuator_2;
    ActuatorStatus actuator_3;
    ActuatorStatus actuator_4;
} robot_status;

SemaphoreHandle_t actuator_com_mutex;

void read_actuator_com_serial()
{
    while (actuator_com_serial.available())
    {
        uint8_t c = actuator_com_serial.read();
        actuator_com.feed(c);
    }
}

void actuator_status_loop(void *)
{
    uint8_t address = Byte::Address::ACTUATOR_1;
    bool packet_sent = false;
    unsigned long packet_timestamp;

    for (;;)
    {
        if (!packet_sent)
        {
            xSemaphoreTake(actuator_com_mutex, portMAX_DELAY);
            mux.channel(Byte::mux_channel(address));
            actuator_com.send_packet(address, Byte::Command::STATUS);
            packet_sent = true;
            packet_timestamp = millis();
        }

        read_actuator_com_serial();

        if (actuator_com.available())
        {
            const Command *cmd = actuator_com.read();
            if (cmd && cmd->cmd == Byte::Command::STATUS)
            {
                switch (address)
                {
                    ActuatorStatus status = {
                        .status = cmd->payload[0],
                        .position = readFloatLE(&cmd->payload[1])};
                case Byte::Address::ACTUATOR_1:
                    robot_status.actuator_1 = status;
                    address = Byte::Address::ACTUATOR_2;
                    break;
                case Byte::Address::ACTUATOR_2:
                    robot_status.actuator_2 = status;
                    address = Byte::Address::ACTUATOR_3;
                    break;
                case Byte::Address::ACTUATOR_3:
                    robot_status.actuator_3 = status;
                    address = Byte::Address::ACTUATOR_4;
                    break;
                case Byte::Address::ACTUATOR_4:
                    robot_status.actuator_4 = status;
                    address = Byte::Address::ACTUATOR_1;
                    break;
                default:
                    break;
                }
                xSemaphoreGive(actuator_com_mutex);
                packet_sent = false;
            }
        }
        else if (millis() - packet_timestamp > SERIAL_PROTOCOL_TIMEOUT)
        {
            DBG_PRINTLN("ACTUATOR STATUS LOOP TIMED OUT");
            packet_sent = false;
            xSemaphoreGive(actuator_com_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool ping(uint8_t addr)
{
    mux.channel(Byte::mux_channel(addr));
    actuator_com.send_packet(addr, Byte::Command::PING);
    unsigned long start = millis();
    while (millis() - start < SERIAL_PROTOCOL_TIMEOUT)
    {
        read_actuator_com_serial();
        if (actuator_com.available())
        {
            const Command *cmd = actuator_com.read();
            if (cmd)
            {
                if (cmd->cmd == Byte::Command::ACK)
                    return true;
                else if (cmd->cmd == Byte::Command::NACK)
                    return false;
            }
        }
    }
    DBG_PRINT("[CMD] Ping on address: ");
    DBG_PRINT(addr);
    DBG_PRINTLN(" timed out.");
    return false;
}

bool ping()
{
    return ping(Byte::Address::ACTUATOR_1) &&
           ping(Byte::Address::ACTUATOR_2) &&
           ping(Byte::Address::ACTUATOR_3) &&
           ping(Byte::Address::ACTUATOR_4);
}

bool pos(uint8_t addr, float &position)
{
    mux.channel(Byte::mux_channel(addr));
    actuator_com.send_packet(addr, Byte::Command::POS);
    unsigned long start = millis();
    while (millis() - start < SERIAL_PROTOCOL_TIMEOUT)
    {
        read_actuator_com_serial();
        if (actuator_com.available())
        {
            const Command *cmd = actuator_com.read();
            if (cmd)
            {
                if (cmd->cmd == Byte::Command::POS)
                {
                    position = readFloatLE(cmd->payload);
                    return true;
                }
                else if (cmd->cmd == Byte::Command::NACK)
                    return false;
            }
        }
    }
    DBG_PRINT("[CMD] Pos on address: ");
    DBG_PRINT(addr);
    DBG_PRINTLN(" timed out.");
    return false;
}

bool pos(RobotPosition &robot_position)
{
    return pos(Byte::Address::ACTUATOR_1, robot_position.theta_1) &&
           pos(Byte::Address::ACTUATOR_2, robot_position.theta_2) &&
           pos(Byte::Address::ACTUATOR_3, robot_position.theta_3) &&
           pos(Byte::Address::ACTUATOR_4, robot_position.theta_4);
}

bool estop(uint8_t addr)
{
    mux.channel(Byte::mux_channel(addr));
    actuator_com.send_packet(addr, Byte::Command::ESTOP);
    unsigned long start = millis();
    while (millis() - start < SERIAL_PROTOCOL_TIMEOUT)
    {
        read_actuator_com_serial();
        if (actuator_com.available())
        {
            const Command *cmd = actuator_com.read();
            if (cmd)
            {
                if (cmd->cmd == Byte::Command::ACK)
                    return true;
                else if (cmd->cmd == Byte::Command::NACK)
                    return false;
            }
        }
    }
    DBG_PRINT("[CMD] Estop on address: ");
    DBG_PRINT(addr);
    DBG_PRINTLN(" timed out.");
    return false;
}

bool estop()
{
    return estop(Byte::Address::ACTUATOR_1) &&
           estop(Byte::Address::ACTUATOR_2) &&
           estop(Byte::Address::ACTUATOR_3) &&
           estop(Byte::Address::ACTUATOR_4);
}

bool load_traj(uint8_t addr, ActuatorTrajectory &trajectory)
{
    mux.channel(Byte::mux_channel(addr));
    uint16_t payload_len = 1 + trajectory.length * 12;
    uint8_t payload[payload_len];
    trajectory.serialize(payload, payload_len);
    actuator_com.send_packet(addr, Byte::Command::LOAD_TRAJ, payload, payload_len);
    unsigned long start = millis();
    while (millis() - start < SERIAL_PROTOCOL_TIMEOUT)
    {
        read_actuator_com_serial();
        if (actuator_com.available())
        {
            const Command *cmd = actuator_com.read();
            if (cmd)
            {
                if (cmd->cmd == Byte::Command::ACK)
                    return true;
                else if (cmd->cmd == Byte::Command::NACK)
                    return false;
            }
        }
    }
    DBG_PRINT("[CMD] Load trajectory on address: ");
    DBG_PRINT(addr);
    DBG_PRINTLN(" timed out.");
    return false;
}

bool load_traj(RobotTrajectory &trajectory)
{
    return load_traj(Byte::Address::ACTUATOR_1, trajectory.actuator_1) &&
           load_traj(Byte::Address::ACTUATOR_2, trajectory.actuator_2) &&
           load_traj(Byte::Address::ACTUATOR_3, trajectory.actuator_3) &&
           load_traj(Byte::Address::ACTUATOR_4, trajectory.actuator_4);
}

bool exec_traj(uint8_t addr)
{
    mux.channel(Byte::mux_channel(addr));
    actuator_com.send_packet(addr, Byte::Command::EXEC_TRAJ);
    unsigned long start = millis();
    while (millis() - start < SERIAL_PROTOCOL_TIMEOUT)
    {
        read_actuator_com_serial();
        if (actuator_com.available())
        {
            const Command *cmd = actuator_com.read();
            if (cmd)
            {
                if (cmd->cmd == Byte::Command::ACK)
                    return true;
                else if (cmd->cmd == Byte::Command::NACK)
                    return false;
            }
        }
    }
    DBG_PRINT("[CMD] Exec trajectory on address: ");
    DBG_PRINT(addr);
    DBG_PRINTLN(" timed out.");
    return false;
}

bool exec_traj()
{
    return exec_traj(Byte::Address::ACTUATOR_1) &&
           exec_traj(Byte::Address::ACTUATOR_2) &&
           exec_traj(Byte::Address::ACTUATOR_3) &&
           exec_traj(Byte::Address::ACTUATOR_4);
}

void parse_cmd(uint8_t cmd, const uint8_t *payload, size_t payload_len)
{
    switch (cmd)
    {
    case Byte::Command::PING:
    {
        DBG_PRINTLN("[CMD] PING");
        bool result = ping();
        uint8_t response;
        if (result)
        {
            response = Byte::Command::ACK;
        }
        else
        {
            response = Byte::Command::NACK;
        }
        client_com.send_packet(Byte::Address::BROADCAST, response);
        break;
    }
    case Byte::Command::ESTOP:
    {
        DBG_PRINTLN("[CMD] ESTOP");
        bool result = estop();
        uint8_t response;
        if (result)
        {
            response = Byte::Command::ACK;
        }
        else
        {
            response = Byte::Command::NACK;
        }
        client_com.send_packet(Byte::Address::BROADCAST, response);
        break;
    }
    case Byte::Command::POS:
    {
        DBG_PRINTLN("[CMD] POS");
        RobotPosition robot_position;
        bool result = pos(robot_position);
        uint8_t response;
        if (result)
        {
            uint8_t payload[24];
            robot_position.serialize(payload, 24);
            client_com.send_packet(Byte::Address::BROADCAST, Byte::Command::POS, payload, 24);
        }
        else
        {
            client_com.send_packet(Byte::Address::BROADCAST, Byte::Command::NACK);
        }
        break;
    }
    case Byte::Command::LOAD_TRAJ:
    {
        DBG_PRINTLN("[CMD] LOAD_TRAJ");

        RobotTrajectory trajectory(payload, payload_len);

        bool result = load_traj(trajectory);
        uint8_t response;
        if (result)
        {
            response = Byte::Command::ACK;
        }
        else
        {
            response = Byte::Command::NACK;
        }
        client_com.send_packet(Byte::Address::BROADCAST, response);
        break;
    }
    case Byte::Command::EXEC_TRAJ:
    {
        DBG_PRINTLN("[CMD] EXEC_TRAJ");
        bool result = exec_traj();
        uint8_t response;
        if (result)
        {
            response = Byte::Command::ACK;
        }
        else
        {
            response = Byte::Command::NACK;
        }
        client_com.send_packet(Byte::Address::BROADCAST, response);
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
    actuator_com_serial.begin(ACTUATOR_COM_BAUD, SERIAL_8N1, ACTUATOR_COM_RX, ACTUATOR_COM_TX);

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

    actuator_com_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(actuator_com_mutex);

    xTaskCreatePinnedToCore(
        actuator_status_loop, // function
        "ActuatorStatusLoop", // name
        4096,                 // stack
        nullptr,              // params
        1,                    // priority
        nullptr,              // handle
        0                     // CORE 0
    );

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
        client_com.feed(tcp_client.read());
    }

    if (client_com.available() > 0)
    {
        const Command *cmd = client_com.read();
        if (cmd)
        {
            xSemaphoreTake(actuator_com_mutex, portMAX_DELAY);
            parse_cmd(cmd->cmd, cmd->payload, cmd->payload_len);
            xSemaphoreGive(actuator_com_mutex);
        }
    }

    read_actuator_com_serial();
}