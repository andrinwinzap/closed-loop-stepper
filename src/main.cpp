#include <Arduino.h>
#include <AS5600/AS5600.h>
#include <ClosedLoop/ClosedLoop.h>
#include <Serialization/Serialization.h>
#include <SerialProtocol/SerialProtocol.h>
#include <Stepper/Stepper.h>
#include <Trajectory/Trajectory.h>
#include <macros.h>

#define HALL_PIN 15
constexpr float GEAR_RATIO = 15.0;

AS5600 encoder(GEAR_RATIO);
Stepper stepper(17, 16, 4, GEAR_RATIO);

SerialProtocol com(Serial);

Trajectory *trajectory = nullptr;

enum cmdByte : uint8_t
{
    PING = 0x01,
    HOME = 0x02,
    POS = 0x03,
    LOAD_TRAJ = 0x04,
    EXEC_TRAJ = 0x05,
    ACK = 0xEE,
    NACK = 0xFF
};

void home()
{

    const float homing_speed = 0.3;
    const float homing_acceleration = 0.5;

    DBG_PRINTLN("Starting homing process...");

    stepper.setSpeed(-homing_speed);
    stepper.move(-PI / 8.0, homing_speed, homing_acceleration);

    float first_bound = 0;

    stepper.start();
    stepper.accelerate(0, homing_speed, homing_acceleration);

    float filteredHallSensorValue = analogRead(HALL_PIN);
    float hallSensorAlpha = 0.2;
    float hallSensorUpdateFrequency = 1000;

    float hallSensorUpdatePeriod = 1e6 / hallSensorUpdateFrequency;

    while (true)
    {

        stepper.updateAcceleration();
        int raw = analogRead(HALL_PIN);
        filteredHallSensorValue = hallSensorAlpha * raw + (1 - hallSensorAlpha) * filteredHallSensorValue;

        if (filteredHallSensorValue < 1)
            break;

        delayMicroseconds(hallSensorUpdatePeriod);
    }

    first_bound = encoder.getPosition();

    while (true)
    {

        int raw = analogRead(HALL_PIN);
        filteredHallSensorValue = hallSensorAlpha * raw + (1 - hallSensorAlpha) * filteredHallSensorValue;

        if (filteredHallSensorValue > 1)
            break;

        delayMicroseconds(hallSensorUpdatePeriod);
    }

    float current_pos = encoder.getPosition();
    float home = (current_pos - first_bound) / 2;
    encoder.setPosition(current_pos - home);

    stepper.accelerate(homing_speed, 0, homing_acceleration);
    while (stepper.updateAcceleration())
    {
        unsigned long start = micros();
        encoder.update();
        DBG_PRINTLN(micros() - start);
    }
    move_to(0, encoder, stepper);
    DBG_PRINTLN("Finished homing");
    DBG_PRINTLN(encoder.getPosition());

    delay(1000);
}

void printTrajectory(Trajectory &traj)
{
    DBG_PRINTLN(traj.length);
    for (size_t i = 0; i < traj.length; ++i)
    {
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

void parse_cmd(uint8_t cmd, const uint8_t *payload, size_t payload_len)
{
    switch (cmd)
    {

    case PING:
    {
        com.send_packet(ACK);
        break;
    }

    case HOME:
    {
        com.send_packet(ACK);
        home();
        break;
    }

    case POS:
    {
        float pos = encoder.getPosition();
        uint8_t payload[4];
        writeFloatLE(payload, pos);
        com.send_packet(POS, payload, 4);
        break;
    }

    case LOAD_TRAJ:
    {
        if (trajectory != nullptr)
        {
            delete trajectory;
            trajectory = nullptr;
        }

        trajectory = new Trajectory(payload, payload_len);

        if (trajectory->length == 0)
        {
            delete trajectory;
            trajectory = nullptr;
        }

        com.send_packet(ACK);
        break;
    }

    case EXEC_TRAJ:
    {
        if (trajectory == nullptr)
            break;
        execute_trajectory(trajectory, encoder, stepper);
        break;
    }

    default:
    {
        DBG_PRINT("Command: 0x");
        DBG_PRINTLN(cmd, HEX);
        DBG_PRINT("Payload: ");
        for (int i = 0; i < payload_len; i++)
        {
            DBG_PRINT("0x");
            DBG_PRINT(payload[i], HEX);
            DBG_PRINT(" ");
        }
        DBG_PRINTLN("");
        break;
    }
    };
}

void setup()
{

    Serial.begin(115200);

    Wire.begin();

    if (!encoder.begin())
    {
        DBG_PRINTLN("AS5600 not found!");
        while (1)
            ;
    }

    DBG_PRINTLN("AS5600 ready.");

    stepper.begin();
    stepper.enable();
    stepper.setMicrosteps(8);
    stepper.setStepsPerRevolution(200);
    stepper.setMaxSpeed(1000000);

    pinMode(HALL_PIN, INPUT_PULLUP);
}

void loop()
{
    if (com.available() > 0)
    {
        const Command *cmd = com.read();
        parse_cmd(cmd->cmd, cmd->payload, cmd->payload_len);
    }
}