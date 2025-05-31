#include <Arduino.h>
#include <AS5600/AS5600.h>
#include <ClosedLoop/ClosedLoop.h>
#include <Serialization/Serialization.h>
#include <SerialProtocol/SerialProtocol.h>
#include <Stepper/Stepper.h>
#include <Trajectory/Trajectory.h>
#include <Macros.h>

AS5600 encoder(GEAR_RATIO);
Stepper stepper(STEPPER_STEP_PIN, STEPPER_DIR_PIN, STEPPER_EN_PIN, GEAR_RATIO);

HardwareSerial com_serial(2);
SerialProtocol com(com_serial);

Trajectory *trajectory = nullptr;

TaskHandle_t controlTaskHandle = nullptr;

volatile bool startTrajectory = false;

enum cmdByte : uint8_t
{
    PING = 0x01,
    HOME = 0x02,
    POS = 0x03,
    LOAD_TRAJ = 0x04,
    EXEC_TRAJ = 0x05,
    FINISHED = 0x06,
    ACK = 0xEE,
    NACK = 0xFF
};

void control_loop_task(void *param)
{
    for (;;)
    {
        if (startTrajectory && trajectory != nullptr)
        {
            DBG_PRINTLN("[CONTROL] Starting trajectory execution...");
            execute_trajectory(trajectory, encoder, stepper);
            DBG_PRINTLN("[CONTROL] Finished trajectory execution");

            startTrajectory = false;
            com.send_packet(FINISHED);
        }

        encoder.update();

        vTaskDelay(pdMS_TO_TICKS(1)); // Run at ~1kHz
    }
}

void home()
{
    DBG_PRINTLN("[HOME] Starting homing process...");

    stepper.setSpeed(-HOMING_SPEED);
    stepper.move(-PI / 8.0, HOMING_SPEED, HOMING_ACCELERATION);
    DBG_PRINTLN("[HOME] Stepper move initiated");

    float first_bound = 0;

    stepper.start();
    stepper.accelerate(0, HOMING_SPEED, HOMING_ACCELERATION);
    DBG_PRINTLN("[HOME] Stepper acceleration started");

    float filteredHallSensorValue = analogRead(HALL_EFFECT_SENSOR_PIN);

    while (true)
    {
        stepper.updateAcceleration();
        int raw = analogRead(HALL_EFFECT_SENSOR_PIN);
        filteredHallSensorValue = HALL_EFFECT_SENSOR_ALPHA * raw + (1 - HALL_EFFECT_SENSOR_ALPHA) * filteredHallSensorValue;

#if PRINT_HALL_SENSOR_VALUE
        DBG_PRINT("[HOME] Hall sensor value: ");
        DBG_PRINTLN(filteredHallSensorValue);
#endif

        if (filteredHallSensorValue < 1)
        {
            DBG_PRINTLN("[HOME] Detected falling edge - Hall sensor triggered (first bound)");
            break;
        }

        delayMicroseconds(HALL_EFFECT_SENSOR_UPDATE_PERIOD);
    }

    first_bound = encoder.getPosition();
    DBG_PRINT("[HOME] First bound position: ");
    DBG_PRINTLN(first_bound);

    while (true)
    {
        int raw = analogRead(HALL_EFFECT_SENSOR_PIN);
        filteredHallSensorValue = HALL_EFFECT_SENSOR_ALPHA * raw + (1 - HALL_EFFECT_SENSOR_ALPHA) * filteredHallSensorValue;

        if (filteredHallSensorValue > 1)
        {
            DBG_PRINTLN("[HOME] Detected rising edge - leaving Hall sensor");
            break;
        }

        delayMicroseconds(HALL_EFFECT_SENSOR_UPDATE_PERIOD);
    }

    float current_pos = encoder.getPosition();
    float home = (current_pos - first_bound) / 2;

    DBG_PRINT("[HOME] Current position: ");
    DBG_PRINTLN(current_pos);
    DBG_PRINT("[HOME] Calculated home offset: ");
    DBG_PRINTLN(home);

    encoder.setPosition(current_pos - home);

    stepper.accelerate(HOMING_SPEED, 0, HOMING_ACCELERATION);
    while (stepper.updateAcceleration())
    {
        unsigned long start = micros();
        encoder.update();
    }

    move_to(0, encoder, stepper);

    DBG_PRINTLN("[HOME] Finished homing");
    DBG_PRINT("[HOME] Encoder position after homing: ");
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
    DBG_PRINT("[CMD] Received command 0x");
    DBG_PRINT(cmd, HEX);
    DBG_PRINT(" with payload length: ");
    DBG_PRINTLN(payload_len);

    switch (cmd)
    {
    case PING:
        DBG_PRINTLN("[CMD] PING");
        com.send_packet(ACK);
        break;

    case HOME:
        DBG_PRINTLN("[CMD] HOME");
        com.send_packet(ACK);
        home();
        com.send_packet(FINISHED);
        break;

    case POS:
    {
        DBG_PRINTLN("[CMD] POS");
        float pos = encoder.getPosition();
        DBG_PRINT("[CMD] Encoder position: ");
        DBG_PRINTLN(pos);
        uint8_t payload[4];
        writeFloatLE(payload, pos);
        com.send_packet(POS, payload, 4);
        break;
    }

    case LOAD_TRAJ:
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

        com.send_packet(ACK);
        break;
    }

    case EXEC_TRAJ:
    {
        DBG_PRINTLN("[CMD] EXEC_TRAJ");

        if (trajectory == nullptr)
        {
            DBG_PRINTLN("[CMD] No trajectory loaded. Skipping execution.");
            break;
        }

        startTrajectory = true;
        DBG_PRINTLN("[CMD] Trajectory execution triggered");
        com.send_packet(ACK);
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
    DBG_PRINTLN("[SETUP] Hall effect sensor pin configured");

    // Start control loop on Core 0
    xTaskCreatePinnedToCore(
        control_loop_task,
        "ControlLoopTask",
        4096,
        NULL,
        1,
        &controlTaskHandle,
        0 // Core 0
    );
}

void loop()
{
    if (com.available() > 0)
    {
        const Command *cmd = com.read();
        if (cmd)
        {
            DBG_PRINTLN("[LOOP] Command available, parsing...");
            parse_cmd(cmd->cmd, cmd->payload, cmd->payload_len);
        }
    }
}
