#include <byte_definitions.h>

// Serial Communication Protocol
constexpr int PROTOCOL_ADDRESS = Byte::Address::MASTER;
constexpr int ACTUATOR_COM_PORT = 2;
constexpr int ACTUATOR_COM_BAUD = 115200;
constexpr int ACTUATOR_COM_RX = 19;
constexpr int ACTUATOR_COM_TX = 18;
constexpr unsigned long SERIAL_PROTOCOL_TIMEOUT = 1000;

constexpr int MUX_S0 = 5;
constexpr int MUX_S1 = 17;
constexpr int MUX_S2 = 16;
constexpr int MUX_S3 = 4;

constexpr uint16_t TCP_LISTEN_PORT = 8000;
constexpr unsigned long WIFI_CONNECT_TIMEOUT = 10000UL;