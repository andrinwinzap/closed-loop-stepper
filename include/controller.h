#include <byte_definitions.h>

// Serial Communication Protocol
constexpr int PROTOCOL_ADDRESS = Byte::Address::MASTER;
constexpr int ACTUATOR_COM_RX = 26;
constexpr int ACTUATOR_COM_TX = 27;
constexpr unsigned long SERIAL_PROTOCOL_TIMEOUT = 1000;

constexpr int MUX_S0 = 13;
constexpr int MUX_S1 = 25;
constexpr int MUX_S2 = 32;
constexpr int MUX_S3 = 33;

constexpr uint16_t TCP_LISTEN_PORT = 8000;
constexpr unsigned long WIFI_CONNECT_TIMEOUT = 10000UL;