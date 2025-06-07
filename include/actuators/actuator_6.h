#include <byte_definitions.h>

// Serial Communication Protocol
constexpr int PROTOCOL_ADDRESS = Byte::Address::ACTUATOR_6;
constexpr int MASTER_COM_PORT = 2;
constexpr int MASTER_COM_BAUD = 115200;
constexpr int MASTER_COM_RX = 26;
constexpr int MASTER_COM_TX = 27;