#ifndef DEBUG_MACRO_H
#define DEBUG_MACRO_H

#if defined(DEBUG_OUTPUT)

#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUD 115200

#define DBG_PRINT(...) DEBUG_SERIAL.print(__VA_ARGS__)
#define DBG_PRINTLN(...) DEBUG_SERIAL.println(__VA_ARGS__)
#else
#define DBG_PRINT(...) \
    do                 \
    {                  \
    } while (0)
#define DBG_PRINTLN(...) \
    do                   \
    {                    \
    } while (0)
#endif

#endif