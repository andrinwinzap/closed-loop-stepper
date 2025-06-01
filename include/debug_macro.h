#ifndef DEBUG_MACRO_H
#define DEBUG_MACRO_H

#define DEBUG_SERIAL Serial

#if defined(DEBUG_OUTPUT)
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