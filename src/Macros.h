#ifndef MACROS_H
#define MACROS_H

////////////////////////////////////////////

#define DEBUG_ENABLED 1
#define DEBUG_SERIAL Serial

#define CONFIG_1

///////////////////////////////////////////

#if DEBUG_ENABLED
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

////////////////////////////////////////////

#if defined(CONFIG_1)
#include <Configs/Config1.h>
#endif

////////////////////////////////////////////
#endif