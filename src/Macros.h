#ifndef MACROS_H
#define MACROS_H

////////////////////////////////////////////

#define DEBUG_ENABLED 1
#define DEBUG_SERIAL Serial

#define ACTUATOR_2_CONFIG

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

#if defined(ACTUATOR_1_CONFIG)
#include <Configs/Actuator_1.h>
#elif defined(ACTUATOR_2_CONFIG)
#include <Configs/Actuator_2.h>
#endif

////////////////////////////////////////////
#endif