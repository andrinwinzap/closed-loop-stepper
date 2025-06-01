#ifndef MACROS_H
#define MACROS_H

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

#if defined(ACTUATOR_CONFIG_1)
#include <actuators/actuator_1.h>
#elif defined(ACTUATOR_CONFIG_2)
#include <actuators/actuator_2.h>
#elif defined(ACTUATOR_CONFIG_3)
#include <actuators/actuator_3.h>
#elif defined(ACTUATOR_CONFIG_4)
#include <actuators/actuator_4.h>
#elif defined(ACTUATOR_CONFIG_5)
#include <actuators/actuator_5.h>
#elif defined(ACTUATOR_CONFIG_6)
#include <actuators/actuator_6.h>
#endif

#endif