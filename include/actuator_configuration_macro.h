#ifndef ACTUATOR_CONFIGURATION
#define ACTUATOR_CONFIGURATION

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