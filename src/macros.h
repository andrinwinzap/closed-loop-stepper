#define DEBUG_ENABLED 1
#define DEBUG_SERIAL Serial

#if DEBUG_ENABLED
  #define DBG_PRINT(...) DEBUG_SERIAL.print(__VA_ARGS__)
  #define DBG_PRINTLN(...) DEBUG_SERIAL.println(__VA_ARGS__)
#else
  #define DBG_PRINT(...) do {} while(0)
  #define DBG_PRINTLN(...) do {} while(0)
#endif