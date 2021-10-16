#ifndef MQTTSN_DEBUG_H
#define MQTTSN_DEBUG_H

#define MQTTSN_DEBUG
#if defined(MQTTSN_DEBUG)
    #define PRINT(...) Serial.print(__VA_ARGS__)
    #define PRINTLN(...) Serial.println(__VA_ARGS__)
#else
    #define PRINT(...)
    #define PRINTLN(...)
#endif


#endif