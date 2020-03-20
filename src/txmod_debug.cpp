#include <SoftwareSerial.h>
#include "txmod_debug.h"

#ifdef DEBUG_USE_SW_SERIAL
#define DEBUG_SERIAL swSer
SoftwareSerial swSer;
#else
#define DEBUG_SERIAL Serial
#endif

void debug_init()
{
    #ifdef DEBUG_USE_SW_SERIAL
        swSer.begin(115200,SWSERIAL_8N1,14,16,false);
        swSer.println(F("[MSG] initd swSer output"));
    #else
        Serial.begin(115200);
        Serial.println(F("[MSG] initd Serial output"));
    #endif
}

void debug_println(String line)
{
    DEBUG_SERIAL.println(line);
}

void debug_print(String str)
{
    DEBUG_SERIAL.print(str);
}