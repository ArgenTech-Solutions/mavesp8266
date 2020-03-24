#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

//#define DEBUG_USE_SW_SERIAL 1
#define DEBUG_DISABLE 

void debug_init();
void debug_serial_println(String line);
void debug_serial_print(String str);

#ifdef DEBUG_DISABLE
#define debug_serial_println(X)
#define debug_serial_print(X)
#else
#define debug_serial_println(X) debug_serial_println(String(X))
#define debug_serial_print(X) debug_serial_print(String(X))
#endif

#endif