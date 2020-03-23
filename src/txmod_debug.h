#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

//#define DEBUG_USE_SW_SERIAL 1

void debug_init();
void debug_println(String line);
void debug_print(String str);

#define debug_serial_println(X) debug_println(String(X))
#define debug_serial_print(X) debug_print(String(X))

#endif