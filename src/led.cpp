#include <Arduino.h>
#include "led.h"

// LED is on GPIO2, see also XModem.cpp, line 125
//#define GPIO02  2
#ifdef DEBUG_USE_SW_SERIAL
#define LEDGPIO 2
#else
#define LEDGPIO 16
#endif

static bool LEDState = 0; // global for LED state.

void setup_led(void) {
    pinMode(LEDGPIO, OUTPUT);
}

void set_led_state(bool state) { 
    LEDState = state; 
    update_led();
}

bool get_led_state(void) { 
    return LEDState; 
}

void toggle_led_state(void) { 
    LEDState = !LEDState; 
    update_led();
}

void update_led(void) {
    digitalWrite(LEDGPIO, LEDState);
}