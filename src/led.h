#ifndef LED_H
#define LED_H

void setup_led(void);
void set_led_state(bool state);
bool get_led_state(void);
void toggle_led_state(void);
void update_led(void);

#endif