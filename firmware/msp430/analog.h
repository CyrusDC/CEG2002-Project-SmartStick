#ifndef SMARTSTICK_ANALOG_H
#define SMARTSTICK_ANALOG_H

#include <stdbool.h>

void analog_init(void);
void analog_suspend(void);
void analog_resume(void);
bool grip_is_held(void);
void update_water_buzzer(void);
void analog_clear_buzzer(void);

#endif
