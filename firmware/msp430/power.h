#ifndef SMARTSTICK_POWER_H
#define SMARTSTICK_POWER_H

void power_init_clocks(void);
void power_init_wake_timer(void);
void enter_lpm3_until_grip(void);

#endif
