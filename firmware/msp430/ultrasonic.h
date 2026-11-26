#ifndef SMARTSTICK_ULTRASONIC_H
#define SMARTSTICK_ULTRASONIC_H

#include <stdint.h>

#define PING_GAP_MS            65u  /* Set the delay between ultrasonic checks (F). */

void ultrasonic_init(void);
void ultrasonic_suspend(void);
void ultrasonic_resume(void);
void ultrasonic_read(uint16_t *left_mm, uint16_t *right_mm);
void ultrasonic_update_leds(uint16_t left_mm, uint16_t right_mm);
void ultrasonic_clear_leds(void);

#endif
