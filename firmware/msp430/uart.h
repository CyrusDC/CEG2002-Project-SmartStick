#ifndef SMARTSTICK_UART_H
#define SMARTSTICK_UART_H

#include <stdint.h>

enum Side { LEFT = 0, RIGHT = 1 };  /* Define the UART side identifiers (A). */

void uart_init(void);
void send_distance(enum Side side, uint16_t mm);

#endif
