#ifndef SMARTSTICK_TIMING_H
#define SMARTSTICK_TIMING_H

#include <msp430.h>
#include <stdint.h>

// Busy-wait for ms milliseconds using the configured CPU clock.
static inline void delay_ms(uint16_t ms)
{
    while (ms--)  /* Run one wait per requested millisecond (HX). */
    {
        __delay_cycles(1049);  /* Wait about 1 ms at the nominal CPU clock (HX). */
    }
}

#endif
