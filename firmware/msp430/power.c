#include "power.h"
#include "analog.h"
#include "covers.h"
#include "ultrasonic.h"
#include <msp430.h>

#define LPM_POLL_TICKS         8192u  /* Set the 250 ms wake interval using 32.768 kHz ACLK (HX). */

// Select REFO as the FLL reference and ACLK source. Takes no arguments.
void power_init_clocks(void)
{
    UCSCTL3 = SELREF__REFOCLK;  /* Use REFO as the frequency-locked loop reference (HX). */
    UCSCTL4 = (UCSCTL4 & ~SELA_7) | SELA__REFOCLK;  /* Select 32.768 kHz REFO for ACLK (HX). */
}

// Configure Timer_A2 for 250 ms ACLK wake-ups and leave it stopped.
// Takes no arguments.
void power_init_wake_timer(void)
{
    TA2CCR0  = LPM_POLL_TICKS - 1u;  /* Set the 250 ms wake compare period (HX). */
    TA2CCTL0 = 0;  /* Keep the wake compare interrupt off during setup (HX). */
    TA2CTL   = TASSEL__ACLK | MC__STOP | TACLR;  /* Select ACLK and leave the wake timer stopped (HX). */
}

// Stop active outputs and wait in LPM3 between Timer_A2 grip checks.
// Takes no arguments. Restore the PWM and echo timers before returning.
void enter_lpm3_until_grip(void)
{
    ultrasonic_clear_leds();  /* Clear the obstacle warnings before sleep (HX). */
    analog_clear_buzzer();  /* Silence the water buzzer before sleep (HX). */
    covers_suspend();  /* Stop PWM and hold both servo outputs low (HX). */
    ultrasonic_suspend();  /* Stop the echo timer (HX). */
    analog_suspend();  /* Power down the ADC between grip checks (HX). */

    TA2CCTL0 = CCIE;  /* Enable the CCR0 wake interrupt (HX). */
    TA2CTL   = TASSEL__ACLK | MC__UP | TACLR;  /* Start the 250 ms ACLK wake timer (HX). */

    while (1)
    {
        // Enter LPM3 (CPU + SMCLK OFF; only 32.768 kHz ACLK + TA2 running)
        __bis_SR_register(LPM3_bits | GIE);  /* Enter LPM3 with interrupts enabled (HX). */
        __no_operation();  /* Resume at this instruction after waking (HX). */

        analog_resume();  /* Power up the ADC for one grip check (HX). */
        if (grip_is_held())
            break;  /* Leave sleep mode when a grip is detected (HX). */

        analog_suspend();  /* Power down the ADC before the next sleep (HX). */
    }

    TA2CCTL0 = 0;  /* Disable the periodic wake interrupt (HX). */
    TA2CTL   = TASSEL__ACLK | MC__STOP | TACLR;  /* Stop and clear the wake timer (HX). */

    covers_resume();  /* Restore PWM with both servos in neutral (HX). */
    ultrasonic_resume();  /* Restart the echo timer (HX). */
}

// Handle the Timer_A2 CCR0 interrupt. Hardware supplies no arguments.
// Wake the foreground grip check at the next 250 ms poll.
#pragma vector = TIMER2_A0_VECTOR  /* Bind the timer ISR to the Timer_A2 CCR0 vector (HX). */
__interrupt void Timer2_A0_ISR(void)
{
    __bic_SR_register_on_exit(LPM3_bits);  /* Resume the foreground grip check after the timer interrupt (HX). */
}
