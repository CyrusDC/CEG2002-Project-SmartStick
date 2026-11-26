#include "ultrasonic.h"
#include <msp430.h>
#include <stdbool.h>

#define CLOCK_HZ               1048576UL  /* Set the nominal SMCLK rate used for echo conversion (F). */
#define US_TICKS(us)           ((uint16_t)(((uint32_t)(us) * 16384UL + 7812UL) / 15625UL))  /* Convert microseconds to rounded Timer_A1 ticks (F). */

#define TRIG_PIN               BIT5  /* Select the shared TRIG on P3.5 (F). */
#define LEFT_ECHO_PIN          BIT5  /* Select the left ECHO on P2.5 (F). */
#define RIGHT_ECHO_PIN         BIT5  /* Select the right ECHO on P1.5 (F). */
#define ECHO_TIMEOUT_US        30000u  /* Limit echo polling to 30 ms (F). */

#define DETECT_MM              200u  /* Light a warning LED below 200 mm (F). */

// Configure the obstacle LEDs, TRIG and ECHO pins, and Timer_A1.
// Takes no arguments. Starts the free-running echo timestamp counter.
void ultrasonic_init(void)
{
    // Onboard LEDs: P1.0 = Red (Left ultrasonic), P4.7 = Green (Right ultrasonic)
    P1DIR |= BIT0;  P1OUT &= ~BIT0;  /* Make the left warning LED an output and turn it off (F). */
    P4DIR |= BIT7;  P4OUT &= ~BIT7;  /* Make the right warning LED an output and turn it off (F). */

    // Ultrasonic pins: P3.5 (TRIG), P2.5 (Left ECHO), P1.5 (Right ECHO) using TA1
    P3OUT &= ~TRIG_PIN;  /* Set the trigger's initial level low (F). */
    P3DIR |= TRIG_PIN;  /* Make the shared trigger an output (F). */
    P2DIR &= ~LEFT_ECHO_PIN;  /* Make the left echo pin an input (F). */
    P1DIR &= ~RIGHT_ECHO_PIN;  /* Make the right echo pin an input (F). */
    TA1CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;  /* Run the echo timestamp counter continuously on SMCLK (F). */
}

// Stop the echo timestamp counter. Takes no arguments.
void ultrasonic_suspend(void)
{
    TA1CTL &= ~MC_3;  /* Stop the ultrasonic echo counter (HX). */
}

// Clear and restart the SMCLK echo timestamp counter. Takes no arguments.
void ultrasonic_resume(void)
{
    TA1CTL  = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;  /* Restart the ultrasonic echo counter (HX). */
}

// Convert ticks of Timer_A1 echo round-trip time to one-way millimeters.
// Return zero for a computed distance outside the accepted 20-4000 mm range.
static uint16_t ticks_to_mm(uint16_t ticks)
{
    uint16_t mm = (uint16_t)(((uint32_t)ticks * 42875UL + CLOCK_HZ / 8UL) /
                             (CLOCK_HZ / 4UL));  /* Convert round-trip echo ticks to rounded one-way millimeters (F). */
    if (mm < 20u || mm > 4000u)
        return 0;  /* Report zero for an out-of-range distance (F). */
    return mm;  /* Return the valid distance in millimeters (F). */
}

// Trigger both ultrasonic sensors and poll both echoes up to the timeout.
// Write the resulting millimeters to left_mm and right_mm, using zero for
// an incomplete or invalid echo. Both output pointers must be valid.
void ultrasonic_read(uint16_t *left_mm, uint16_t *right_mm)
{
    uint16_t started, l_rise_t = 0, r_rise_t = 0;  /* Store the poll start and each echo's rising edge (F). */
    uint16_t l_ticks = 0, r_ticks = 0;  /* Store each measured echo width (F). */
    bool l_rising = false, r_rising = false;  /* Track which rising edges have been observed (F). */
    bool l_done = false, r_done = false;  /* Track which echo measurements are complete (F). */

    // 1. Fire shared TRIG (P3.5)
    P3OUT |= TRIG_PIN;  /* Drive the shared trigger high (F). */
    __delay_cycles(16);  /* Hold the trigger high for about 15 microseconds (F). */
    P3OUT &= ~TRIG_PIN;  /* End the trigger pulse (F). */

    // 2. Capture both ECHO pulses simultaneously using TA1
    started = TA1R;  /* Start the common echo timeout (F). */
    while ((!l_done || !r_done) &&
           ((uint16_t)(TA1R - started) < US_TICKS(ECHO_TIMEOUT_US)))  /* Poll until both echoes finish or the timeout expires (F). */
    {
        uint8_t p1 = P1IN;  /* Sample port 1 for the right echo (F). */
        uint8_t p2 = P2IN;  /* Sample port 2 for the left echo (F). */

        if (!l_done)
        {
            if (!l_rising && (p2 & LEFT_ECHO_PIN))
            {
                l_rise_t = TA1R;  /* Save the left echo's rising-edge timestamp (F). */
                l_rising = true;  /* Begin looking for the left falling edge (F). */
            }
            else if (l_rising && !(p2 & LEFT_ECHO_PIN))
            {
                l_ticks = (uint16_t)(TA1R - l_rise_t);  /* Measure left echo width with wraparound (F). */
                l_done = true;  /* Finish the left echo measurement (F). */
            }
        }

        if (!r_done)
        {
            if (!r_rising && (p1 & RIGHT_ECHO_PIN))
            {
                r_rise_t = TA1R;  /* Save the right echo's rising-edge timestamp (F). */
                r_rising = true;  /* Begin looking for the right falling edge (F). */
            }
            else if (r_rising && !(p1 & RIGHT_ECHO_PIN))
            {
                r_ticks = (uint16_t)(TA1R - r_rise_t);  /* Measure right echo width with wraparound (F). */
                r_done = true;  /* Finish the right echo measurement (F). */
            }
        }
    }

    *left_mm = l_done ? ticks_to_mm(l_ticks) : 0;  /* Convert the left echo or report zero after timeout (F). */
    *right_mm = r_done ? ticks_to_mm(r_ticks) : 0;  /* Convert the right echo or report zero after timeout (F). */
}

// Update the matching obstacle LEDs for left_mm and right_mm.
// A reading must be nonzero and below the distance threshold to light an LED.
void ultrasonic_update_leds(uint16_t left_mm, uint16_t right_mm)
{
    // Update ultrasonic LEDs (< 200 mm and valid)
    if (left_mm > 0 && left_mm < DETECT_MM) P1OUT |= BIT0; else P1OUT &= ~BIT0;  /* Show the left obstacle warning when the reading is valid and close (F). */
    if (right_mm > 0 && right_mm < DETECT_MM) P4OUT |= BIT7; else P4OUT &= ~BIT7;  /* Show the right obstacle warning when the reading is valid and close (F). */
}

// Turn off both obstacle LEDs. Takes no arguments.
void ultrasonic_clear_leds(void)
{
    P1OUT &= ~BIT0;  /* Turn off the left warning LED (HX). */
    P4OUT &= ~BIT7;  /* Turn off the right warning LED (HX). */
}
