#include "covers.h"
#include "timing.h"
#include <msp430.h>
#include <stdint.h>

// Timer_A0 PWM and cover travel calibration.
#define PWM_PERIOD_20MS        20971u  /* Set the shared 20 ms PWM period (C). */
#define LEFT_STOP              1573u  /* Set the left servo's neutral pulse on P1.2 (C). */
#define RIGHT_STOP             1573u  /* Set the right servo's neutral pulse on P1.3 (C). */
#define RIGHT_STOP_TRIM        0  /* Adjust the right neutral pulse by this many ticks (C). */
#define LEFT_SPEED             260u  /* Offset the left servo from neutral by this many ticks (C). */
#define COVER_TRAVEL_MS        1086u  /* Set the calibrated cover travel time in milliseconds (C). */
#define RIGHT_CLOSE_OFFSET_MS  (-195)  /* Shorten right cover closing by 195 ms (C). */

volatile bool cover_open     = false;  /* Track whether the servo covers are open (C). */

// Configure both cover-servo outputs and start Timer_A0 PWM. Takes no arguments.
void covers_init(void)
{
    // Servo PWM pins: P1.2 (TA0.1) and P1.3 (TA0.2)
    P1DIR |= BIT2 | BIT3;  /* Set both PWM pins as outputs (C). */
    P1SEL |= BIT2 | BIT3;  /* Select Timer_A0 output on both servo pins (C). */
    TA0CCR0  = PWM_PERIOD_20MS - 1;  /* Set the shared 20 ms PWM frame (C). */
    TA0CCTL1 = OUTMOD_7;  /* Use reset/set mode for left PWM (C). */
    TA0CCR1  = LEFT_STOP;  /* Start the left servo at neutral (C). */
    TA0CCTL2 = OUTMOD_7;  /* Use reset/set mode for right PWM (C). */
    TA0CCR2  = RIGHT_STOP;  /* Start the right servo at neutral (C). */
    TA0CTL   = TASSEL__SMCLK | MC__UP | TACLR;  /* Start Timer_A0 in up mode using SMCLK (C). */
}

// Stop Timer_A0 and hold both servo signal pins low. Takes no arguments.
void covers_suspend(void)
{
    TA0CTL &= ~MC_3;  /* Stop the servo PWM counter (HX). */
    P1SEL  &= ~(BIT2 | BIT3);  /* Return the servo pins to GPIO control (HX). */
    P1OUT  &= ~(BIT2 | BIT3);  /* Hold both servo signal pins low (HX). */
}

// Reconnect the servo outputs and restart neutral PWM. Takes no arguments.
void covers_resume(void)
{
    P1SEL |= BIT2 | BIT3;  /* Reconnect PWM to the servo pins (HX). */
    TA0CCR1 = LEFT_STOP;  /* Restore the left servo neutral pulse (HX). */
    TA0CCR2 = RIGHT_STOP;  /* Restore the right servo neutral pulse (HX). */
    TA0CTL  = TASSEL__SMCLK | MC__UP | TACLR;  /* Restart the servo PWM counter (HX). */
}

// Mirror the left servo's PWM offset onto the right servo and clamp its pulse.
// Takes no arguments because the left command is read directly from TA0CCR1.
static void syncRightToLeft(void)
{
    int32_t left_delta = (int32_t)TA0CCR1 - (int32_t)LEFT_STOP;  /* Find the left pulse offset from neutral (C). */
    int32_t stop_base  = (left_delta == 0) ? ((int32_t)RIGHT_STOP + (int32_t)RIGHT_STOP_TRIM)
                                           : (int32_t)RIGHT_STOP;  /* Apply the right servo trim only at neutral (C). */
    int32_t right_cmd  = stop_base - left_delta;  /* Mirror the left pulse offset for the right servo (C). */

    // Clamp to safe pulse bounds (~0.5 ms to ~2.5 ms at 1.048576 MHz)
    if (right_cmd < 524)  right_cmd = 524;  /* Enforce the minimum right pulse width (C). */
    if (right_cmd > 2621) right_cmd = 2621;  /* Enforce the maximum right pulse width (C). */

    TA0CCR2 = (uint16_t)right_cmd;  /* Apply the mirrored right-servo pulse width (C). */
}

// Command both cover servos in the opening direction. Takes no arguments.
static void moveForward(void)
{
    TA0CCR1 = LEFT_STOP + LEFT_SPEED;  /* Command the left servo to open (C). */
    syncRightToLeft();  /* Mirror the opening command to the right servo (C). */
}

// Command both cover servos in the closing direction. Takes no arguments.
static void moveBack(void)
{
    TA0CCR1 = LEFT_STOP - LEFT_SPEED;  /* Command the left servo to close (C). */
    syncRightToLeft();  /* Mirror the closing command to the right servo (C). */
}

// Restore the neutral PWM commands on both cover servos. Takes no arguments.
static void stopMotors(void)
{
    TA0CCR1 = LEFT_STOP;  /* Restore the left neutral pulse (C). */
    syncRightToLeft();  /* Restore the right neutral pulse (C). */
}

// Open both covers for the calibrated travel time and record the open state.
// Takes no arguments. Stop both servos before returning.
void openCover(void)
{
    moveForward();  /* Start opening both covers (C). */
    delay_ms(COVER_TRAVEL_MS);  /* Allow both covers to finish opening (C). */
    stopMotors();  /* Return both servos to neutral (C). */
    cover_open = true;  /* Record that the covers are open (C). */
}

// Close both covers, stopping the right servo earlier according to calibration.
// Takes no arguments. Clear the open state after both servos stop.
void closeCover(void)
{
    int32_t right_ms_signed = (int32_t)COVER_TRAVEL_MS + (int32_t)RIGHT_CLOSE_OFFSET_MS;  /* Calculate the right servo's closing time (C). */
    uint16_t right_ms = (right_ms_signed > 0) ? (uint16_t)right_ms_signed : 0u;  /* Floor the closing time at zero (C). */
    if (right_ms > COVER_TRAVEL_MS)
    {
        right_ms = COVER_TRAVEL_MS;  /* Cap the right servo's closing time at the left servo's time (C). */
    }

    uint16_t rem_ms = COVER_TRAVEL_MS - right_ms;  /* Calculate how long the left servo runs alone (C). */

    moveBack();  /* Start closing both covers (C). */
    delay_ms(right_ms);  /* Wait for the right cover to finish (C). */

    // Stop right motor (TA0CCR2) 195 ms early; left motor (TA0CCR1) finishes
    TA0CCR2 = (uint16_t)((int32_t)RIGHT_STOP + (int32_t)RIGHT_STOP_TRIM);  /* Stop the right servo at its trimmed neutral pulse (C). */
    delay_ms(rem_ms);  /* Let the left servo finish closing (C). */

    stopMotors();  /* Return both servos to neutral (C). */
    cover_open = false;  /* Record that the covers are closed (C). */
}
