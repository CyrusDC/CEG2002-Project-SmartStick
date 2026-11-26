/*
 * SmartStick cane firmware for the MSP430F5529.
 * Version control: see the Git history for the submission baseline and revisions.
 * Author credits: Alfred (A) for UART. See the inline credit key below.
 * The analog, covers, power, ultrasonic and UART modules configure the peripherals.
 * On startup, the cane enters LPM3 and Timer_A2 wakes it every 250 ms.
 * ADC12 samples the grip FSR on P6.0 and the water sensor on P7.0.
 * The ADC12 MEM0 interrupt hands each result back to the foreground.
 * A grip above the threshold starts Timer_A0 PWM and opens the servo covers.
 * Timer_A1 measures the left and right echo pulses from a shared trigger.
 * The main loop updates the obstacle LEDs and water buzzer from the readings.
 * P3.3 transmits framed distances to the ESP32 sender at 9600 baud, 8N1.
 * The original wiring note specified ESP RX GPIO 18. See the README for D7.
 * After eight released-grip checks, the cane clears alerts and closes the covers.
 * The cane then powers down ADC12, stops the active timers and returns to LPM3.
 */

/* Inline credit key
 * (A) Alfred, confirmed UART contribution.
 * (C) Cyrus, proposed servo and PWM assignment.
 * (F) Farid, proposed ultrasonic ranging assignment.
 * (J) Joseph, proposed analog sensing and buzzer assignment.
 * (HX) HuiXin, proposed program flow and sleep assignment.
 * The proposed assignments are pending teammate confirmation.
 * (AI-assisted) ADC completion interrupt added during submission preparation.
 */

#include "analog.h"
#include "covers.h"
#include "power.h"
#include "timing.h"
#include "uart.h"
#include "ultrasonic.h"
#include <msp430.h>
#include <stdint.h>

#define RELEASE_CYCLES         8u  /* Require eight consecutive released-grip checks (HX). */

// Initialize the system, then alternate between grip-triggered sleep and
// active sensing. Takes no arguments and runs for the life of the firmware.
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;  /* Stop the watchdog before peripheral setup (HX). */
    power_init_clocks();  /* Configure the clocks used by the peripherals (HX). */
    ultrasonic_init();  /* Start the ranging timer and configure the obstacle LEDs (F). */
    covers_init();  /* Start neutral PWM for both cover servos (C). */
    analog_init();  /* Set up the analog sensors and water buzzer (J). */
    uart_init();  /* Set up UART to the ESP32 sender (A). */
    power_init_wake_timer();  /* Configure the idle wake timer (HX). */
    __enable_interrupt();  /* Enable interrupt delivery before sleeping (HX). */

    // Start in LPM3 immediately until the user grips the FSR
    enter_lpm3_until_grip();  /* Sleep until a grip is detected (HX). */

    uint8_t release_count = 0;  /* Initialize the consecutive release count (HX). */

    for (;;)
    {
        if (grip_is_held())  /* Sample the grip sensor during active operation (HX). */
        {
            release_count = 0;  /* Reset the release count while the grip is held (HX). */

            if (!cover_open)
            {
                openCover();  /* Open the covers when active sensing starts (HX). */
            }
            uint16_t left_mm, right_mm;  /* Hold the current distances for the UART and LEDs (HX). */
            ultrasonic_read(&left_mm, &right_mm);  /* Measure both obstacle distances (F). */
            send_distance(LEFT, left_mm);  /* Send the current left distance (A). */
            send_distance(RIGHT, right_mm);  /* Send the current right distance (A). */
            ultrasonic_update_leds(left_mm, right_mm);  /* Refresh the obstacle indicators (F). */
            update_water_buzzer();  /* Sample the water sensor and set the buzzer (J). */
            delay_ms(PING_GAP_MS);  /* Leave the configured gap before the next ping (HX). */
        }
        else
        {
            release_count++;  /* Count one more consecutive released-grip check (HX). */
            if (release_count >= RELEASE_CYCLES)
            {
                release_count = 0;  /* Clear the release count for the next grip (HX). */
                ultrasonic_clear_leds();  /* Clear both warning LEDs (HX). */
                analog_clear_buzzer();  /* Switch off the water buzzer before sleep (HX). */
                send_distance(LEFT, 0);  /* Clear the left wrist alert (A). */
                send_distance(RIGHT, 0); /* Clear the right wrist alert (A). */

                if (cover_open)
                {
                    closeCover();  /* Close the covers before sleeping (HX). */
                }

                // Go back to LPM3 sleep until next grip
                enter_lpm3_until_grip();  /* Return to the grip-wake sleep loop (HX). */
            }
            else
            {
                delay_ms(PING_GAP_MS);  /* Wait before checking for another released sample (HX). */
            }
        }
    }
}

/*
 * Individual reflections
 * Alfred (A):
 * I wrote the ESP32 sender and wrist receiver sketches and the UART link.
 * I applied CEG2002 UART concepts by matching the baud rate and frame format.
 * I used a start byte so the sender could recover alignment in the byte stream.
 * The XOR check and range check help the sender reject invalid readings.
 * The packet's side field tells the sender which wrist should get the reading.
 * Keeping serial parsing and wireless sending in separate functions made each link easier to inspect.
 *
 * Cyrus (C)
 * The cover mechanism made the CEG2002 Timer_A PWM topic more concrete for me.
 * Timer_A0 gives both servos the same 20 ms frame with a separate pulse width for each.
 * The firmware coordinates the covers by mirroring the two commands around neutral.
 * The subtle part is that neutral pulse width and travel time solve different problems.
 * During closing, the firmware stops the right servo early using its calibrated timing offset.
 * For a bench check, I would confirm each neutral setting before tuning travel time.
 *
 * Farid (F)
 * I would start debugging the ultrasonic code by separating the trigger from the echoes.
 * Both sensors share one trigger and have separate rise and fall timestamps.
 * This applies the CEG2002 concept of measuring elapsed ticks with a free-running counter.
 * Unsigned subtraction lets the code measure a short pulse even if the counter wraps.
 * The timeout matters because either sensor can fail to return a complete pulse.
 * I would compare both readings at a known distance and then check the timeout path.
 *
 * Joseph (J)
 * This section connects the CEG2002 ADC12 lessons to two different sensor decisions.
 * The grip sensor uses A0, and the water sensor uses A12 through the same MEM0 register.
 * The conversion interrupt hands the foreground a result after the CPU waits in LPM0.
 * One detail I would check first is whether the ADC counts match the intended input.
 * The grip threshold controls whether the cane stays awake, while the water threshold drives the buzzer.
 * I would watch both ADC readings in CCS before changing either threshold.
 *
 * HuiXin (HX)
 * The main loop helped me see how the CEG2002 low-power and interrupt topics fit together.
 * In LPM3, Timer_A2 uses ACLK to wake the MSP430 every 250 ms for a grip check.
 * A detected grip brings the servo and echo timers back into operation.
 * Eight consecutive release checks keep one brief dip from closing the covers.
 * Clearing the alerts and closing the covers before sleep leaves the cane ready for the next grip.
 * I would check a quick re-grip separately from a full release to exercise both paths.
 */
