// MSP430F5529 Standalone Dual HC-SR04 Ultrasonic Test
// Shared TRIG: P3.5
// Left ECHO:   P2.5 (via divider) -> Red LED (P1.0) when < 200 mm
// Right ECHO:  P1.5 (via divider) -> Green LED (P4.7) when < 200 mm

#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

#define CLOCK_HZ            1048576UL
#define US_TICKS(us)        ((uint16_t)(((uint32_t)(us) * 16384UL + 7812UL) / 15625UL))

#define TRIG_PIN            BIT5    // P3.5
#define LEFT_ECHO_PIN       BIT5    // P2.5
#define RIGHT_ECHO_PIN      BIT5    // P1.5
#define ECHO_TIMEOUT_US     30000u  // 30 ms timeout
#define DETECT_MM           200u    // 20 cm LED threshold

// Watch in CCS Expressions (0 = out of range / no echo)
volatile uint16_t distance_mm[2] = {0, 0};  // [0] = Left, [1] = Right

static uint16_t ticks_to_mm(uint16_t ticks)
{
    uint16_t mm = (uint16_t)(((uint32_t)ticks * 42875UL + CLOCK_HZ / 8UL) /
                             (CLOCK_HZ / 4UL));
    if (mm < 20u || mm > 4000u)
        return 0;
    return mm;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    UCSCTL3 = SELREF__REFOCLK;

    // Onboard LEDs
    P1DIR |= BIT0;  P1OUT &= ~BIT0;
    P4DIR |= BIT7;  P4OUT &= ~BIT7;

    // Ultrasonic pins: P3.5 (TRIG), P2.5 (Left ECHO), P1.5 (Right ECHO) & Timer1_A
    P3OUT &= ~TRIG_PIN;
    P3DIR |= TRIG_PIN;
    P2DIR &= ~LEFT_ECHO_PIN;
    P1DIR &= ~RIGHT_ECHO_PIN;
    TA1CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;

    while (1)
    {
        uint16_t started, l_rise_t = 0, r_rise_t = 0;
        uint16_t l_ticks = 0, r_ticks = 0;
        bool l_rising = false, r_rising = false;
        bool l_done = false, r_done = false;

        // 1. Fire 15 us shared TRIG pulse on P3.5
        P3OUT |= TRIG_PIN;
        __delay_cycles(16);
        P3OUT &= ~TRIG_PIN;

        // 2. Capture both ECHO pulses
        started = TA1R;
        while ((!l_done || !r_done) &&
               ((uint16_t)(TA1R - started) < US_TICKS(ECHO_TIMEOUT_US)))
        {
            uint8_t p1 = P1IN;
            uint8_t p2 = P2IN;

            if (!l_done)
            {
                if (!l_rising && (p2 & LEFT_ECHO_PIN))
                {
                    l_rise_t = TA1R;
                    l_rising = true;
                }
                else if (l_rising && !(p2 & LEFT_ECHO_PIN))
                {
                    l_ticks = (uint16_t)(TA1R - l_rise_t);
                    l_done = true;
                }
            }

            if (!r_done)
            {
                if (!r_rising && (p1 & RIGHT_ECHO_PIN))
                {
                    r_rise_t = TA1R;
                    r_rising = true;
                }
                else if (r_rising && !(p1 & RIGHT_ECHO_PIN))
                {
                    r_ticks = (uint16_t)(TA1R - r_rise_t);
                    r_done = true;
                }
            }
        }

        distance_mm[0] = l_done ? ticks_to_mm(l_ticks) : 0;
        distance_mm[1] = r_done ? ticks_to_mm(r_ticks) : 0;

        if (distance_mm[0] > 0 && distance_mm[0] < DETECT_MM)
            P1OUT |= BIT0;
        else
            P1OUT &= ~BIT0;

        if (distance_mm[1] > 0 && distance_mm[1] < DETECT_MM)
            P4OUT |= BIT7;
        else
            P4OUT &= ~BIT7;

        __delay_cycles(68157);      // ~65 ms gap between pings
    }
}