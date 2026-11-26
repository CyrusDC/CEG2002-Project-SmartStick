// MSP430F5529 Standalone Servo Test
// Left Servo:  P1.2 (TA0.1)
// Right Servo: P1.3 (TA0.2)

#include <msp430.h>
#include <stdint.h>

#define PWM_PERIOD_20MS     20971u  // 20 ms period (50 Hz)
#define LEFT_STOP           1573u   // ~1.50 ms neutral pulse
#define RIGHT_STOP          1573u   // ~1.50 ms neutral pulse
#define LEFT_SPEED          260u
#define RIGHT_SPEED         260u
#define COVER_TRAVEL_MS     800u    // Adjust to match your cover travel time

static void delay_ms(uint16_t ms)
{
    while (ms--)
    {
        __delay_cycles(1049);       // ~1 ms at 1.048576 MHz
    }
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    UCSCTL3 = SELREF__REFOCLK;

    // Onboard LEDs for direction feedback
    P1DIR |= BIT0;  P1OUT &= ~BIT0;
    P4DIR |= BIT7;  P4OUT &= ~BIT7;

    // Servo PWM setup on P1.2 (TA0.1) and P1.3 (TA0.2)
    P1DIR |= BIT2 | BIT3;
    P1SEL |= BIT2 | BIT3;
    TA0CCR0  = PWM_PERIOD_20MS - 1;
    TA0CCTL1 = OUTMOD_7;
    TA0CCR1  = LEFT_STOP;
    TA0CCTL2 = OUTMOD_7;
    TA0CCR2  = RIGHT_STOP;
    TA0CTL   = TASSEL__SMCLK | MC__UP | TACLR;

    while (1)
    {
        // Forward (Open)
        P1OUT |= BIT0;
        TA0CCR1 = LEFT_STOP + LEFT_SPEED;
        TA0CCR2 = RIGHT_STOP - RIGHT_SPEED;
        delay_ms(COVER_TRAVEL_MS);

        // Stop
        P1OUT &= ~BIT0;
        TA0CCR1 = LEFT_STOP;
        TA0CCR2 = RIGHT_STOP;
        delay_ms(1500);

        // Reverse (Close)
        P4OUT |= BIT7;
        TA0CCR1 = LEFT_STOP - LEFT_SPEED;
        TA0CCR2 = RIGHT_STOP + RIGHT_SPEED;
        delay_ms(COVER_TRAVEL_MS);

        // Stop
        P4OUT &= ~BIT7;
        TA0CCR1 = LEFT_STOP;
        TA0CCR2 = RIGHT_STOP;
        delay_ms(1500);
    }
}