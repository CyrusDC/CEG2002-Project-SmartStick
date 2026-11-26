#include <msp430.h>
#include <stdint.h>

// MSP430F5529 LaunchPad with one continuous-rotation MG90S servo.
#define S1_PIN BIT1     // P2.1, active low
#define S2_PIN BIT1     // P1.1, active low
#define SERVO_PIN BIT5  // P3.5 (J3 pin 30), Timer_B channel 5

#define SERVO_NEUTRAL_US  1505u  // Adjust if the servo creeps with both buttons released.
#define SERVO_S1_US       1600u
#define SERVO_S2_US       1400u
#define SERVO_FRAME_US   20000u
#define BUTTON_SAMPLE_US  1000u
#define DEBOUNCE_SAMPLES    20u

// SMCLK is nominally 1,048,576 Hz with the reset FLL multiplier and REFO.
#define US_TICKS(us) ((uint16_t)(((uint32_t)(us) * 16384UL + 7812UL) / 15625UL))

enum ButtonState { BUTTON_STOP, BUTTON_S1, BUTTON_S2 };

static enum ButtonState read_buttons(void)
{
    const uint8_t s1_pressed = (P2IN & S1_PIN) == 0;
    const uint8_t s2_pressed = (P1IN & S2_PIN) == 0;

    if (s1_pressed == s2_pressed)
        return BUTTON_STOP;

    return s1_pressed ? BUTTON_S1 : BUTTON_S2;
}

int main(void)
{
    enum ButtonState candidate = BUTTON_STOP;
    uint8_t stable_samples = 0;

    WDTCTL = WDTPW | WDTHOLD;
    UCSCTL3 = SELREF__REFOCLK;

    // Each onboard button shorts its input to ground when pressed.
    P2SEL &= ~S1_PIN;
    P2DIR &= ~S1_PIN;
    P2OUT |= S1_PIN;
    P2REN |= S1_PIN;
    P1SEL &= ~S2_PIN;
    P1DIR &= ~S2_PIN;
    P1OUT |= S2_PIN;
    P1REN |= S2_PIN;

    // Hardware PWM keeps running while the buttons are sampled.
    TB0CCR0 = US_TICKS(SERVO_FRAME_US) - 1u;
    TB0CCR5 = US_TICKS(SERVO_NEUTRAL_US);
    TB0CCTL5 = OUTMOD_7 | CLLD_1;  // Apply pulse-width changes at the frame boundary.
    P3DIR |= SERVO_PIN;
    P3SEL |= SERVO_PIN;
    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;

    TA1CCR0 = US_TICKS(BUTTON_SAMPLE_US) - 1u;
    TA1CCTL0 = 0;
    TA1CTL = TASSEL__SMCLK | MC__UP | TACLR;

    for (;;)
    {
        enum ButtonState buttons;

        while ((TA1CCTL0 & CCIFG) == 0)
        {
            // Wait for the next 1 ms sample.
        }
        TA1CCTL0 &= ~CCIFG;

        buttons = read_buttons();
        if (buttons != candidate)
        {
            candidate = buttons;
            stable_samples = 0;
        }

        if (stable_samples == DEBOUNCE_SAMPLES)
            continue;

        ++stable_samples;
        if (stable_samples == DEBOUNCE_SAMPLES)
        {
            if (candidate == BUTTON_S1)
                TB0CCR5 = US_TICKS(SERVO_S1_US);
            else if (candidate == BUTTON_S2)
                TB0CCR5 = US_TICKS(SERVO_S2_US);
            else
                TB0CCR5 = US_TICKS(SERVO_NEUTRAL_US);
        }
    }
}
