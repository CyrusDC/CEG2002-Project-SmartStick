// Standalone CCS bench test for the MSP-EXP430F5529LP LaunchPad.
// S1 sends one 'v' over 9600 baud UART and toggles red LED1.
// P3.3 (UCA0TXD) -> XIAO sender D7 (RX), and GND -> GND.
#include <msp430.h>
#include <stdbool.h>

#define BUTTON_PIN BIT1  // S1 on P2.1, active low
#define LED_PIN BIT0     // LED1 on P1.0
#define DEBOUNCE_CYCLES 21000UL  // About 20 ms at 1.048576 MHz MCLK

int main(void)
{
    bool button_pressed = false;
    WDTCTL = WDTPW | WDTHOLD;

    P2DIR &= ~BUTTON_PIN;
    P2OUT |= BUTTON_PIN;
    P2REN |= BUTTON_PIN;
    P1OUT &= ~LED_PIN;
    P1DIR |= LED_PIN;

    // Use the reset FLL multiplier (32) with the 32.768 kHz REFO reference.
    // MCLK and SMCLK are then nominally 1.048576 MHz.
    UCSCTL3 = SELREF__REFOCLK;
    UCA0CTL1 = UCSWRST | UCSSEL__SMCLK;
    UCA0CTL0 = 0;  // Asynchronous UART, 8 data bits, no parity, one stop bit
    UCA0BRW = 6;
    UCA0MCTL = UCOS16 | UCBRF_13;  // 9600 baud with 1.048576 MHz SMCLK
    P3SEL |= BIT3;  // P3.3 = UCA0TXD
    UCA0CTL1 &= ~UCSWRST;

    for (;;)
    {
        const bool sampled_pressed = (P2IN & BUTTON_PIN) == 0;
        if (sampled_pressed == button_pressed)
            continue;

        __delay_cycles(DEBOUNCE_CYCLES);
        if (((P2IN & BUTTON_PIN) == 0) != sampled_pressed)
            continue;

        button_pressed = sampled_pressed;
        if (button_pressed)
        {
            while ((UCA0IFG & UCTXIFG) == 0) {}
            UCA0TXBUF = 'v';
            P1OUT ^= LED_PIN;
        }
    }
}
