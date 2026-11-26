// MSP430F5529 LaunchPad bench test for the SenseStep wristbands.
// S1 (P2.1) buzzes the left wrist. S2 (P1.1) buzzes the right wrist.
// P3.3/UCA0TXD -> XIAO sender D7/RX, GND -> GND.
// Use sensestep_sender.ino on the cane ESP and sensestep_wrist.ino on both wrists.
#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

#define S1_PIN BIT1
#define S2_PIN BIT1
#define DEBOUNCE_MS 20u
#define TEST_MM 300u
#define TEST_MS 250u
#define UART_START 0xA5u

enum Side { LEFT, RIGHT };

typedef struct
{
    bool pressed;
    uint8_t debounce_ms;
    uint16_t buzz_ms;
} Button;

static void uart_write(uint8_t byte)
{
    while ((UCA0IFG & UCTXIFG) == 0) {}
    UCA0TXBUF = byte;
}

static void send_distance(enum Side side, uint16_t mm)
{
    // SenseStep UART frame: marker, side, two 7-bit distance fields, XOR.
    const uint8_t low = mm & 0x7Fu;
    const uint8_t high = mm >> 7;

    uart_write(UART_START);
    uart_write((uint8_t)side);
    uart_write(low);
    uart_write(high);
    uart_write((uint8_t)side ^ low ^ high);
}

static void sample_button(Button *button, bool pressed, enum Side side)
{
    if (button->buzz_ms != 0 && --button->buzz_ms == 0)
        send_distance(side, 0);

    if (pressed == button->pressed)
    {
        button->debounce_ms = 0;
        return;
    }

    if (++button->debounce_ms < DEBOUNCE_MS)
        return;

    button->debounce_ms = 0;
    button->pressed = pressed;
    if (pressed && button->buzz_ms == 0)
    {
        // 300 mm triggers a 200 ms pulse on the wrist. Zero clears the alert.
        send_distance(side, TEST_MM);
        button->buzz_ms = TEST_MS;
    }
}

int main(void)
{
    Button left = {0};
    Button right = {0};

    WDTCTL = WDTPW | WDTHOLD;
    UCSCTL3 = SELREF__REFOCLK;  // Nominal SMCLK is 1.048576 MHz.

    // The onboard buttons are active low.
    P2SEL &= ~S1_PIN;
    P2DIR &= ~S1_PIN;
    P2OUT |= S1_PIN;
    P2REN |= S1_PIN;
    P1SEL &= ~S2_PIN;
    P1DIR &= ~S2_PIN;
    P1OUT |= S2_PIN;
    P1REN |= S2_PIN;

    // USCI_A0 at 9600 baud, 8N1.
    UCA0CTL1 = UCSWRST | UCSSEL__SMCLK;
    UCA0CTL0 = 0;
    UCA0BRW = 6;
    UCA0MCTL = UCOS16 | UCBRF_13;
    P3SEL |= BIT3;
    UCA0CTL1 &= ~UCSWRST;

    // Timer_A1 supplies a 1 ms polling tick for both buttons.
    TA1CCR0 = 1048u;
    TA1CCTL0 = 0;
    TA1CTL = TASSEL__SMCLK | MC__UP | TACLR;

    for (;;)
    {
        while ((TA1CCTL0 & CCIFG) == 0) {}
        TA1CCTL0 &= ~CCIFG;
        sample_button(&left, (P2IN & S1_PIN) == 0, LEFT);
        sample_button(&right, (P1IN & S2_PIN) == 0, RIGHT);
    }
}
