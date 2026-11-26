// Standalone HC-SR04 test for the MSP-EXP430F5529LP LaunchPad.
// LEFT:  P2.3 -> TRIG, ECHO -> divider -> P1.2 (TA0.1).
// RIGHT: P2.4 -> TRIG, ECHO -> divider -> P1.3 (TA0.2).
// Both VCC pins go to LaunchPad 5V. Both GND pins go to LaunchPad GND.
// On each ECHO, use 10 kΩ in series and 15 kΩ from the MSP430 input to GND.
// Read the LaunchPad's MSP Application UART1 at 9600 baud, 8N1.

#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

#define CLOCK_HZ 1048576UL  // Reset FLL multiplier of 32 with 32.768 kHz REFO
#define US_TICKS(us) ((uint16_t)(((uint32_t)(us) * 16384UL + 7812UL) / 15625UL))
#define PING_GAP_MS 65UL
#define ECHO_TIMEOUT_US 30000u
#define LEFT_TRIG BIT3
#define RIGHT_TRIG BIT4

enum Side { LEFT, RIGHT };

// CCS Expressions: 0 means no valid echo in the 20–4000 mm range.
volatile uint16_t distance_mm[2];

static void init_hardware(void)
{
    UCSCTL3 = SELREF__REFOCLK;

    P2OUT &= ~(LEFT_TRIG | RIGHT_TRIG);
    P2DIR |= LEFT_TRIG | RIGHT_TRIG;
    P1DIR &= ~(BIT2 | BIT3);
    P1SEL |= BIT2 | BIT3;
    TA0CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;

    // USCI_A1 connects to the LaunchPad's USB debugger backchannel.
    UCA1CTL1 = UCSWRST | UCSSEL__SMCLK;
    UCA1CTL0 = 0;
    UCA1BRW = 6;
    UCA1MCTL = UCOS16 | UCBRF_13;
    P4SEL |= BIT4;  // P4.4 / UCA1TXD
    UCA1CTL1 &= ~UCSWRST;
}

static bool capture_edge(volatile uint16_t *control, volatile uint16_t *capture,
                         uint16_t started, uint16_t *edge)
{
    // Unsigned subtraction also works when Timer_A rolls over mid-ping.
    while ((uint16_t)(TA0R - started) < US_TICKS(ECHO_TIMEOUT_US))
    {
        if (*control & CCIFG)
        {
            *edge = *capture;
            return (*control & COV) == 0;
        }
    }
    return false;
}

static uint16_t measure_distance(enum Side side)
{
    volatile uint16_t *control = side == LEFT ? &TA0CCTL1 : &TA0CCTL2;
    volatile uint16_t *capture = side == LEFT ? &TA0CCR1 : &TA0CCR2;
    const uint8_t trigger = side == LEFT ? LEFT_TRIG : RIGHT_TRIG;
    const uint16_t started = TA0R;
    uint16_t rise, fall, mm = 0;

    *control = CM_1 | CCIS_0 | SCS | CAP;
    P2OUT |= trigger;
    __delay_cycles(16);  // About 15 us at the nominal 1.048576 MHz clock
    P2OUT &= ~trigger;

    if (capture_edge(control, capture, started, &rise))
    {
        *control = CM_2 | CCIS_0 | SCS | CAP;
        if (capture_edge(control, capture, started, &fall))
        {
            const uint16_t ticks = (uint16_t)(fall - rise);
            // Round-trip ticks to one-way mm at 343 m/s. Reduce by 8 to fit uint32_t.
            mm = (uint16_t)(((uint32_t)ticks * 42875UL + CLOCK_HZ / 8UL) / (CLOCK_HZ / 4UL));
            if (mm < 20u || mm > 4000u)
                mm = 0;
        }
    }

    *control = 0;
    return mm;
}

static void write_char(char c)
{
    while ((UCA1IFG & UCTXIFG) == 0) {}
    UCA1TXBUF = (uint8_t)c;
}

static void write_text(const char *text)
{
    while (*text)
        write_char(*text++);
}

static void write_distance(uint16_t mm)
{
    char digits[5];
    uint8_t count = 0;

    if (mm == 0)
    {
        write_text("no valid echo");
        return;
    }

    while (mm)
    {
        digits[count++] = (char)('0' + mm % 10u);
        mm /= 10u;
    }
    while (count)
        write_char(digits[--count]);
    write_text(" mm");
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    init_hardware();
    write_text("HC-SR04 left and right distance test\r\n");

    for (;;)
    {
        __delay_cycles(CLOCK_HZ * PING_GAP_MS / 1000UL);
        distance_mm[LEFT] = measure_distance(LEFT);

        __delay_cycles(CLOCK_HZ * PING_GAP_MS / 1000UL);
        distance_mm[RIGHT] = measure_distance(RIGHT);

        write_text("LEFT ");
        write_distance(distance_mm[LEFT]);
        write_text("  |  RIGHT ");
        write_distance(distance_mm[RIGHT]);
        write_text("\r\n");
    }
}
