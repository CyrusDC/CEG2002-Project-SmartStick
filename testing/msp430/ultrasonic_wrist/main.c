// MSP430F5529 LaunchPad bench test for two HC-SR04s and the SenseStep wristbands.
// Left: P2.3 = TRIG, P1.2 = ECHO through a voltage divider.
// Right: P2.4 = TRIG, P1.3 = ECHO through a voltage divider.
// P3.3/UCA0TXD -> XIAO sender D7/RX, GND -> GND.
// Use sensestep_sender.ino on the cane ESP and sensestep_wrist.ino on both wrists.
#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

#define CLOCK_HZ 1048576UL
#define US_TICKS(us) ((uint16_t)(((uint32_t)(us) * 16384UL + 7812UL) / 15625UL))
#define PING_GAP_MS 65UL
#define ECHO_TIMEOUT_US 30000u
#define LEFT_TRIG BIT3
#define RIGHT_TRIG BIT4
#define UART_START 0xA5u

enum Side { LEFT, RIGHT };

// Watch in CCS Expressions. Zero means no valid echo in the 20-4000 mm range.
volatile uint16_t distance_mm[2];

static void init_hardware(void)
{
    UCSCTL3 = SELREF__REFOCLK;  // Nominal SMCLK is 1.048576 MHz.

    P2OUT &= ~(LEFT_TRIG | RIGHT_TRIG);
    P2DIR |= LEFT_TRIG | RIGHT_TRIG;
    P1DIR &= ~(BIT2 | BIT3);
    P1SEL |= BIT2 | BIT3;  // P1.2 = TA0.1, P1.3 = TA0.2
    TA0CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;

    // USCI_A0 at 9600 baud, 8N1, to the ESP sender.
    UCA0CTL1 = UCSWRST | UCSSEL__SMCLK;
    UCA0CTL0 = 0;
    UCA0BRW = 6;
    UCA0MCTL = UCOS16 | UCBRF_13;
    P3SEL |= BIT3;
    UCA0CTL1 &= ~UCSWRST;
}

static bool capture_edge(volatile uint16_t *control, volatile uint16_t *capture,
                         uint16_t started, uint16_t *edge)
{
    // Unsigned subtraction handles a Timer_A rollover during the ping.
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
    __delay_cycles(16);
    P2OUT &= ~trigger;

    if (capture_edge(control, capture, started, &rise))
    {
        *control = CM_2 | CCIS_0 | SCS | CAP;
        if (capture_edge(control, capture, started, &fall))
        {
            const uint16_t ticks = (uint16_t)(fall - rise);
            // Convert round-trip ticks to one-way millimeters at 343 m/s.
            mm = (uint16_t)(((uint32_t)ticks * 42875UL + CLOCK_HZ / 8UL) /
                            (CLOCK_HZ / 4UL));
            if (mm < 20u || mm > 4000u)
                mm = 0;
        }
    }
    *control = 0;
    return mm;
}

static void uart_write(uint8_t byte)
{
    while ((UCA0IFG & UCTXIFG) == 0) {}
    UCA0TXBUF = byte;
}

static void send_distance(enum Side side, uint16_t mm)
{
    // Existing SenseStep frame: marker, side, two 7-bit distance fields, XOR.
    const uint8_t low = mm & 0x7Fu;
    const uint8_t high = mm >> 7;

    distance_mm[side] = mm;
    uart_write(UART_START);
    uart_write((uint8_t)side);
    uart_write(low);
    uart_write(high);
    uart_write((uint8_t)side ^ low ^ high);
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    init_hardware();

    for (;;)
    {
        // Stagger pings so the two sensors do not transmit together.
        __delay_cycles(CLOCK_HZ * PING_GAP_MS / 1000UL);
        send_distance(LEFT, measure_distance(LEFT));
        __delay_cycles(CLOCK_HZ * PING_GAP_MS / 1000UL);
        send_distance(RIGHT, measure_distance(RIGHT));
    }
}
