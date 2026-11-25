// MSP-EXP430F5529LP. See the pin map in README.md.
#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

// Set to 0 when the force sensors (FSRs) are connected.
#ifndef GRIP_USE_BUTTON
#define GRIP_USE_BUTTON 1
#endif

#if GRIP_USE_BUTTON
#define GRIP_BUTTON_PIN BIT1  // LaunchPad S1 on P2.1, active low
#else
// Bench starting values. Calibrate each FSR.
static const uint16_t grip_on[3] = {1000, 1000, 1000};
static const uint16_t grip_off[3] = {750, 750, 750};
#endif

// Bench starting values. Calibrate both MG90S 360 servos.
#define LEFT_STOP_US     1505u
#define LEFT_OPEN_US     1600u
#define LEFT_CLOSE_US    1400u
#define LEFT_OPEN_MS     2000u
#define LEFT_CLOSE_MS    1900u
#define RIGHT_STOP_US    1505u
#define RIGHT_OPEN_US    1600u
#define RIGHT_CLOSE_US   1400u
#define RIGHT_OPEN_MS     260u
#define RIGHT_CLOSE_MS    260u
// Allow the next 20 ms PWM frame to deliver neutral and the covers to settle.
#define COVER_SETTLE_MS    80u
#define GRIP_DEBOUNCE_MS   30u
#define RELEASE_HOLD_MS   300u
#define VIBRATION_DEBOUNCE_MS 30u
#define VIBRATION_TEST_MS    250u  // One 200 ms wrist pulse, with time to spare
#define VIBRATION_TEST_MM    300u  // In the wrist's urgent vibration range
#define PING_INTERVAL_MS   65u
#define ECHO_TIMEOUT_US 30000u

#define CLOCK_HZ 1048576UL  // Reset FLL multiplier of 32 with 32.768 kHz REFO
#define US_TICKS(us) ((uint16_t)(((uint32_t)(us) * 16384UL + 7812UL) / 15625UL))
#define LEFT_TRIG BIT3      // P2.3 to left HC-SR04 TRIG
#define RIGHT_TRIG BIT4     // P2.4 to right HC-SR04 TRIG
#define VIBRATION_BUTTON_PIN BIT1  // LaunchPad S2 on P1.1, active low
#define UART_START 0xA5u

enum Side { LEFT, RIGHT };
enum CaneState { CLOSED, OPENING, RANGING, CLOSING };

// Watch these in CCS Expressions while calibrating. A distance of 0 is invalid.
#if !GRIP_USE_BUTTON
volatile uint16_t fsr_raw[3];
#endif
volatile uint16_t distance_mm[2];
volatile uint32_t tick_ms;
static volatile uint16_t left_drive_ms;
static volatile uint16_t right_drive_ms;

#pragma vector=TIMER1_A0_VECTOR
__interrupt void tick_isr(void)
{
    ++tick_ms;
    // End each movement even if ADC or UART processing stalls the main loop.
    if (left_drive_ms && --left_drive_ms == 0)
        TB0CCR5 = US_TICKS(LEFT_STOP_US);
    if (right_drive_ms && --right_drive_ms == 0)
        TB0CCR6 = US_TICKS(RIGHT_STOP_US);
}

static uint32_t now_ms(void)
{
    // MSP430 cannot read a 32-bit value atomically with a single instruction.
    uint32_t now;
    __disable_interrupt();
    now = tick_ms;
    __enable_interrupt();
    return now;
}

static uint32_t start_cover_motion(bool opening)
{
    const uint16_t left_ms = opening ? LEFT_OPEN_MS : LEFT_CLOSE_MS;
    const uint16_t right_ms = opening ? RIGHT_OPEN_MS : RIGHT_CLOSE_MS;

    __disable_interrupt();
    left_drive_ms = left_ms;
    right_drive_ms = right_ms;
    if (left_ms)
        TB0CCR5 = US_TICKS(opening ? LEFT_OPEN_US : LEFT_CLOSE_US);
    if (right_ms)
        TB0CCR6 = US_TICKS(opening ? RIGHT_OPEN_US : RIGHT_CLOSE_US);
    __enable_interrupt();

    return (uint32_t)(left_ms > right_ms ? left_ms : right_ms) + COVER_SETTLE_MS;
}

static void init_hardware(void)
{
    UCSCTL3 = SELREF__REFOCLK;

    P1OUT &= ~BIT0;                       // Red LED1 indicates active grip
    P1DIR |= BIT0;
    P1SEL &= ~VIBRATION_BUTTON_PIN;
    P1DIR &= ~VIBRATION_BUTTON_PIN;
    P1OUT |= VIBRATION_BUTTON_PIN;
    P1REN |= VIBRATION_BUTTON_PIN;        // Pull S2 high when released
    P1DIR &= ~(BIT2 | BIT3);
    P1SEL |= BIT2 | BIT3;                 // Left TA0.1 / right TA0.2 ECHO
    P2OUT &= ~(LEFT_TRIG | RIGHT_TRIG);
    P2DIR |= LEFT_TRIG | RIGHT_TRIG;

    TA0CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;
    TA1CCR0 = US_TICKS(1000u) - 1u;
    TA1CCTL0 = CCIE;
    TA1CTL = TASSEL__SMCLK | MC__UP | TACLR;

    // One 20 ms frame with separate PWM outputs on P3.5 and P3.6.
    TB0CCR0 = US_TICKS(20000u) - 1u;
    // Start at neutral. Set the covers to closed by hand before powering up.
    TB0CCR5 = US_TICKS(LEFT_STOP_US);
    TB0CCR6 = US_TICKS(RIGHT_STOP_US);
    TB0CCTL5 = OUTMOD_7 | CLLD_1;
    TB0CCTL6 = OUTMOD_7 | CLLD_1;
    P3DIR |= BIT5 | BIT6;
    P3SEL |= BIT5 | BIT6;
    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;

#if GRIP_USE_BUTTON
    P2SEL &= ~GRIP_BUTTON_PIN;
    P2DIR &= ~GRIP_BUTTON_PIN;
    P2OUT |= GRIP_BUTTON_PIN;
    P2REN |= GRIP_BUTTON_PIN;            // Pull S1 high when released
#else
    P6SEL |= BIT0 | BIT1 | BIT2;          // A0, A1, A2 for the three FSRs
    ADC12CTL0 = ADC12ON | ADC12MSC | ADC12SHT0_6;
    ADC12CTL1 = ADC12SHP | ADC12CONSEQ_1;
    ADC12CTL2 = ADC12RES_2;
    ADC12MCTL0 = ADC12INCH_0;
    ADC12MCTL1 = ADC12INCH_1;
    ADC12MCTL2 = ADC12INCH_2 | ADC12EOS;
    ADC12CTL0 |= ADC12ENC;
#endif

    // USCI_A0 P3.3 TX to XIAO D7, 9600 baud, 8N1.
    UCA0CTL1 = UCSWRST | UCSSEL__SMCLK;
    UCA0CTL0 = 0;
    UCA0BRW = 6;
    UCA0MCTL = UCOS16 | UCBRF_13;
    P3SEL |= BIT3;
    UCA0CTL1 &= ~UCSWRST;
}

static bool read_grip(void)
{
#if GRIP_USE_BUTTON
    return (P2IN & GRIP_BUTTON_PIN) == 0;
#else
    static bool pressed[3];
    uint8_t i;
    ADC12CTL0 &= ~ADC12SC;
    ADC12CTL0 |= ADC12SC;
    while ((ADC12IFG & BIT2) == 0) {}
    fsr_raw[0] = ADC12MEM0;
    fsr_raw[1] = ADC12MEM1;
    fsr_raw[2] = ADC12MEM2;

    for (i = 0; i < 3; ++i)
        pressed[i] = fsr_raw[i] >= (pressed[i] ? grip_off[i] : grip_on[i]);

    return pressed[0] || pressed[1] || pressed[2];
#endif
}

static bool capture_edge(volatile uint16_t *control, volatile uint16_t *capture,
                         uint16_t started, uint16_t *edge)
{
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
    uint16_t rise, fall, mm = 0;
    const uint16_t started = TA0R;

    *control = CM_1 | CCIS_0 | SCS | CAP;
    P2OUT |= trigger;
    __delay_cycles(16);  // At least 10 us at the nominal 1.048576 MHz clock
    P2OUT &= ~trigger;

    if (capture_edge(control, capture, started, &rise))
    {
        *control = CM_2 | CCIS_0 | SCS | CAP;
        if (capture_edge(control, capture, started, &fall))
        {
            // Convert round-trip ticks to one-way mm at 343 m/s.
            const uint16_t ticks = (uint16_t)(fall - rise);
            // Reduce the ratio by 8 so even a full 16-bit tick count fits in uint32_t.
            mm = (uint16_t)(((uint32_t)ticks * 42875UL + CLOCK_HZ / 8UL) / (CLOCK_HZ / 4UL));
            if (mm < 20u || mm > 4000u)
                mm = 0;
        }
    }
    *control = 0;
    return mm;
}

static void write_byte(uint8_t byte)
{
    while ((UCA0IFG & UCTXIFG) == 0) {}
    UCA0TXBUF = byte;
}

static void send_distance(enum Side side, uint16_t mm)
{
    // A5, side (0/1), low 7 bits, high 7 bits, XOR of the three payload bytes.
    // The start marker can never occur inside a payload. Zero means motor off.
    const uint8_t low = mm & 0x7Fu;
    const uint8_t high = mm >> 7;
    distance_mm[side] = mm;
    write_byte(UART_START);
    write_byte((uint8_t)side);
    write_byte(low);
    write_byte(high);
    write_byte((uint8_t)side ^ low ^ high);
}

static bool update_vibration_test(uint32_t now)
{
    static bool last_sample;
    static bool button_pressed;
    static bool testing;
    static uint32_t changed_at;
    static uint32_t started_at;
    const bool sampled_pressed = (P1IN & VIBRATION_BUTTON_PIN) == 0;

    if (sampled_pressed != last_sample)
    {
        last_sample = sampled_pressed;
        changed_at = now;
    }

    if (sampled_pressed != button_pressed && now - changed_at >= VIBRATION_DEBOUNCE_MS)
    {
        button_pressed = sampled_pressed;
        if (button_pressed)
        {
            started_at = now;
            testing = true;
            send_distance(LEFT, VIBRATION_TEST_MM);
            send_distance(RIGHT, VIBRATION_TEST_MM);
        }
    }

    if (testing && now - started_at >= VIBRATION_TEST_MS)
    {
        testing = false;
        send_distance(LEFT, 0);
        send_distance(RIGHT, 0);
    }
    return testing;
}

int main(void)
{
    enum CaneState state = CLOSED;
    enum Side next_side = LEFT;
    bool last_grip = false;
    uint32_t grip_changed_at = 0;
    uint32_t motion_started_at = 0;
    uint32_t motion_wait_ms = 0;
    uint32_t last_ping_at = 0;

    WDTCTL = WDTPW | WDTHOLD;
    init_hardware();
    __enable_interrupt();
    send_distance(LEFT, 0);
    send_distance(RIGHT, 0);

    for (;;)
    {
        const bool grip = read_grip();
        const uint32_t now = now_ms();
        const bool vibration_test_active = update_vibration_test(now);
        if (grip != last_grip)
        {
            last_grip = grip;
            grip_changed_at = now;
        }

        // Finish a complete stroke before responding to a reversal of the grip.
        if (state == OPENING || state == CLOSING)
        {
            if (now - motion_started_at < motion_wait_ms)
                continue;
            if (state == CLOSING)
                state = CLOSED;
        }

        if (state == CLOSED)
        {
            if (grip && now - grip_changed_at >= GRIP_DEBOUNCE_MS)
            {
                motion_started_at = now;
                motion_wait_ms = start_cover_motion(true);
                P1OUT |= BIT0;
                next_side = LEFT;
                state = OPENING;
            }
            continue;
        }

        if (!grip && now - grip_changed_at >= RELEASE_HOLD_MS)
        {
            motion_started_at = now;
            motion_wait_ms = start_cover_motion(false);
            P1OUT &= ~BIT0;
            if (!vibration_test_active)
            {
                send_distance(LEFT, 0);
                send_distance(RIGHT, 0);
            }
            state = CLOSING;
            continue;
        }

        if (state == OPENING)
        {
            if (!grip)
                continue;
            state = RANGING;
            last_ping_at = now - PING_INTERVAL_MS;
        }

        if (!vibration_test_active && now - last_ping_at >= PING_INTERVAL_MS)
        {
            last_ping_at = now;
            send_distance(next_side, measure_distance(next_side));
            next_side = next_side == LEFT ? RIGHT : LEFT;
        }
    }
}
