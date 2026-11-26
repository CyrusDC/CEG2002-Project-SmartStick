// MSP430F5529 Walking Cane + SenseStep ESP32 (FSR Trigger + Ultrasonic LEDs + LPM3 Sleep)
// Servos:      P1.2 (Left PWM, TA0.1), P1.3 (Right PWM, TA0.2)
// FSR:         P6.0 (A0) -> 3.3V -- FSR -- P6.0 -- 10k -- GND
// Ultrasonics: P3.5 (Shared TRIG), P2.5 (Left ECHO), P1.5 (Right ECHO) -> uses TA1
// Water/Buzz:  P7.0 (Water Sensor S, A12), P2.0 (Active Buzzer +)
// XIAO/ESP:    P3.3 (UCA0TXD) -> ESP RX (GPIO 18), GND -> GND (9600 baud, 8N1)
// LEDs:        P1.0 (Red = Left < 200 mm), P4.7 (Green = Right < 200 mm)
// Sleep Timer: TA2 on 32.768 kHz ACLK wakes every 250 ms in LPM3 to poll FSR

#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

#define CLOCK_HZ               1048576UL
#define US_TICKS(us)           ((uint16_t)(((uint32_t)(us) * 16384UL + 7812UL) / 15625UL))

// --- 1. Servo Calibration & Cover Travel Time (TA0) ---
#define PWM_PERIOD_20MS        20971u  // 20 ms period (50 Hz)
#define LEFT_STOP              1573u   // ~1.50 ms neutral pulse (P1.2)
#define RIGHT_STOP             1573u   // ~1.50 ms neutral pulse (P1.3)
#define RIGHT_STOP_TRIM        0       // Signed tick offset applied when stopped
#define LEFT_SPEED             260u
#define COVER_TRAVEL_MS        1086u    // Tune (ms) to match full open/close travel
#define RIGHT_CLOSE_OFFSET_MS  (-195)  // Offset (ms) applied to right motor close time

// --- 2. Ultrasonic & SenseStep UART (TA1 + UCA0) ---
#define TRIG_PIN               BIT5    // P3.5 (Shared TRIG)
#define LEFT_ECHO_PIN          BIT5    // P2.5 (Left ECHO)
#define RIGHT_ECHO_PIN         BIT5    // P1.5 (Right ECHO)
#define ECHO_TIMEOUT_US        30000u  // 30 ms timeout
#define PING_GAP_MS            65u     // ~65 ms between ultrasonic pings
#define DETECT_MM              200u    // LED threshold (200 mm = 20 cm)
#define UART_START             0xA5u

// --- 3. Water Sensor, Buzzer, FSR & LPM3 Sleep ---
#define BUZZER_PIN             BIT0    // P2.0 (Active Buzzer +)
#define WATER_THRESHOLD        1000u   // ADC12 threshold for Water (P7.0 / A12)
#define FSR_THRESHOLD          500u    // ADC12 threshold for FSR (P6.0 / A0)
#define RELEASE_CYCLES         8u      // Consecutive released loops (~0.5 s) before sleeping
#define LPM_POLL_TICKS         8192u   // 8192 / 32768 Hz ACLK = 250 ms FSR wake interval

enum Side { LEFT = 0, RIGHT = 1 };

// Watchable in CCS Expressions
volatile uint16_t distance_mm[2] = {0, 0};
volatile uint16_t water_val      = 0;
volatile uint16_t fsr_val        = 0;
volatile bool     cover_open     = false;

static void delay_ms(uint16_t ms)
{
    while (ms--)
    {
        __delay_cycles(1049);       // ~1 ms at 1.048576 MHz
    }
}

// --- Polled Single-Channel ADC12 Read (A0 = FSR, A12 = Water) ---
static uint16_t read_adc_channel(uint8_t inch)
{
    ADC12CTL0 &= ~ADC12ENC;
    ADC12MCTL0 = inch | ADC12EOS;
    ADC12CTL0 |= ADC12ENC | ADC12SC;
    while (ADC12CTL1 & ADC12BUSY) {}
    return ADC12MEM0;
}

static void init_hardware(void)
{
    UCSCTL3 = SELREF__REFOCLK;                      // SMCLK FLL ref = 32.768 kHz REFO
    UCSCTL4 = (UCSCTL4 & ~SELA_7) | SELA__REFOCLK;  // Guarantee ACLK = 32.768 kHz REFO

    // Onboard LEDs: P1.0 = Red (Left ultrasonic), P4.7 = Green (Right ultrasonic)
    P1DIR |= BIT0;  P1OUT &= ~BIT0;
    P4DIR |= BIT7;  P4OUT &= ~BIT7;

    // Servo PWM pins: P1.2 (TA0.1) and P1.3 (TA0.2)
    P1DIR |= BIT2 | BIT3;
    P1SEL |= BIT2 | BIT3;
    TA0CCR0  = PWM_PERIOD_20MS - 1;
    TA0CCTL1 = OUTMOD_7;
    TA0CCR1  = LEFT_STOP;
    TA0CCTL2 = OUTMOD_7;
    TA0CCR2  = RIGHT_STOP;
    TA0CTL   = TASSEL__SMCLK | MC__UP | TACLR;

    // Ultrasonic pins: P3.5 (TRIG), P2.5 (Left ECHO), P1.5 (Right ECHO) using TA1
    P3OUT &= ~TRIG_PIN;
    P3DIR |= TRIG_PIN;
    P2DIR &= ~LEFT_ECHO_PIN;
    P1DIR &= ~RIGHT_ECHO_PIN;
    TA1CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;

    // Buzzer pin: P2.0 output (OFF initially)
    P2DIR |= BUZZER_PIN;
    P2OUT &= ~BUZZER_PIN;

    // ADC12: P6.0 (A0 = FSR) and P7.0 (A12 = Water Sensor)
    P6SEL |= BIT0;
    P7SEL |= BIT0;
    ADC12CTL0 &= ~ADC12ENC;
    ADC12CTL0 = ADC12ON | ADC12SHT0_6;  // 192 sampling clocks
    ADC12CTL1 = ADC12SHP;
    ADC12IFG  = 0;
    ADC12IE   = 0;

    // USCI_A0 at 9600 baud, 8N1, on P3.3 -> ESP32 sender
    UCA0CTL1 = UCSWRST | UCSSEL__SMCLK;
    UCA0CTL0 = 0;
    UCA0BRW  = 6;
    UCA0MCTL = UCOS16 | UCBRF_13;
    P3SEL   |= BIT3;
    UCA0CTL1 &= ~UCSWRST;

    // Timer2_A configured for ACLK (stopped until entering LPM3)
    TA2CCR0  = LPM_POLL_TICKS - 1u;
    TA2CCTL0 = 0;
    TA2CTL   = TASSEL__ACLK | MC__STOP | TACLR;
}

// --- LPM3 Sleep Until FSR Is Gripped ---
static void enter_lpm3_until_grip(void)
{
    // 1. Ensure all alerts & outputs are OFF
    P1OUT &= ~BIT0;
    P4OUT &= ~BIT7;
    P2OUT &= ~BUZZER_PIN;

    // 2. Stop Servo PWM & hold signal pins LOW so servos don't jitter in sleep
    TA0CTL &= ~MC_3;
    P1SEL  &= ~(BIT2 | BIT3);
    P1OUT  &= ~(BIT2 | BIT3);

    // 3. Stop Ultrasonic timer & power down ADC12 core
    TA1CTL &= ~MC_3;
    ADC12CTL0 &= ~(ADC12ENC | ADC12ON);

    // 4. Start 250 ms ACLK wake timer
    TA2CCTL0 = CCIE;
    TA2CTL   = TASSEL__ACLK | MC__UP | TACLR;

    while (1)
    {
        // Enter LPM3 (CPU + SMCLK OFF; only 32.768 kHz ACLK + TA2 running)
        __bis_SR_register(LPM3_bits | GIE);
        __no_operation();

        // Briefly power ADC back on to sample FSR (P6.0 / A0)
        ADC12CTL0 |= ADC12ON;
        __delay_cycles(200);            // ADC startup settle
        fsr_val = read_adc_channel(ADC12INCH_0);

        if (fsr_val > FSR_THRESHOLD)
            break;                      // Cane gripped -> wake up!

        // Still unpressed -> turn ADC back off and return to LPM3
        ADC12CTL0 &= ~(ADC12ENC | ADC12ON);
    }

    // 5. Wake up: stop ACLK wake timer and restore active timers
    TA2CCTL0 = 0;
    TA2CTL   = TASSEL__ACLK | MC__STOP | TACLR;

    P1SEL |= BIT2 | BIT3;
    TA0CCR1 = LEFT_STOP;
    TA0CCR2 = RIGHT_STOP;
    TA0CTL  = TASSEL__SMCLK | MC__UP | TACLR;
    TA1CTL  = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;
}

// --- Motor Controls (Right follows Left) ---
static void syncRightToLeft(void)
{
    int32_t left_delta = (int32_t)TA0CCR1 - (int32_t)LEFT_STOP;
    int32_t stop_base  = (left_delta == 0) ? ((int32_t)RIGHT_STOP + (int32_t)RIGHT_STOP_TRIM)
                                           : (int32_t)RIGHT_STOP;
    int32_t right_cmd  = stop_base - left_delta;

    // Clamp to safe pulse bounds (~0.5 ms to ~2.5 ms at 1.048576 MHz)
    if (right_cmd < 524)  right_cmd = 524;
    if (right_cmd > 2621) right_cmd = 2621;

    TA0CCR2 = (uint16_t)right_cmd;
}

static void moveForward(void)
{
    TA0CCR1 = LEFT_STOP + LEFT_SPEED;
    syncRightToLeft();
}

static void moveBack(void)
{
    TA0CCR1 = LEFT_STOP - LEFT_SPEED;
    syncRightToLeft();
}

static void stopMotors(void)
{
    TA0CCR1 = LEFT_STOP;
    syncRightToLeft();
}

static void openCover(void)
{
    moveForward();
    delay_ms(COVER_TRAVEL_MS);
    stopMotors();
    cover_open = true;
}

static void closeCover(void)
{
    int32_t right_ms_signed = (int32_t)COVER_TRAVEL_MS + (int32_t)RIGHT_CLOSE_OFFSET_MS;
    uint16_t right_ms = (right_ms_signed > 0) ? (uint16_t)right_ms_signed : 0u;
    if (right_ms > COVER_TRAVEL_MS)
    {
        right_ms = COVER_TRAVEL_MS;
    }

    uint16_t rem_ms = COVER_TRAVEL_MS - right_ms;

    moveBack();
    delay_ms(right_ms);

    // Stop right motor (TA0CCR2) 195 ms early; left motor (TA0CCR1) finishes
    TA0CCR2 = (uint16_t)((int32_t)RIGHT_STOP + (int32_t)RIGHT_STOP_TRIM);
    delay_ms(rem_ms);

    stopMotors();
    cover_open = false;
}

// --- SenseStep UART to ESP32 ---
static void uart_write(uint8_t byte)
{
    while ((UCA0IFG & UCTXIFG) == 0) {}
    UCA0TXBUF = byte;
}

static void send_distance(enum Side side, uint16_t mm)
{
    const uint8_t low  = (uint8_t)(mm & 0x7Fu);
    const uint8_t high = (uint8_t)(mm >> 7);

    distance_mm[side] = mm;
    uart_write(UART_START);
    uart_write((uint8_t)side);
    uart_write(low);
    uart_write(high);
    uart_write((uint8_t)side ^ low ^ high);
}

static uint16_t ticks_to_mm(uint16_t ticks)
{
    uint16_t mm = (uint16_t)(((uint32_t)ticks * 42875UL + CLOCK_HZ / 8UL) /
                             (CLOCK_HZ / 4UL));
    if (mm < 20u || mm > 4000u)
        return 0;
    return mm;
}

// --- Read Ultrasonics + Water Sensor, Update LEDs, Buzzer & ESP32 ---
static void update_sensors(void)
{
    uint16_t started, l_rise_t = 0, r_rise_t = 0;
    uint16_t l_ticks = 0, r_ticks = 0;
    bool l_rising = false, r_rising = false;
    bool l_done = false, r_done = false;

    // 1. Fire shared TRIG (P3.5)
    P3OUT |= TRIG_PIN;
    __delay_cycles(16);
    P3OUT &= ~TRIG_PIN;

    // 2. Capture both ECHO pulses simultaneously using TA1
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

    uint16_t l_mm = l_done ? ticks_to_mm(l_ticks) : 0;
    uint16_t r_mm = r_done ? ticks_to_mm(r_ticks) : 0;

    send_distance(LEFT,  l_mm);
    send_distance(RIGHT, r_mm);

    // Update ultrasonic LEDs (< 200 mm and valid)
    if (l_mm > 0 && l_mm < DETECT_MM) P1OUT |= BIT0; else P1OUT &= ~BIT0;
    if (r_mm > 0 && r_mm < DETECT_MM) P4OUT |= BIT7; else P4OUT &= ~BIT7;

    // 3. Read Water Sensor (P7.0 / A12) & drive Buzzer (P2.0)
    water_val = read_adc_channel(ADC12INCH_12);
    if (water_val > WATER_THRESHOLD)
        P2OUT |= BUZZER_PIN;
    else
        P2OUT &= ~BUZZER_PIN;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    init_hardware();
    __enable_interrupt();

    // Start in LPM3 immediately until the user grips the FSR
    enter_lpm3_until_grip();

    uint8_t release_count = 0;

    for (;;)
    {
        fsr_val = read_adc_channel(ADC12INCH_0);

        if (fsr_val > FSR_THRESHOLD)
        {
            release_count = 0;

            if (!cover_open)
            {
                openCover();
            }
            update_sensors();
            delay_ms(PING_GAP_MS);
        }
        else
        {
            release_count++;
            if (release_count >= RELEASE_CYCLES)
            {
                release_count = 0;
                P1OUT &= ~BIT0;     // Turn off Left LED
                P4OUT &= ~BIT7;     // Turn off Right LED
                P2OUT &= ~BUZZER_PIN;
                send_distance(LEFT, 0);
                send_distance(RIGHT, 0);

                if (cover_open)
                {
                    closeCover();
                }

                // Go back to LPM3 sleep until next grip
                enter_lpm3_until_grip();
            }
            else
            {
                delay_ms(PING_GAP_MS);
            }
        }
    }
}

// --- Timer2_A0 ISR: Wakes CPU from LPM3 every 250 ms to check FSR ---
#pragma vector = TIMER2_A0_VECTOR
__interrupt void Timer2_A0_ISR(void)
{
    __bic_SR_register_on_exit(LPM3_bits);
}