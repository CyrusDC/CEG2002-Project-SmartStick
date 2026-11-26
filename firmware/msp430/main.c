/*
 * SmartStick cane firmware for the MSP430F5529.
 * Version control: see the Git history for the submission baseline and revisions.
 * Author credits: Alfred (A) for UART. See the inline credit key below.
 * Hardware setup configures GPIO, three Timer_A peripherals, ADC12 and USCI_A0.
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
 * (C?) Cyrus, proposed servo and PWM assignment.
 * (F?) Farid, proposed ultrasonic ranging assignment.
 * (J?) Joseph, proposed analog sensing and buzzer assignment.
 * (HX?) HuiXin, proposed program flow and sleep assignment.
 * A question mark means the teammate needs to confirm the code credit.
 * (AI-assisted) ADC completion interrupt added during submission preparation.
 */

#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

#define CLOCK_HZ               1048576UL  /* Set the nominal SMCLK rate used for echo conversion (F?). */
#define US_TICKS(us)           ((uint16_t)(((uint32_t)(us) * 16384UL + 7812UL) / 15625UL))  /* Convert microseconds to rounded Timer_A1 ticks (F?). */

// --- 1. Servo Calibration & Cover Travel Time (TA0) ---
#define PWM_PERIOD_20MS        20971u  /* Set the shared 20 ms PWM period (C?). */
#define LEFT_STOP              1573u  /* Set the left servo's neutral pulse on P1.2 (C?). */
#define RIGHT_STOP             1573u  /* Set the right servo's neutral pulse on P1.3 (C?). */
#define RIGHT_STOP_TRIM        0  /* Adjust the right neutral pulse by this many ticks (C?). */
#define LEFT_SPEED             260u  /* Offset the left servo from neutral by this many ticks (C?). */
#define COVER_TRAVEL_MS        1086u  /* Set the calibrated cover travel time in milliseconds (C?). */
#define RIGHT_CLOSE_OFFSET_MS  (-195)  /* Shorten right cover closing by 195 ms (C?). */

// --- 2. Ultrasonic & SenseStep UART (TA1 + UCA0) ---
#define TRIG_PIN               BIT5  /* Select the shared TRIG on P3.5 (F?). */
#define LEFT_ECHO_PIN          BIT5  /* Select the left ECHO on P2.5 (F?). */
#define RIGHT_ECHO_PIN         BIT5  /* Select the right ECHO on P1.5 (F?). */
#define ECHO_TIMEOUT_US        30000u  /* Limit echo polling to 30 ms (F?). */
#define PING_GAP_MS            65u  /* Set the delay between ultrasonic checks (F?). */
#define DETECT_MM              200u  /* Light a warning LED below 200 mm (F?). */
#define UART_START             0xA5u  /* Set the UART frame start byte (A). */

// --- 3. Water Sensor, Buzzer, FSR & LPM3 Sleep ---
#define BUZZER_PIN             BIT0  /* Select the active buzzer output on P2.0 (J?). */
#define WATER_THRESHOLD        1000u  /* Set the water-sensor threshold on A12 (J?). */
#define FSR_THRESHOLD          500u  /* Set the grip-sensor threshold on A0 (J?). */
#define RELEASE_CYCLES         8u  /* Require eight consecutive released-grip checks (HX?). */
#define LPM_POLL_TICKS         8192u  /* Set the 250 ms wake interval using 32.768 kHz ACLK (HX?). */

enum Side { LEFT = 0, RIGHT = 1 };  /* Define the UART side identifiers (A). */

// Watchable in CCS Expressions
volatile uint16_t distance_mm[2] = {0, 0};  /* Keep the last UART reading for each side visible in CCS (A). */
volatile uint16_t water_val      = 0;  /* Keep the latest water sample visible in CCS (J?). */
volatile uint16_t fsr_val        = 0;  /* Keep the latest grip sample visible in CCS (J?). */
volatile bool     cover_open     = false;  /* Track whether the servo covers are open (C?). */
static volatile uint16_t adc_result;  /* Store the conversion result shared by the ISR and foreground (AI-assisted). */
static volatile bool adc_ready;  /* Flag when the ISR has published the conversion (AI-assisted). */

// Busy-wait for ms milliseconds using the configured CPU clock.
static void delay_ms(uint16_t ms)
{
    while (ms--)  /* Run one wait per requested millisecond (HX?). */
    {
        __delay_cycles(1049);  /* Wait about 1 ms at the nominal CPU clock (HX?). */
    }
}

// Sample ADC channel inch (ADC12INCH_0 for grip or ADC12INCH_12 for water).
// Return the 12-bit result delivered by ADC12_ISR. The caller powers ADC12 on
// before calling this function. It must be called from foreground code.
static uint16_t read_adc_channel(uint8_t inch)
{
    __disable_interrupt();  /* Mask interrupts until the foreground enters LPM0 (AI-assisted). */
    ADC12CTL0 &= ~ADC12ENC;  /* Unlock the channel configuration (J?). */
    ADC12MCTL0 = inch | ADC12EOS;  /* Route the selected analog input to MEM0 (J?). */
    ADC12IFG &= ~ADC12IFG0;  /* Clear a pending MEM0 completion flag before starting (AI-assisted). */
    adc_ready = false;  /* Mark the new conversion as incomplete (AI-assisted). */
    ADC12CTL0 |= ADC12ENC | ADC12SC;  /* Enable and start a single ADC conversion (J?). */
    while (!adc_ready)  /* Wait until the ADC ISR publishes the current result (AI-assisted). */
    {
        __bis_SR_register(LPM0_bits | GIE);  /* Enter LPM0 and enable interrupts in the same instruction (AI-assisted). */
        __disable_interrupt();  /* Check completion with interrupts masked after any wake-up (AI-assisted). */
    }
    __enable_interrupt();  /* Restore foreground interrupt handling (AI-assisted). */
    return adc_result;  /* Return the sample published by the ADC ISR (AI-assisted). */
}

// Configure the clock sources, GPIO, PWM, echo timing, ADC12 and UART.
// Takes no arguments. Leaves the wake timer stopped until the first sleep.
static void init_hardware(void)
{
    UCSCTL3 = SELREF__REFOCLK;  /* Use REFO as the frequency-locked loop reference (HX?). */
    UCSCTL4 = (UCSCTL4 & ~SELA_7) | SELA__REFOCLK;  /* Select 32.768 kHz REFO for ACLK (HX?). */

    // Onboard LEDs: P1.0 = Red (Left ultrasonic), P4.7 = Green (Right ultrasonic)
    P1DIR |= BIT0;  P1OUT &= ~BIT0;  /* Make the left warning LED an output and turn it off (F?). */
    P4DIR |= BIT7;  P4OUT &= ~BIT7;  /* Make the right warning LED an output and turn it off (F?). */

    // Servo PWM pins: P1.2 (TA0.1) and P1.3 (TA0.2)
    P1DIR |= BIT2 | BIT3;  /* Set both PWM pins as outputs (C?). */
    P1SEL |= BIT2 | BIT3;  /* Select Timer_A0 output on both servo pins (C?). */
    TA0CCR0  = PWM_PERIOD_20MS - 1;  /* Set the shared 20 ms PWM frame (C?). */
    TA0CCTL1 = OUTMOD_7;  /* Use reset/set mode for left PWM (C?). */
    TA0CCR1  = LEFT_STOP;  /* Start the left servo at neutral (C?). */
    TA0CCTL2 = OUTMOD_7;  /* Use reset/set mode for right PWM (C?). */
    TA0CCR2  = RIGHT_STOP;  /* Start the right servo at neutral (C?). */
    TA0CTL   = TASSEL__SMCLK | MC__UP | TACLR;  /* Start Timer_A0 in up mode using SMCLK (C?). */

    // Ultrasonic pins: P3.5 (TRIG), P2.5 (Left ECHO), P1.5 (Right ECHO) using TA1
    P3OUT &= ~TRIG_PIN;  /* Set the trigger's initial level low (F?). */
    P3DIR |= TRIG_PIN;  /* Make the shared trigger an output (F?). */
    P2DIR &= ~LEFT_ECHO_PIN;  /* Make the left echo pin an input (F?). */
    P1DIR &= ~RIGHT_ECHO_PIN;  /* Make the right echo pin an input (F?). */
    TA1CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;  /* Run the echo timestamp counter continuously on SMCLK (F?). */

    // Buzzer pin: P2.0 output (OFF initially)
    P2DIR |= BUZZER_PIN;  /* Set the buzzer pin as an output (J?). */
    P2OUT &= ~BUZZER_PIN;  /* Hold the buzzer off at startup (J?). */

    // ADC12: P6.0 (A0 = FSR) and P7.0 (A12 = Water Sensor)
    P6SEL |= BIT0;  /* Select the analog function for the grip sensor (J?). */
    P7SEL |= BIT0;  /* Select the analog function for the water sensor (J?). */
    ADC12CTL0 &= ~ADC12ENC;  /* Unlock the ADC configuration registers (J?). */
    ADC12CTL0 = ADC12ON | ADC12SHT0_6;  /* Enable the ADC and select its sampling window (J?). */
    ADC12CTL1 = ADC12SHP;  /* Use the ADC sample-and-hold timer (J?). */
    ADC12IFG  = 0;  /* Clear pending ADC conversion flags (J?). */
    ADC12IE   = ADC12IE0;  /* Enable the ADC12MEM0 completion interrupt (AI-assisted). */

    // USCI_A0 at 9600 baud, 8N1, on P3.3 -> ESP32 sender
    UCA0CTL1 = UCSWRST | UCSSEL__SMCLK;  /* Hold USCI_A0 in reset and select SMCLK (A). */
    UCA0CTL0 = 0;                       /* Select the UART's 8N1 frame format (A). */
    UCA0BRW  = 6;                       /* Set the integer baud divider (A). */
    UCA0MCTL = UCOS16 | UCBRF_13;       /* Enable oversampling and fine modulation (A). */
    P3SEL   |= BIT3;                    /* Connect UCA0TXD to P3.3 (A). */
    UCA0CTL1 &= ~UCSWRST;               /* Start the configured UART (A). */

    // Timer2_A configured for ACLK (stopped until entering LPM3)
    TA2CCR0  = LPM_POLL_TICKS - 1u;  /* Set the 250 ms wake compare period (HX?). */
    TA2CCTL0 = 0;  /* Keep the wake compare interrupt off during setup (HX?). */
    TA2CTL   = TASSEL__ACLK | MC__STOP | TACLR;  /* Select ACLK and leave the wake timer stopped (HX?). */
}

// Stop active outputs and wait in LPM3 between Timer_A2 grip checks.
// Takes no arguments. Restore the PWM and echo timers before returning.
static void enter_lpm3_until_grip(void)
{
    // 1. Ensure all alerts & outputs are OFF
    P1OUT &= ~BIT0;  /* Clear the left warning LED before sleep (HX?). */
    P4OUT &= ~BIT7;  /* Clear the right warning LED before sleep (HX?). */
    P2OUT &= ~BUZZER_PIN;  /* Silence the buzzer before sleep (HX?). */

    // 2. Stop Servo PWM & hold signal pins LOW so servos don't jitter in sleep
    TA0CTL &= ~MC_3;  /* Stop the servo PWM counter (HX?). */
    P1SEL  &= ~(BIT2 | BIT3);  /* Return the servo pins to GPIO control (HX?). */
    P1OUT  &= ~(BIT2 | BIT3);  /* Hold both servo signal pins low (HX?). */

    // 3. Stop Ultrasonic timer & power down ADC12 core
    TA1CTL &= ~MC_3;  /* Stop the ultrasonic echo counter (HX?). */
    ADC12CTL0 &= ~(ADC12ENC | ADC12ON);  /* Disable conversions and turn off ADC12 (HX?). */

    // 4. Start 250 ms ACLK wake timer
    TA2CCTL0 = CCIE;  /* Enable the CCR0 wake interrupt (HX?). */
    TA2CTL   = TASSEL__ACLK | MC__UP | TACLR;  /* Start the 250 ms ACLK wake timer (HX?). */

    while (1)
    {
        // Enter LPM3 (CPU + SMCLK OFF; only 32.768 kHz ACLK + TA2 running)
        __bis_SR_register(LPM3_bits | GIE);  /* Enter LPM3 with interrupts enabled (HX?). */
        __no_operation();  /* Resume at this instruction after waking (HX?). */

        // Briefly power ADC back on to sample FSR (P6.0 / A0)
        ADC12CTL0 |= ADC12ON;  /* Power ADC12 for a grip check (HX?). */
        __delay_cycles(200);  /* Allow the ADC to settle after power-up (HX?). */
        fsr_val = read_adc_channel(ADC12INCH_0);  /* Sample the grip sensor on A0 (HX?). */

        if (fsr_val > FSR_THRESHOLD)
            break;  /* Leave sleep mode when a grip is detected (HX?). */

        // Still unpressed -> turn ADC back off and return to LPM3
        ADC12CTL0 &= ~(ADC12ENC | ADC12ON);  /* Power ADC12 down again while the cane is idle (HX?). */
    }

    // 5. Wake up: stop ACLK wake timer and restore active timers
    TA2CCTL0 = 0;  /* Disable the periodic wake interrupt (HX?). */
    TA2CTL   = TASSEL__ACLK | MC__STOP | TACLR;  /* Stop and clear the wake timer (HX?). */

    P1SEL |= BIT2 | BIT3;  /* Reconnect PWM to the servo pins (HX?). */
    TA0CCR1 = LEFT_STOP;  /* Restore the left servo neutral pulse (HX?). */
    TA0CCR2 = RIGHT_STOP;  /* Restore the right servo neutral pulse (HX?). */
    TA0CTL  = TASSEL__SMCLK | MC__UP | TACLR;  /* Restart the servo PWM counter (HX?). */
    TA1CTL  = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;  /* Restart the ultrasonic echo counter (HX?). */
}

// Mirror the left servo's PWM offset onto the right servo and clamp its pulse.
// Takes no arguments because the left command is read directly from TA0CCR1.
static void syncRightToLeft(void)
{
    int32_t left_delta = (int32_t)TA0CCR1 - (int32_t)LEFT_STOP;  /* Find the left pulse offset from neutral (C?). */
    int32_t stop_base  = (left_delta == 0) ? ((int32_t)RIGHT_STOP + (int32_t)RIGHT_STOP_TRIM)
                                           : (int32_t)RIGHT_STOP;  /* Apply the right servo trim only at neutral (C?). */
    int32_t right_cmd  = stop_base - left_delta;  /* Mirror the left pulse offset for the right servo (C?). */

    // Clamp to safe pulse bounds (~0.5 ms to ~2.5 ms at 1.048576 MHz)
    if (right_cmd < 524)  right_cmd = 524;  /* Enforce the minimum right pulse width (C?). */
    if (right_cmd > 2621) right_cmd = 2621;  /* Enforce the maximum right pulse width (C?). */

    TA0CCR2 = (uint16_t)right_cmd;  /* Apply the mirrored right-servo pulse width (C?). */
}

// Command both cover servos in the opening direction. Takes no arguments.
static void moveForward(void)
{
    TA0CCR1 = LEFT_STOP + LEFT_SPEED;  /* Command the left servo to open (C?). */
    syncRightToLeft();  /* Mirror the opening command to the right servo (C?). */
}

// Command both cover servos in the closing direction. Takes no arguments.
static void moveBack(void)
{
    TA0CCR1 = LEFT_STOP - LEFT_SPEED;  /* Command the left servo to close (C?). */
    syncRightToLeft();  /* Mirror the closing command to the right servo (C?). */
}

// Restore the neutral PWM commands on both cover servos. Takes no arguments.
static void stopMotors(void)
{
    TA0CCR1 = LEFT_STOP;  /* Restore the left neutral pulse (C?). */
    syncRightToLeft();  /* Restore the right neutral pulse (C?). */
}

// Open both covers for the calibrated travel time and record the open state.
// Takes no arguments. Stop both servos before returning.
static void openCover(void)
{
    moveForward();  /* Start opening both covers (C?). */
    delay_ms(COVER_TRAVEL_MS);  /* Allow both covers to finish opening (C?). */
    stopMotors();  /* Return both servos to neutral (C?). */
    cover_open = true;  /* Record that the covers are open (C?). */
}

// Close both covers, stopping the right servo earlier according to calibration.
// Takes no arguments. Clear the open state after both servos stop.
static void closeCover(void)
{
    int32_t right_ms_signed = (int32_t)COVER_TRAVEL_MS + (int32_t)RIGHT_CLOSE_OFFSET_MS;  /* Calculate the right servo's closing time (C?). */
    uint16_t right_ms = (right_ms_signed > 0) ? (uint16_t)right_ms_signed : 0u;  /* Floor the closing time at zero (C?). */
    if (right_ms > COVER_TRAVEL_MS)
    {
        right_ms = COVER_TRAVEL_MS;  /* Cap the right servo's closing time at the left servo's time (C?). */
    }

    uint16_t rem_ms = COVER_TRAVEL_MS - right_ms;  /* Calculate how long the left servo runs alone (C?). */

    moveBack();  /* Start closing both covers (C?). */
    delay_ms(right_ms);  /* Wait for the right cover to finish (C?). */

    // Stop right motor (TA0CCR2) 195 ms early; left motor (TA0CCR1) finishes
    TA0CCR2 = (uint16_t)((int32_t)RIGHT_STOP + (int32_t)RIGHT_STOP_TRIM);  /* Stop the right servo at its trimmed neutral pulse (C?). */
    delay_ms(rem_ms);  /* Let the left servo finish closing (C?). */

    stopMotors();  /* Return both servos to neutral (C?). */
    cover_open = false;  /* Record that the covers are closed (C?). */
}

// Wait for an available UART transmit buffer and write byte to it (A).
static void uart_write(uint8_t byte)
{
    while ((UCA0IFG & UCTXIFG) == 0) {}  /* Wait for TXBUF to become available (A). */
    UCA0TXBUF = byte;                    /* Queue the byte for serial transmission (A). */
}

// Send the reading for side (LEFT or RIGHT) in millimeters (mm) over UART.
// Transmit a marker, side, two 7-bit distance fields and their XOR checksum.
// A zero millimeter value clears the alert for that side.
static void send_distance(enum Side side, uint16_t mm)
{
    const uint8_t low  = (uint8_t)(mm & 0x7Fu);  /* Pack the low seven distance bits (A). */
    const uint8_t high = (uint8_t)(mm >> 7);     /* Pack the remaining distance bits (A). */

    distance_mm[side] = mm;                    /* Record the value for CCS inspection (A). */
    uart_write(UART_START);                    /* Mark the start of a new UART frame (A). */
    uart_write((uint8_t)side);                 /* Identify the left or right reading (A). */
    uart_write(low);                           /* Send the low distance field first (A). */
    uart_write(high);                          /* Send the high distance field (A). */
    uart_write((uint8_t)side ^ low ^ high);     /* Send an XOR check of the three fields (A). */
}

// Convert ticks of Timer_A1 echo round-trip time to one-way millimeters.
// Return zero for a computed distance outside the accepted 20–4000 mm range.
static uint16_t ticks_to_mm(uint16_t ticks)
{
    uint16_t mm = (uint16_t)(((uint32_t)ticks * 42875UL + CLOCK_HZ / 8UL) /
                             (CLOCK_HZ / 4UL));  /* Convert round-trip echo ticks to rounded one-way millimeters (F?). */
    if (mm < 20u || mm > 4000u)
        return 0;  /* Report zero for an out-of-range distance (F?). */
    return mm;  /* Return the valid distance in millimeters (F?). */
}

// Trigger both ultrasonic sensors and time each pulse within the echo timeout.
// Takes no arguments. Send both readings by UART, update the obstacle LEDs
// and sample the ADC water input to decide whether to sound the buzzer.
static void update_sensors(void)
{
    uint16_t started, l_rise_t = 0, r_rise_t = 0;  /* Store the poll start and each echo's rising edge (F?). */
    uint16_t l_ticks = 0, r_ticks = 0;  /* Store each measured echo width (F?). */
    bool l_rising = false, r_rising = false;  /* Track which rising edges have been observed (F?). */
    bool l_done = false, r_done = false;  /* Track which echo measurements are complete (F?). */

    // 1. Fire shared TRIG (P3.5)
    P3OUT |= TRIG_PIN;  /* Drive the shared trigger high (F?). */
    __delay_cycles(16);  /* Hold the trigger high for about 15 microseconds (F?). */
    P3OUT &= ~TRIG_PIN;  /* End the trigger pulse (F?). */

    // 2. Capture both ECHO pulses simultaneously using TA1
    started = TA1R;  /* Start the common echo timeout (F?). */
    while ((!l_done || !r_done) &&
           ((uint16_t)(TA1R - started) < US_TICKS(ECHO_TIMEOUT_US)))  /* Poll until both echoes finish or the timeout expires (F?). */
    {
        uint8_t p1 = P1IN;  /* Sample port 1 for the right echo (F?). */
        uint8_t p2 = P2IN;  /* Sample port 2 for the left echo (F?). */

        if (!l_done)
        {
            if (!l_rising && (p2 & LEFT_ECHO_PIN))
            {
                l_rise_t = TA1R;  /* Save the left echo's rising-edge timestamp (F?). */
                l_rising = true;  /* Begin looking for the left falling edge (F?). */
            }
            else if (l_rising && !(p2 & LEFT_ECHO_PIN))
            {
                l_ticks = (uint16_t)(TA1R - l_rise_t);  /* Measure left echo width with wraparound (F?). */
                l_done = true;  /* Finish the left echo measurement (F?). */
            }
        }

        if (!r_done)
        {
            if (!r_rising && (p1 & RIGHT_ECHO_PIN))
            {
                r_rise_t = TA1R;  /* Save the right echo's rising-edge timestamp (F?). */
                r_rising = true;  /* Begin looking for the right falling edge (F?). */
            }
            else if (r_rising && !(p1 & RIGHT_ECHO_PIN))
            {
                r_ticks = (uint16_t)(TA1R - r_rise_t);  /* Measure right echo width with wraparound (F?). */
                r_done = true;  /* Finish the right echo measurement (F?). */
            }
        }
    }

    uint16_t l_mm = l_done ? ticks_to_mm(l_ticks) : 0;  /* Convert the left echo or report zero after timeout (F?). */
    uint16_t r_mm = r_done ? ticks_to_mm(r_ticks) : 0;  /* Convert the right echo or report zero after timeout (F?). */

    send_distance(LEFT,  l_mm);         /* Send the current left distance (A). */
    send_distance(RIGHT, r_mm);         /* Send the current right distance (A). */

    // Update ultrasonic LEDs (< 200 mm and valid)
    if (l_mm > 0 && l_mm < DETECT_MM) P1OUT |= BIT0; else P1OUT &= ~BIT0;  /* Show the left obstacle warning when the reading is valid and close (F?). */
    if (r_mm > 0 && r_mm < DETECT_MM) P4OUT |= BIT7; else P4OUT &= ~BIT7;  /* Show the right obstacle warning when the reading is valid and close (F?). */

    // 3. Read Water Sensor (P7.0 / A12) & drive Buzzer (P2.0)
    water_val = read_adc_channel(ADC12INCH_12);  /* Sample the water sensor on A12 (J?). */
    if (water_val > WATER_THRESHOLD)
        P2OUT |= BUZZER_PIN;  /* Sound the buzzer above the water threshold (J?). */
    else
        P2OUT &= ~BUZZER_PIN;  /* Silence the buzzer at or below the threshold (J?). */
}

// Initialize the system, then alternate between grip-triggered sleep and
// active sensing. Takes no arguments and runs for the life of the firmware.
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;  /* Stop the watchdog before peripheral setup (HX?). */
    init_hardware();  /* Initialize GPIO and peripherals (HX?). */
    __enable_interrupt();  /* Enable interrupt delivery before sleeping (HX?). */

    // Start in LPM3 immediately until the user grips the FSR
    enter_lpm3_until_grip();  /* Sleep until a grip is detected (HX?). */

    uint8_t release_count = 0;  /* Initialize the consecutive release count (HX?). */

    for (;;)
    {
        fsr_val = read_adc_channel(ADC12INCH_0);  /* Read the grip sensor during active operation (HX?). */

        if (fsr_val > FSR_THRESHOLD)
        {
            release_count = 0;  /* Reset the release count while the grip is held (HX?). */

            if (!cover_open)
            {
                openCover();  /* Open the covers when active sensing starts (HX?). */
            }
            update_sensors();  /* Refresh sensor readings and outputs (HX?). */
            delay_ms(PING_GAP_MS);  /* Leave the configured gap before the next ping (HX?). */
        }
        else
        {
            release_count++;  /* Count one more consecutive released-grip check (HX?). */
            if (release_count >= RELEASE_CYCLES)
            {
                release_count = 0;  /* Clear the release count for the next grip (HX?). */
                P1OUT &= ~BIT0;  /* Turn off the left warning LED (HX?). */
                P4OUT &= ~BIT7;  /* Turn off the right warning LED (HX?). */
                P2OUT &= ~BUZZER_PIN;  /* Switch off the water buzzer before sleep (HX?). */
                send_distance(LEFT, 0);  /* Clear the left wrist alert (A). */
                send_distance(RIGHT, 0); /* Clear the right wrist alert (A). */

                if (cover_open)
                {
                    closeCover();  /* Close the covers before sleeping (HX?). */
                }

                // Go back to LPM3 sleep until next grip
                enter_lpm3_until_grip();  /* Return to the grip-wake sleep loop (HX?). */
            }
            else
            {
                delay_ms(PING_GAP_MS);  /* Wait before checking for another released sample (HX?). */
            }
        }
    }
}

// Handle the ADC12 MEM0 completion interrupt. Hardware supplies no arguments.
// Copy the result before flagging it ready, then resume the foreground caller.
#pragma vector = ADC12_VECTOR  /* Bind the ADC ISR to the ADC12 vector (AI-assisted). */
__interrupt void ADC12_ISR(void)
{
    adc_result = ADC12MEM0;  /* Read MEM0 and clear its interrupt flag (AI-assisted). */
    adc_ready = true;  /* Publish the result to the waiting foreground code (AI-assisted). */
    __bic_SR_register_on_exit(LPM0_bits);  /* Clear CPUOFF in the saved status register (AI-assisted). */
}

// Handle the Timer_A2 CCR0 interrupt. Hardware supplies no arguments.
// Wake the foreground grip check at the next 250 ms poll.
#pragma vector = TIMER2_A0_VECTOR  /* Bind the timer ISR to the Timer_A2 CCR0 vector (HX?). */
__interrupt void Timer2_A0_ISR(void)
{
    __bic_SR_register_on_exit(LPM3_bits);  /* Resume the foreground grip check after the timer interrupt (HX?). */
}

/*
 * Individual reflections
 * Alfred (A), draft for review:
 * I wrote the ESP32 sender and wrist receiver sketches and the UART link.
 * I applied CEG2002 UART concepts by matching the baud rate and frame format.
 * I used a start byte so the sender could recover alignment in the byte stream.
 * The XOR check and range check help the sender reject invalid readings.
 * The packet's side field tells the sender which wrist should get the reading.
 * Keeping serial parsing and wireless sending in separate functions made each link easier to inspect.
 *
 * Suggested reflection assignments for the remaining team members:
 * Cyrus: Timer_A0 PWM, servo calibration and cover motions.
 * Farid: Timer_A1 echo timing, timeout and tick-to-distance conversion.
 * Joseph: FSR and water ADC channels, thresholds and buzzer output.
 * HuiXin: Main loop, grip release logic and Timer_A2/LPM3 wake-up.
 * Confirm your own contribution and add 5–10 signed lines about what you wrote,
 * a coding challenge you encountered and which CEG2002 concept you applied.
 */
