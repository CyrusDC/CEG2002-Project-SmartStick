#include "analog.h"
#include <msp430.h>
#include <stdint.h>

#define BUZZER_PIN             BIT0  /* Select the active buzzer output on P2.0 (J). */
#define WATER_THRESHOLD        1000u  /* Set the water-sensor threshold on A12 (J). */
#define FSR_THRESHOLD          500u  /* Set the grip-sensor threshold on A0 (J). */

volatile uint16_t water_val      = 0;  /* Keep the latest water sample visible in CCS (J). */
volatile uint16_t fsr_val        = 0;  /* Keep the latest grip sample visible in CCS (J). */

static volatile uint16_t adc_result;  /* Store the conversion result shared by the ISR and foreground (J). */
static volatile bool adc_ready;  /* Flag when the ISR has published the conversion (J). */

// Configure the buzzer and ADC12 channels for grip and water sensing.
// Takes no arguments. Enable the MEM0 completion interrupt.
void analog_init(void)
{
    // Buzzer pin: P2.0 output (OFF initially)
    P2DIR |= BUZZER_PIN;  /* Set the buzzer pin as an output (J). */
    P2OUT &= ~BUZZER_PIN;  /* Hold the buzzer off at startup (J). */

    // ADC12: P6.0 (A0 = FSR) and P7.0 (A12 = Water Sensor)
    P6SEL |= BIT0;  /* Select the analog function for the grip sensor (J). */
    P7SEL |= BIT0;  /* Select the analog function for the water sensor (J). */
    ADC12CTL0 &= ~ADC12ENC;  /* Unlock the ADC configuration registers (J). */
    ADC12CTL0 = ADC12ON | ADC12SHT0_6;  /* Enable the ADC and select its sampling window (J). */
    ADC12CTL1 = ADC12SHP;  /* Use the ADC sample-and-hold timer (J). */
    ADC12IFG  = 0;  /* Clear pending ADC conversion flags (J). */
    ADC12IE   = ADC12IE0;  /* Enable the ADC12MEM0 completion interrupt (J). */
}

// Sample ADC channel inch (ADC12INCH_0 for grip or ADC12INCH_12 for water).
// Return the 12-bit result delivered by ADC12_ISR. The caller powers ADC12 on
// before calling this function. It must be called from foreground code.
static uint16_t read_adc_channel(uint8_t inch)
{
    __disable_interrupt();  /* Mask interrupts until the foreground enters LPM0 (J). */
    ADC12CTL0 &= ~ADC12ENC;  /* Unlock the channel configuration (J). */
    ADC12MCTL0 = inch | ADC12EOS;  /* Route the selected analog input to MEM0 (J). */
    ADC12IFG &= ~ADC12IFG0;  /* Clear a pending MEM0 completion flag before starting (J). */
    adc_ready = false;  /* Mark the new conversion as incomplete (J). */
    ADC12CTL0 |= ADC12ENC | ADC12SC;  /* Enable and start a single ADC conversion (J). */
    while (!adc_ready)  /* Wait until the ADC ISR publishes the current result (J). */
    {
        __bis_SR_register(LPM0_bits | GIE);  /* Enter LPM0 and enable interrupts in the same instruction (J). */
        __disable_interrupt();  /* Check completion with interrupts masked after any wake-up (J). */
    }
    __enable_interrupt();  /* Restore foreground interrupt handling (J). */
    return adc_result;  /* Return the sample published by the ADC ISR (J). */
}

// Disable conversion and power down ADC12. Takes no arguments.
void analog_suspend(void)
{
    ADC12CTL0 &= ~(ADC12ENC | ADC12ON);  /* Disable conversions and turn off ADC12 (HX). */
}

// Power up ADC12 and wait for it to settle. Takes no arguments.
void analog_resume(void)
{
    ADC12CTL0 |= ADC12ON;  /* Power ADC12 for a grip check (HX). */
    __delay_cycles(200);  /* Allow the ADC to settle after power-up (HX). */
}

// Sample the FSR on A0 and return whether it exceeds the grip threshold.
// Takes no arguments. ADC12 must already be powered on.
bool grip_is_held(void)
{
    fsr_val = read_adc_channel(ADC12INCH_0);  /* Read the grip sensor (HX). */
    return fsr_val > FSR_THRESHOLD;  /* Check whether the user is gripping the cane (HX). */
}

// Sample the water input on A12 and set the buzzer using the threshold.
// Takes no arguments. ADC12 must already be powered on.
void update_water_buzzer(void)
{
    water_val = read_adc_channel(ADC12INCH_12);  /* Sample the water sensor on A12 (J). */
    if (water_val > WATER_THRESHOLD)
        P2OUT |= BUZZER_PIN;  /* Sound the buzzer above the water threshold (J). */
    else
        P2OUT &= ~BUZZER_PIN;  /* Silence the buzzer at or below the threshold (J). */
}

// Turn off the water buzzer. Takes no arguments.
void analog_clear_buzzer(void)
{
    P2OUT &= ~BUZZER_PIN;  /* Silence the buzzer (J). */
}

// Handle the ADC12 MEM0 completion interrupt. Hardware supplies no arguments.
// Copy the result before flagging it ready, then resume the foreground caller.
#pragma vector = ADC12_VECTOR  /* Bind the ADC ISR to the ADC12 vector (J). */
__interrupt void ADC12_ISR(void)
{
    adc_result = ADC12MEM0;  /* Read MEM0 and clear its interrupt flag (J). */
    adc_ready = true;  /* Publish the result to the waiting foreground code (J). */
    __bic_SR_register_on_exit(LPM0_bits);  /* Clear CPUOFF in the saved status register (J). */
}
