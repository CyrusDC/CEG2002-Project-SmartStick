// MSP430F5529 Standalone Water Sensor & Buzzer Test
// Water Sensor S:  P7.0 (A12)
// Active Buzzer +: P2.0
// Red LED (P1.0):   ON when water detected (mirrors buzzer)
// Green LED (P4.7): Blinks as heartbeat

#include <msp430.h>
#include <stdint.h>

#define BUZZER_PIN          BIT0    // P2.0
#define WATER_THRESHOLD     1000u   // Tune after watching water_val (0..4095)

volatile uint16_t water_val = 0;    // Watch in CCS Expressions

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    UCSCTL3 = SELREF__REFOCLK;

    // Onboard LEDs
    P1DIR |= BIT0;  P1OUT &= ~BIT0;
    P4DIR |= BIT7;  P4OUT &= ~BIT7;

    // Active Buzzer output on P2.0 (OFF initially)
    P2DIR |= BUZZER_PIN;
    P2OUT &= ~BUZZER_PIN;

    // ADC12 setup on P7.0 (A12)
    P7SEL |= BIT0;
    ADC12CTL0 &= ~ADC12ENC;
    ADC12CTL0  = ADC12ON | ADC12SHT0_6;
    ADC12CTL1  = ADC12SHP;
    ADC12MCTL0 = ADC12INCH_12 | ADC12EOS;
    ADC12CTL0 |= ADC12ENC;

    uint8_t hb_count = 0;

    while (1)
    {
        // Read Water Sensor (A12)
        ADC12CTL0 |= ADC12SC;
        while (ADC12CTL1 & ADC12BUSY) {}
        water_val = ADC12MEM0;

        if (water_val > WATER_THRESHOLD)
        {
            P2OUT |= BUZZER_PIN;    // Buzzer ON
            P1OUT |= BIT0;          // Red LED ON
        }
        else
        {
            P2OUT &= ~BUZZER_PIN;   // Buzzer OFF
            P1OUT &= ~BIT0;         // Red LED OFF
        }

        // Toggle Green LED every ~250 ms as heartbeat
        if (++hb_count >= 5)
        {
            P4OUT ^= BIT7;
            hb_count = 0;
        }

        __delay_cycles(52429);      // ~50 ms loop delay
    }
}