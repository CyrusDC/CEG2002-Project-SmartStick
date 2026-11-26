#include "uart.h"
#include <msp430.h>

#define UART_START             0xA5u  /* Set the UART frame start byte (A). */

volatile uint16_t distance_mm[2] = {0, 0};  /* Keep the last UART reading for each side visible in CCS (A). */

// Configure USCI_A0 to transmit on P3.3 at 9600 baud, 8N1. Takes no arguments.
void uart_init(void)
{
    UCA0CTL1 = UCSWRST | UCSSEL__SMCLK;  /* Hold USCI_A0 in reset and select SMCLK (A). */
    UCA0CTL0 = 0;                       /* Select the UART's 8N1 frame format (A). */
    UCA0BRW  = 6;                       /* Set the integer baud divider (A). */
    UCA0MCTL = UCOS16 | UCBRF_13;       /* Enable oversampling and fine modulation (A). */
    P3SEL   |= BIT3;                    /* Connect UCA0TXD to P3.3 (A). */
    UCA0CTL1 &= ~UCSWRST;               /* Start the configured UART (A). */
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
void send_distance(enum Side side, uint16_t mm)
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
