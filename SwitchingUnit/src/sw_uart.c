#include "sw_uart.h"
#include "board_pins.h"
#include "ch32v00x.h"

/* ---------------------------------------------------------------------------
 * Bit-Timing Constants
 *
 * SysTick runs at HCLK (48 MHz), configured by systick_init().
 * At 4800 baud: 1 bit = 48 000 000 / 4800 = 10 000 ticks.
 *
 * HLW8032 uses 8-E-1 format (8 data + even parity + 1 stop = 11 bits).
 * We sample data bits and skip over the parity and stop bits.
 * -------------------------------------------------------------------------*/
#define SYSTICK_HZ          48000000UL

#define TICKS_PER_BIT       (SYSTICK_HZ / SW_UART_BAUD)          /* 10 000 */
#define TICKS_HALF_BIT      (TICKS_PER_BIT / 2)                  /*  5 000 */
#define TICKS_1_5_BIT       ((TICKS_PER_BIT * 3) / 2)            /* 15 000 */
/* 10.5 bits from start: skip 8 data + parity + half stop-bit margin       */
#define TICKS_FRAME_END     ((TICKS_PER_BIT * 105) / 10)         /*105 000 */

void sw_uart_init(void)
{
    /* PC6 is already configured as input with pull-up by board_init(). */
    /* SysTick is already running as free-running counter.              */
}

int16_t sw_uart_rx_byte(void)
{
    /* Quick check: if line is idle (HIGH), return -1 immediately */
    if (hlw8032_rx_read() != 0) {
        return -1;
    }

    uint32_t start_time = SysTick->CNT;

    /* ----- Validate start bit at 0.5 bit-time ----- */
    uint32_t target = start_time + TICKS_HALF_BIT;
    while (((int32_t)(SysTick->CNT - target)) < 0) { /* spin */ }

    if (hlw8032_rx_read() != 0) {
        return -1;
    }

    /* ----- Sample 8 data bits (LSB first) ----- */
    uint8_t rx_byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        target = start_time + TICKS_1_5_BIT + (uint32_t)i * TICKS_PER_BIT;
        while (((int32_t)(SysTick->CNT - target)) < 0) { /* spin */ }
        if (hlw8032_rx_read()) {
            rx_byte |= (1u << i);
        }
    }

    /* ----- Wait past parity + stop bit (10.5 bit-times from start) ----- */
    target = start_time + TICKS_FRAME_END;
    while (((int32_t)(SysTick->CNT - target)) < 0) { /* spin */ }

    return (int16_t)rx_byte;
}
