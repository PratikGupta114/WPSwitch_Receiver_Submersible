#ifndef SW_UART_H
#define SW_UART_H

#include <stdint.h>

/** HLW8032 UART baud rate (fixed by the IC) */
#define SW_UART_BAUD  4800

/**
 * @brief Initialize the software UART RX driver.
 *
 * PC6 must already be configured as input with pull-up by board_init().
 * SysTick must be running as a free-running counter (configured by
 * systick_init()).
 */
void sw_uart_init(void);

/**
 * @brief Non-blocking check for start bit and byte reception.
 *
 * If the line is idle (HIGH), returns -1 immediately.
 * If a start bit is detected (LOW), blocks with interrupts disabled for
 * ~2.29 ms to receive a single 8-E-1 byte, then returns it.
 *
 * @return The received byte (0-255), or -1 if no start bit was detected.
 */
int16_t sw_uart_rx_byte(void);

#endif /* SW_UART_H */
