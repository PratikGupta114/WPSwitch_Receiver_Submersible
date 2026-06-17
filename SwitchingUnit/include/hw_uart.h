#ifndef HW_UART_H
#define HW_UART_H

#include <stdint.h>

/** RX ring buffer size — must be a power of 2 */
#define HW_UART_RX_BUF_SIZE  128

/**
 * @brief Initialize USART1 on PD5 (TX) / PD6 (RX).
 * @param baud  Baud rate (e.g. 115200).
 *
 * Enables RXNE interrupt with ring buffer. TX is blocking (poll TXE).
 */
void hw_uart_init(uint32_t baud);

/** @brief Send one byte (blocks until TXE). */
void hw_uart_send_byte(uint8_t b);

/** @brief Send a buffer of bytes (blocking). */
void hw_uart_send(const uint8_t *data, uint16_t len);

/**
 * @brief Read one byte from the RX ring buffer.
 * @return Received byte (0–255), or -1 if buffer is empty.
 */
int16_t hw_uart_read_byte(void);

/** @brief Number of bytes available in the RX ring buffer. */
uint16_t hw_uart_rx_available(void);

#endif /* HW_UART_H */
