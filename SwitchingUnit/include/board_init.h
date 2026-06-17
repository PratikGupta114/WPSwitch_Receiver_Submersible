#ifndef BOARD_INIT_H
#define BOARD_INIT_H

/**
 * @brief Initialize all board GPIO pins and peripheral clocks.
 *
 * Configures:
 *  - PC0, PC1, PC2 as push-pull outputs (relays, safe OFF state)
 *  - PC3 as open-drain output (presence pin, released HIGH)
 *  - PC6 as input with pull-up (HLW8032 software UART RX)
 *  - PC7 as input with pull-up (HLW8032 CF pulse)
 *  - PD4 as input with pull-up (motor detection optoisolator)
 *  - PD5/PD6 UART pins are configured by hw_uart_init()
 */
void board_init(void);

#endif /* BOARD_INIT_H */
