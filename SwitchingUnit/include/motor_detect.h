#ifndef MOTOR_DETECT_H
#define MOTOR_DETECT_H

#include <stdint.h>

/**
 * @brief Initialize motor detection driver.
 *
 * Reads the initial state of PD4 (optoisolator output) and seeds
 * the debounce filter.
 */
void motor_detect_init(void);

/**
 * @brief Debounce and detect motor state changes (call from main loop).
 *
 * Samples PD4 every 10 ms. Requires 3 consecutive consistent readings
 * before accepting a state change. On transitions, sends appropriate
 * EVT_PUMP_STARTED / EVT_PUMP_STOPPED events via comm_protocol.
 */
void motor_detect_tick(void);

/**
 * @brief Check whether the motor is currently running.
 * @return 1 if running (PD4 = HIGH, debounced), 0 if off.
 */
uint8_t motor_is_running(void);

#endif /* MOTOR_DETECT_H */
