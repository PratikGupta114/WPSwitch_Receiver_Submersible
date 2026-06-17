#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

/**
 * @brief Initialize system timebase.
 *
 * Configures SysTick as a free-running up-counter at HCLK (48 MHz) for
 * software UART bit timing, and TIM2 with a 1 ms interrupt for the
 * millis() counter used by all application timing.
 *
 * Call this INSTEAD of Delay_Init() — do NOT call Delay_Ms() after this.
 */
void systick_init(void);

/**
 * @brief Get current millisecond count since boot.
 * @return Monotonically increasing 32-bit ms counter (wraps at ~49.7 days).
 *
 * @note Counter may drift by up to ~50 ms during HLW8032 software-UART
 *       reception (interrupts disabled). Acceptable for relay timing
 *       (3 s / 170 s scales).
 */
uint32_t millis(void);

/**
 * @brief Non-blocking periodic check.
 * @param last  Pointer to timestamp variable (updated when period elapses).
 * @param period_ms  Period in milliseconds.
 * @return 1 if period has elapsed since *last, 0 otherwise.
 */
uint8_t elapsed(uint32_t *last, uint32_t period_ms);

/**
 * @brief Blocking delay using the TIM2-based millis counter.
 * @param ms  Delay in milliseconds.
 *
 * @warning Do not use in ISR context.
 */
void delay_ms_blocking(uint32_t ms);

#endif /* SYSTICK_H */
