#ifndef PRESENCE_H
#define PRESENCE_H

#include <stdint.h>

/**
 * @brief Initialize presence detection on PC3.
 *
 * Configures the open-drain heartbeat protocol:
 *  - Switching Unit pulses PC3 LOW for 2 ms every 100 ms.
 *  - Between pulses, monitors for LOW pulses from the Control Unit.
 *  - If no CU pulse detected for 500 ms, CU is considered absent.
 */
void presence_init(void);

/**
 * @brief Run presence detection logic (call from main loop).
 *
 * Outputs heartbeat pulses and monitors for Control Unit pulses.
 * Fires EVT_PRESENCE_CHANGE events on state transitions.
 */
void presence_tick(void);

/**
 * @brief Check if Control Unit is present.
 * @return 1 if present (pulses detected within timeout), 0 if absent.
 */
uint8_t presence_is_control_unit_present(void);

/** @brief Check if Switching Unit is currently outputting its own pulse. */
uint8_t presence_is_pulsing_active(void);

/** @brief Record that a pulse was received from the Control Unit (called by EXTI ISR). */
void presence_record_cu_seen(void);

#endif /* PRESENCE_H */
