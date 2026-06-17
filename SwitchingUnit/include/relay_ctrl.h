#ifndef RELAY_CTRL_H
#define RELAY_CTRL_H

#include <stdint.h>

/* ===========================================================================
 * Relay State Machine
 *
 *   IDLE ──(pump_on)──▶ STARTING ──(3 s)──▶ RUNNING
 *     ▲                    │                    │
 *     │                    │(pump_off)          │(pump_off / ext. stop)
 *     │                    ▼                    ▼
 *     └──(170 s)── COOLDOWN ◀── STOPPING ──(200 ms)──┘
 * ==========================================================================*/

typedef enum {
    RELAY_STATE_IDLE = 0,   /* All relays off, ready for commands          */
    RELAY_STATE_STARTING,   /* Large NO + Small NO active (max 3 s)       */
    RELAY_STATE_RUNNING,    /* Small NO active, motor confirmed running    */
    RELAY_STATE_STOPPING,   /* NC relay pulsed to break circuit (~200 ms) */
    RELAY_STATE_COOLDOWN    /* All relays off, 170 s wait before re-start */
} relay_state_t;

typedef enum {
    RELAY_OK = 0,
    RELAY_ERR_COOLDOWN,
    RELAY_ERR_ALREADY_ON,
    RELAY_ERR_ALREADY_OFF,
    RELAY_ERR_FAULT
} relay_result_t;

/** @brief Initialize relay GPIOs to safe OFF state and enter IDLE. */
void relay_init(void);

/**
 * @brief Request pump start.
 * @return RELAY_OK on success, error code otherwise.
 */
relay_result_t relay_pump_on(void);

/**
 * @brief Request pump stop.
 * @return RELAY_OK on success, error code otherwise.
 */
relay_result_t relay_pump_off(void);

/**
 * @brief Advance the relay state machine (call from main loop).
 *
 * Handles timing for STARTING (3 s), STOPPING (200 ms), and
 * COOLDOWN (170 s) phases. Also monitors motor detection feedback
 * for external stop detection during RUNNING state.
 */
void relay_tick(void);

/** @brief Get the current relay state. */
relay_state_t relay_get_state(void);

/** @brief Get seconds remaining in cooldown (0 if not in cooldown). */
uint16_t relay_get_cooldown_remaining(void);

#endif /* RELAY_CTRL_H */
