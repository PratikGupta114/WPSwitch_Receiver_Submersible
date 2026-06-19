#ifndef RELAY_CTRL_H
#define RELAY_CTRL_H

#include <stdint.h>

/* ===========================================================================
 * Relay State Machine
 *
 *                    CMD_PUMP_LOCK (from any state)
 *                          │
 *                          ▼
 *   IDLE ──(pump_on)──▶ STARTING ──(3 s)──▶ RUNNING
 *     ▲                    │                    │
 *     │                    │(pump_off)          │(pump_off / ext. stop)
 *     │                    ▼                    ▼
 *     └──(170 s)── COOLDOWN ◀── STOPPING ──(500 ms)──┘
 *     ▲
 *     │
 *   LOCKED ──(pump_unlock)──┘
 * ==========================================================================*/

typedef enum {
    RELAY_STATE_IDLE = 0,   /* All relays off, ready for commands          */
    RELAY_STATE_STARTING,   /* Large NO + Small NO active (max 3 s)       */
    RELAY_STATE_RUNNING,    /* Small NO active, motor confirmed running    */
    RELAY_STATE_STOPPING,   /* NC relay pulsed to break circuit (~500 ms) */
    RELAY_STATE_COOLDOWN,   /* All relays off, 170 s wait before re-start */
    RELAY_STATE_LOCKED      /* NC relay held energized — pump disabled    */
} relay_state_t;

typedef enum {
    RELAY_OK = 0,
    RELAY_ERR_COOLDOWN,
    RELAY_ERR_ALREADY_ON,
    RELAY_ERR_ALREADY_OFF,
    RELAY_ERR_FAULT,
    RELAY_ERR_LOCKED
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
 * @brief Engage pump lockout (NC relay held energized, pump disabled).
 *
 * Accepted from any state. All NO relays are de-energized, NC relay
 * is activated (contact opens), and the state machine enters LOCKED.
 * The lock persists until an explicit CMD_PUMP_UNLOCK command.
 *
 * @return RELAY_OK always.
 */
relay_result_t relay_pump_lock(void);

/**
 * @brief Release pump lockout.
 *
 * NC relay is deactivated (contact closes), state returns to IDLE.
 *
 * @return RELAY_OK on success, RELAY_ERR_ALREADY_OFF if not locked.
 */
relay_result_t relay_pump_unlock(void);

/**
 * @brief Advance the relay state machine (call from main loop).
 *
 * Handles timing for STARTING (3 s), STOPPING (500 ms), and
 * COOLDOWN (170 s) phases. Also monitors motor detection feedback
 * for external stop detection during RUNNING state.
 * LOCKED state is stable — no auto-transition.
 */
void relay_tick(void);

/** @brief Get the current relay state. */
relay_state_t relay_get_state(void);

/** @brief Get seconds remaining in cooldown (0 if not in cooldown). */
uint16_t relay_get_cooldown_remaining(void);

#endif /* RELAY_CTRL_H */

