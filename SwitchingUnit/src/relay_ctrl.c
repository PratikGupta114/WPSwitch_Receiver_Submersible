#include "relay_ctrl.h"
#include "board_pins.h"
#include "systick.h"
#include "comm_protocol.h"
#include "motor_detect.h"

/* ---------------------------------------------------------------------------
 * Timing Constants
 * -------------------------------------------------------------------------*/
#define STARTER_MAX_MS      3000UL   /* 3 s max for starter winding         */
#define STOP_PULSE_MS       500UL    /* NC relay pulse to break coil circuit */
#define COOLDOWN_MS         170000UL /* 170 s before next start allowed     */
#define MOTOR_GRACE_MS      2000UL   /* Grace period after entering RUNNING
                                        before checking motor feedback      */

/* ---------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------*/
static relay_state_t state          = RELAY_STATE_IDLE;
static uint32_t      state_enter_ms = 0;

static void enter_state(relay_state_t new_state)
{
    state          = new_state;
    state_enter_ms = millis();
}

/** Drive all relay outputs to safe OFF state */
static void all_relays_safe(void)
{
    relay_large_no_off();    /* PC0 OFF */
    relay_small_no_off();    /* PC1 OFF */
    relay_nc_deactivate();   /* PC2 OFF → NC contact closed (normal) */
}

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

void relay_init(void)
{
    all_relays_safe();
    enter_state(RELAY_STATE_IDLE);
}

relay_result_t relay_pump_on(void)
{
    switch (state) {
    case RELAY_STATE_IDLE:
        /*
         * Startup sequence — simulates pressing the Green button:
         *  1. Activate Large NO relay (PC0) → power to starter winding
         *  2. Activate Small NO relay (PC1) → power to contactor coil
         * The starter winding relay will be released after STARTER_MAX_MS
         * by relay_tick().
         */
        relay_large_no_on();
        relay_small_no_on();
        enter_state(RELAY_STATE_STARTING);
        return RELAY_OK;

    case RELAY_STATE_STARTING:
    case RELAY_STATE_RUNNING:
        return RELAY_ERR_ALREADY_ON;

    case RELAY_STATE_COOLDOWN:
    case RELAY_STATE_STOPPING:
        return RELAY_ERR_COOLDOWN;

    case RELAY_STATE_LOCKED:
        return RELAY_ERR_LOCKED;

    default:
        return RELAY_ERR_FAULT;
    }
}

relay_result_t relay_pump_off(void)
{
    switch (state) {
    case RELAY_STATE_STARTING:
    case RELAY_STATE_RUNNING:
        /*
         * Stop sequence — simulates pressing the Red button:
         *  1. Deactivate both NO relays (remove our parallel paths)
         *  2. Activate NC relay → opens the NC series contact,
         *     breaking the contactor coil circuit
         * The NC relay will be released after STOP_PULSE_MS by relay_tick().
         */
        relay_large_no_off();
        relay_small_no_off();
        relay_nc_activate();
        enter_state(RELAY_STATE_STOPPING);
        return RELAY_OK;

    case RELAY_STATE_IDLE:
    case RELAY_STATE_COOLDOWN:
    case RELAY_STATE_STOPPING:
        return RELAY_ERR_ALREADY_OFF;

    case RELAY_STATE_LOCKED:
        return RELAY_ERR_LOCKED;

    default:
        return RELAY_ERR_FAULT;
    }
}

relay_result_t relay_pump_lock(void)
{
    /* Lock is accepted from any state. Safe all NO relays, energize NC. */
    relay_large_no_off();
    relay_small_no_off();
    relay_nc_activate();
    enter_state(RELAY_STATE_LOCKED);
    return RELAY_OK;
}

relay_result_t relay_pump_unlock(void)
{
    if (state != RELAY_STATE_LOCKED) {
        return RELAY_ERR_ALREADY_OFF;
    }
    relay_nc_deactivate();
    enter_state(RELAY_STATE_IDLE);
    return RELAY_OK;
}

void relay_tick(void)
{
    uint32_t elapsed_ms = millis() - state_enter_ms;

    switch (state) {

    case RELAY_STATE_IDLE:
        /* Nothing to do — waiting for pump_on command */
        break;

    case RELAY_STATE_STARTING:
        if (elapsed_ms >= STARTER_MAX_MS) {
            /*
             * Starter winding has been energized for 3 s — disconnect it.
             * The contactor coil relay (Small NO / PC1) stays active.
             * The contactor's own holding contact + our Small NO relay
             * keep the coil energized.
             */
            relay_large_no_off();
            enter_state(RELAY_STATE_RUNNING);
        }
        break;

    case RELAY_STATE_RUNNING:
        /*
         * Monitor motor feedback (PD4 optoisolator).
         * Allow a grace period after entering RUNNING so the motor
         * detection debounce settles.
         */
        if (elapsed_ms > MOTOR_GRACE_MS && !motor_is_running()) {
            /* Motor stopped externally (Red button, fault, overload) */
            all_relays_safe();
            enter_state(RELAY_STATE_COOLDOWN);
            {
                uint16_t cd_secs = (uint16_t)(COOLDOWN_MS / 1000);
                uint8_t payload[2] = {
                    (uint8_t)(cd_secs >> 8),
                    (uint8_t)(cd_secs & 0xFF)
                };
                comm_send_event(EVT_COOLDOWN_STARTED, payload, 2);
            }
        }
        break;

    case RELAY_STATE_STOPPING:
        if (elapsed_ms >= STOP_PULSE_MS) {
            /* Release NC relay → contact closes (normal state) */
            relay_nc_deactivate();
            enter_state(RELAY_STATE_COOLDOWN);
            {
                uint16_t cd_secs = (uint16_t)(COOLDOWN_MS / 1000);
                uint8_t payload[2] = {
                    (uint8_t)(cd_secs >> 8),
                    (uint8_t)(cd_secs & 0xFF)
                };
                comm_send_event(EVT_COOLDOWN_STARTED, payload, 2);
            }
        }
        break;

    case RELAY_STATE_COOLDOWN:
        if (elapsed_ms >= COOLDOWN_MS) {
            enter_state(RELAY_STATE_IDLE);
            comm_send_event(EVT_COOLDOWN_ENDED, (void *)0, 0);
        }
        break;

    case RELAY_STATE_LOCKED:
        /* Stable state — NC relay held energized, no auto-transition. */
        break;
    }
}

relay_state_t relay_get_state(void)
{
    return state;
}

uint16_t relay_get_cooldown_remaining(void)
{
    if (state != RELAY_STATE_COOLDOWN) {
        return 0;
    }
    uint32_t elapsed_ms = millis() - state_enter_ms;
    if (elapsed_ms >= COOLDOWN_MS) {
        return 0;
    }
    return (uint16_t)((COOLDOWN_MS - elapsed_ms) / 1000);
}
