#include "motor_detect.h"
#include "board_pins.h"
#include "systick.h"
#include "relay_ctrl.h"
#include "comm_protocol.h"

/* ---------------------------------------------------------------------------
 * Debounce Parameters
 * -------------------------------------------------------------------------*/
#define DEBOUNCE_INTERVAL_MS    10  /* Sample PD4 every 10 ms              */
#define DEBOUNCE_THRESHOLD       3  /* 3 consecutive consistent readings   */

/* ---------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------*/
static uint8_t  current_state   = 0; /* 0 = motor off, 1 = motor running   */
static uint8_t  last_sample     = 0;
static uint8_t  sample_count    = 0;
static uint32_t last_sample_ms  = 0;

void motor_detect_init(void)
{
    current_state  = motor_detect_pin_read() ? 1 : 0;
    last_sample    = current_state;
    sample_count   = DEBOUNCE_THRESHOLD;
    last_sample_ms = millis();
}

void motor_detect_tick(void)
{
    if (!elapsed(&last_sample_ms, DEBOUNCE_INTERVAL_MS)) {
        return;
    }

    uint8_t raw = motor_detect_pin_read() ? 1 : 0;

    if (raw == last_sample) {
        if (sample_count < DEBOUNCE_THRESHOLD) {
            sample_count++;
        }
    } else {
        last_sample  = raw;
        sample_count = 1;
    }

    /* Accept new state only after stable readings */
    if (sample_count >= DEBOUNCE_THRESHOLD && raw != current_state) {
        uint8_t prev = current_state;
        current_state = raw;

        relay_state_t rstate = relay_get_state();

        if (current_state == 1 && prev == 0) {
            /* ----- Motor just started ----- */
            uint8_t src;
            if (rstate == RELAY_STATE_STARTING || rstate == RELAY_STATE_RUNNING) {
                src = EVT_SRC_COMMAND;   /* We initiated this start */
            } else {
                src = EVT_SRC_MANUAL;    /* Manual Green button press */
            }
            comm_send_event(EVT_PUMP_STARTED, &src, 1);

        } else if (current_state == 0 && prev == 1) {
            /* ----- Motor just stopped ----- */
            uint8_t src;
            if (rstate == RELAY_STATE_STOPPING) {
                src = EVT_SRC_COMMAND;   /* We commanded the stop */
            } else if (rstate == RELAY_STATE_RUNNING ||
                       rstate == RELAY_STATE_STARTING) {
                src = EVT_SRC_FAULT;     /* External / unexpected stop */
            } else {
                src = EVT_SRC_MANUAL;    /* Stopped from a manual start */
            }
            comm_send_event(EVT_PUMP_STOPPED, &src, 1);
        }
    }
}

uint8_t motor_is_running(void)
{
    return current_state;
}
