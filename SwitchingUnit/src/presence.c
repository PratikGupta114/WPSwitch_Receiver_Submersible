#include "presence.h"
#include "board_pins.h"
#include "systick.h"
#include "comm_protocol.h"

/* ---------------------------------------------------------------------------
 * Presence Detection Protocol (single open-drain wire on PC3)
 *
 * The Switching Unit outputs a 2 ms LOW pulse every 100 ms as its
 * heartbeat.  Between its own pulses it monitors for LOW pulses from
 * the Control Unit (which runs an identical protocol with a 50 ms
 * offset to avoid collision).
 *
 * If no Control Unit pulse is detected within 500 ms, the CU is
 * considered absent.
 * -------------------------------------------------------------------------*/

#define PULSE_PERIOD_MS     100  /* Heartbeat interval       */
#define PULSE_WIDTH_MS        2  /* How long to hold LOW     */
#define CU_TIMEOUT_MS       500  /* CU absent if no pulse    */

/* ---------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------*/
static uint32_t last_pulse_ms   = 0;
static uint32_t last_cu_seen_ms = 0;
static volatile uint8_t  pulse_active    = 0;
static uint8_t  cu_present      = 0;

void presence_init(void)
{
    presence_pin_release();          /* Start released (HIGH) */
    last_pulse_ms   = millis();
    last_cu_seen_ms = millis();
    cu_present      = 0;
    pulse_active    = 0;
}

void presence_tick(void)
{
    uint32_t now = millis();

    /* --- Output: send our heartbeat pulse --- */
    if (!pulse_active) {
        if ((now - last_pulse_ms) >= PULSE_PERIOD_MS) {
            presence_pin_drive_low();
            pulse_active  = 1;
            last_pulse_ms = now;
        }
    } else {
        if ((now - last_pulse_ms) >= PULSE_WIDTH_MS) {
            presence_pin_release();
            pulse_active = 0;
        }
    }

    /* --- Evaluate CU presence --- */
    uint8_t was_present = cu_present;

    /*
     * Disabling interrupts briefly is not strictly necessary for 32-bit reads,
     * but prevents compiler reordering/cache of last_cu_seen_ms which is modified by ISR.
     */
    uint32_t cu_seen;
    __disable_irq();
    cu_seen = last_cu_seen_ms;
    __enable_irq();

    if ((now - cu_seen) >= CU_TIMEOUT_MS) {
        cu_present = 0;
    } else {
        cu_present = 1;
    }

    /* Fire event on state change */
    if (cu_present != was_present) {
        comm_send_event(EVT_PRESENCE_CHANGE, &cu_present, 1);
    }
}

uint8_t presence_is_control_unit_present(void)
{
    return cu_present;
}

uint8_t presence_is_pulsing_active(void)
{
    return pulse_active;
}

void presence_record_cu_seen(void)
{
    last_cu_seen_ms = millis();
}

/* ---------------------------------------------------------------------------
 * EXTI 7_0 IRQ Handler (PC3 falls when CU drives it LOW)
 * -------------------------------------------------------------------------*/
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line3);
        /* Only record if we are not actively outputting our own pulse */
        if (!pulse_active) {
            last_cu_seen_ms = millis();
        }
    }
}
