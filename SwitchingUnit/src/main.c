#include "debug.h"
#include "board_init.h"
#include "board_pins.h"
#include "systick.h"
#include "hw_uart.h"
#include "comm_protocol.h"
#include "relay_ctrl.h"
#include "hlw8032.h"
#include "motor_detect.h"
#include "presence.h"

/* Default baud rate to Control Unit (overridable via build_flags) */
#ifndef CTRL_UART_BAUD
#define CTRL_UART_BAUD  115200
#endif

int main(void)
{
    /* ---- Core System Init ---- */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();

    /* ---- Board-Level GPIO & Peripheral Clocks ---- */
    board_init();

    /* ---- System Timebase (SysTick + TIM2 millis) ---- */
    systick_init();

    /* ---- Hardware UART for Control Unit Communication ---- */
    hw_uart_init(CTRL_UART_BAUD);

    /* ---- Communication Protocol Parser ---- */
    comm_protocol_init();

    /* ---- Relay Controller (ensures safe state on boot) ---- */
    relay_init();

    /* ---- HLW8032 Energy Meter ---- */
    hlw8032_init();

    /* ---- Motor Detection (PD4 optoisolator) ---- */
    motor_detect_init();

    /* ---- Presence Detection (PC3 heartbeat) ---- */
    presence_init();

    /* ================================================================
     * Main Application Loop
     *
     * Execution order matters:
     *  1. comm_protocol_tick  — process incoming UART commands first
     *  2. motor_detect_tick   — update motor state before relay checks
     *  3. relay_tick          — advance relay SM (uses motor state)
     *  4. hlw8032_tick        — poll energy data (may block ~50 ms)
     *  5. presence_tick       — heartbeat pulse + CU monitoring
     * ==============================================================*/
    while (1)
    {
        comm_protocol_tick();
        motor_detect_tick();
        relay_tick();
        hlw8032_tick();
        presence_tick();
    }
}
