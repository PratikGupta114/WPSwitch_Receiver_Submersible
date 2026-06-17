#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "ch32v00x.h"

/*=============================================================================
 * Pin Assignments — Switching Unit PCB
 * MCU: CH32V003A4M6 (SOP16)
 *
 * Pin  1 : PC1  — Small Relay NO   (contactor coil parallel switch)
 * Pin  2 : PC2  — Small Relay NC   (series with Red stop button)
 * Pin  3 : PC3  — Presence Pin     (mutual detection with Control Unit)
 * Pin  4 : PC4  — Not Used
 * Pin  5 : PC6  — Software UART RX (HLW8032 data output)
 * Pin  6 : PC7  — HLW8032 Pulse    (CF energy pulse output)
 * Pin  7 : PD1  — SWIO Debug       (WCH-LinkE programmer)
 * Pin  8 : PD4  — AC Line Feedback (optoisolator — motor on/off)
 * Pin  9 : PD5  — Hardware UART TX (to Control Unit)
 * Pin 10 : PD6  — Hardware UART RX (from Control Unit)
 * Pin 11 : PD7  — NRST             (reset)
 * Pin 12 : PA1  — Not Used
 * Pin 13 : PA2  — Not Used
 * Pin 14 : VSS
 * Pin 15 : VDD
 * Pin 16 : PC0  — Large Relay NO   (starter winding parallel switch)
 *===========================================================================*/

/* ---------------------------------------------------------------------------
 * Relay Outputs (Active-High: GPIO HIGH = coil energized)
 * -------------------------------------------------------------------------*/
#define RELAY_LARGE_NO_PORT     GPIOC
#define RELAY_LARGE_NO_PIN      GPIO_Pin_0  /* PC0 — Starter winding */

#define RELAY_SMALL_NO_PORT     GPIOC
#define RELAY_SMALL_NO_PIN      GPIO_Pin_1  /* PC1 — Contactor coil */

#define RELAY_NC_PORT           GPIOC
#define RELAY_NC_PIN            GPIO_Pin_2  /* PC2 — Series with Red stop */

/* ---------------------------------------------------------------------------
 * Presence Detection
 * -------------------------------------------------------------------------*/
#define PRESENCE_PORT           GPIOC
#define PRESENCE_PIN            GPIO_Pin_3  /* PC3 — Open-drain heartbeat */

/* ---------------------------------------------------------------------------
 * HLW8032 Energy Meter
 * -------------------------------------------------------------------------*/
#define HLW8032_RX_PORT         GPIOC
#define HLW8032_RX_PIN          GPIO_Pin_6  /* PC6 — Software UART RX */

#define HLW8032_PF_PORT         GPIOC
#define HLW8032_PF_PIN          GPIO_Pin_7  /* PC7 — CF pulse output */

/* ---------------------------------------------------------------------------
 * Motor Detection (Optoisolator)
 * -------------------------------------------------------------------------*/
#define MOTOR_DETECT_PORT       GPIOD
#define MOTOR_DETECT_PIN        GPIO_Pin_4  /* PD4 — HIGH = motor running */

/* ---------------------------------------------------------------------------
 * Hardware UART (Control Unit Communication)
 * -------------------------------------------------------------------------*/
#define UART_TX_PORT            GPIOD
#define UART_TX_PIN             GPIO_Pin_5  /* PD5 — USART1 TX */

#define UART_RX_PORT            GPIOD
#define UART_RX_PIN             GPIO_Pin_6  /* PD6 — USART1 RX */

/* ===========================================================================
 * Inline Helpers — Relay Control
 * Active-high: Bit_SET = relay coil energized
 * ==========================================================================*/

static inline void relay_large_no_on(void) {
    GPIO_WriteBit(RELAY_LARGE_NO_PORT, RELAY_LARGE_NO_PIN, Bit_SET);
}
static inline void relay_large_no_off(void) {
    GPIO_WriteBit(RELAY_LARGE_NO_PORT, RELAY_LARGE_NO_PIN, Bit_RESET);
}

static inline void relay_small_no_on(void) {
    GPIO_WriteBit(RELAY_SMALL_NO_PORT, RELAY_SMALL_NO_PIN, Bit_SET);
}
static inline void relay_small_no_off(void) {
    GPIO_WriteBit(RELAY_SMALL_NO_PORT, RELAY_SMALL_NO_PIN, Bit_RESET);
}

/* Activating NC relay OPENS the contact (breaks circuit = like pressing Red) */
static inline void relay_nc_activate(void) {
    GPIO_WriteBit(RELAY_NC_PORT, RELAY_NC_PIN, Bit_SET);
}
/* Deactivating NC relay CLOSES the contact (normal operating state) */
static inline void relay_nc_deactivate(void) {
    GPIO_WriteBit(RELAY_NC_PORT, RELAY_NC_PIN, Bit_RESET);
}

/* ===========================================================================
 * Inline Helpers — Presence Pin (Open-Drain)
 * ==========================================================================*/

static inline void presence_pin_drive_low(void) {
    GPIO_WriteBit(PRESENCE_PORT, PRESENCE_PIN, Bit_RESET);
}
static inline void presence_pin_release(void) {
    GPIO_WriteBit(PRESENCE_PORT, PRESENCE_PIN, Bit_SET);
}
static inline uint8_t presence_pin_read(void) {
    return GPIO_ReadInputDataBit(PRESENCE_PORT, PRESENCE_PIN);
}

/* ===========================================================================
 * Inline Helpers — Motor Detection
 * ==========================================================================*/

static inline uint8_t motor_detect_pin_read(void) {
    return GPIO_ReadInputDataBit(MOTOR_DETECT_PORT, MOTOR_DETECT_PIN);
}

/* ===========================================================================
 * Inline Helpers — HLW8032
 * ==========================================================================*/

static inline uint8_t hlw8032_rx_read(void) {
    return GPIO_ReadInputDataBit(HLW8032_RX_PORT, HLW8032_RX_PIN);
}
static inline uint8_t hlw8032_pf_read(void) {
    return GPIO_ReadInputDataBit(HLW8032_PF_PORT, HLW8032_PF_PIN);
}

#endif /* BOARD_PINS_H */
