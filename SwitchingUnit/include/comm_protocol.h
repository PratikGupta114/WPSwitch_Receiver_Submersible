#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <stdint.h>

/* ===========================================================================
 * Frame Format (binary, compact, checksummed)
 *
 * ┌──────┬──────┬─────────┬─────┬─────────────┬──────────┐
 * │ SOF  │ Type │ Cmd/Evt │ Len │ Payload     │ Checksum │
 * │ 0xAA │ 1 B  │  1 B    │ 1 B │ 0..16 bytes │ 1 B      │
 * └──────┴──────┴─────────┴─────┴─────────────┴──────────┘
 *
 * Checksum = XOR of all bytes from Type through end of Payload.
 * ==========================================================================*/

#define PROTO_SOF               0xAA
#define PROTO_MAX_PAYLOAD       16
#define PROTO_FRAME_OVERHEAD    5   /* SOF + Type + ID + Len + Checksum */

/* --- Message Types --- */
#define PROTO_TYPE_COMMAND      0x01  /* Control Unit  → Switching Unit */
#define PROTO_TYPE_RESPONSE     0x02  /* Switching Unit → Control Unit  */
#define PROTO_TYPE_EVENT        0x03  /* Switching Unit → Control Unit  */
#define PROTO_TYPE_REQUEST      0x04  /* Control Unit  → Switching Unit (Queries) */

/* --- Command & Request IDs (Control Unit → Switching Unit) --- */
#define CMD_PUMP_ON             0x01
#define CMD_PUMP_OFF            0x02
#define REQ_GET_STATUS          0x03
#define REQ_GET_ENERGY          0x04
#define CMD_PING                0x05
#define REQ_GET_VERSION         0x06

/* --- Response Status Codes (payload byte 0 in responses) --- */
#define RESP_OK                 0x00
#define RESP_ERR_COOLDOWN       0x01
#define RESP_ERR_ALREADY_ON     0x02
#define RESP_ERR_ALREADY_OFF    0x03
#define RESP_ERR_FAULT          0x04
#define RESP_ERR_UNKNOWN_CMD    0x05
#define RESP_ERR_NOT_AVAILABLE  0x06

/* --- Event IDs (Switching Unit → Control Unit) --- */
#define EVT_PUMP_STARTED        0x01
#define EVT_PUMP_STOPPED        0x02
#define EVT_HIGH_ENERGY         0x03
#define EVT_COOLDOWN_STARTED    0x04
#define EVT_COOLDOWN_ENDED      0x05
#define EVT_FAULT               0x06
#define EVT_PRESENCE_CHANGE     0x07

/* --- Event Source Codes (used in event payloads) --- */
#define EVT_SRC_COMMAND         0x00
#define EVT_SRC_MANUAL          0x01
#define EVT_SRC_FAULT           0x02

/* --- Parsed Frame Structure --- */
typedef struct {
    uint8_t type;
    uint8_t id;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_frame_t;

/**
 * @brief Initialize protocol parser state machine.
 */
void comm_protocol_init(void);

/**
 * @brief Process incoming UART bytes and dispatch commands.
 *
 * Call from the main loop. Reads bytes from hw_uart, parses frames,
 * and dispatches valid commands to handler functions.
 */
void comm_protocol_tick(void);

/**
 * @brief Send a response frame to the Control Unit.
 * @param cmd_id   The command ID being responded to.
 * @param payload  Response payload (may be NULL if len == 0).
 * @param len      Payload length (0–16).
 */
void comm_send_response(uint8_t cmd_id, const uint8_t *payload, uint8_t len);

/**
 * @brief Send an unsolicited event frame to the Control Unit.
 * @param evt_id   Event ID.
 * @param payload  Event payload (may be NULL if len == 0).
 * @param len      Payload length (0–16).
 */
void comm_send_event(uint8_t evt_id, const uint8_t *payload, uint8_t len);

#endif /* COMM_PROTOCOL_H */
