#include "comm_protocol.h"
#include "hw_uart.h"
#include "relay_ctrl.h"
#include "hlw8032.h"
#include "motor_detect.h"
#include "presence.h"
#include "systick.h"

#ifndef FW_VERSION_MAJOR
#define FW_VERSION_MAJOR  0
#endif
#ifndef FW_VERSION_MINOR
#define FW_VERSION_MINOR  1
#endif
#ifndef FW_VERSION_PATCH
#define FW_VERSION_PATCH  0
#endif

/* ---------------------------------------------------------------------------
 * Frame Parser State Machine
 * -------------------------------------------------------------------------*/
typedef enum {
    PARSE_WAIT_SOF,
    PARSE_WAIT_TYPE,
    PARSE_WAIT_ID,
    PARSE_WAIT_LEN,
    PARSE_RECV_PAYLOAD,
    PARSE_WAIT_CHECKSUM
} parse_state_t;

static parse_state_t parse_state   = PARSE_WAIT_SOF;
static proto_frame_t rx_frame;
static uint8_t       payload_idx;
static uint8_t       calc_checksum;
static uint32_t      frame_start_ms;

/** Discard incomplete frame after this many ms with no new byte */
#define FRAME_TIMEOUT_MS  50

/* Forward declaration */
static void handle_command(const proto_frame_t *frame);
static void handle_request(const proto_frame_t *frame);

/* ---------------------------------------------------------------------------
 * Frame Transmitter
 * -------------------------------------------------------------------------*/
static void send_frame(uint8_t type, uint8_t id,
                       const uint8_t *payload, uint8_t len)
{
    uint8_t checksum = type ^ id ^ len;

    hw_uart_send_byte(PROTO_SOF);
    hw_uart_send_byte(type);
    hw_uart_send_byte(id);
    hw_uart_send_byte(len);

    for (uint8_t i = 0; i < len; i++) {
        hw_uart_send_byte(payload[i]);
        checksum ^= payload[i];
    }

    hw_uart_send_byte(checksum);
}

void comm_send_response(uint8_t cmd_id, const uint8_t *payload, uint8_t len)
{
    send_frame(PROTO_TYPE_RESPONSE, cmd_id, payload, len);
}

void comm_send_event(uint8_t evt_id, const uint8_t *payload, uint8_t len)
{
    send_frame(PROTO_TYPE_EVENT, evt_id, payload, len);
}

/* ---------------------------------------------------------------------------
 * Protocol Tick — Parse incoming bytes & dispatch
 * -------------------------------------------------------------------------*/
void comm_protocol_init(void)
{
    parse_state = PARSE_WAIT_SOF;
}

void comm_protocol_tick(void)
{
    /* Timeout guard: discard incomplete frame */
    if (parse_state != PARSE_WAIT_SOF) {
        if ((millis() - frame_start_ms) > FRAME_TIMEOUT_MS) {
            parse_state = PARSE_WAIT_SOF;
        }
    }

    /* Process all available bytes from the hardware UART */
    int16_t b;
    while ((b = hw_uart_read_byte()) >= 0) {
        uint8_t byte = (uint8_t)b;

        switch (parse_state) {
        case PARSE_WAIT_SOF:
            if (byte == PROTO_SOF) {
                parse_state    = PARSE_WAIT_TYPE;
                frame_start_ms = millis();
            }
            break;

        case PARSE_WAIT_TYPE:
            rx_frame.type = byte;
            calc_checksum = byte;
            parse_state   = PARSE_WAIT_ID;
            break;

        case PARSE_WAIT_ID:
            rx_frame.id    = byte;
            calc_checksum ^= byte;
            parse_state    = PARSE_WAIT_LEN;
            break;

        case PARSE_WAIT_LEN:
            rx_frame.len   = byte;
            calc_checksum ^= byte;
            if (byte > PROTO_MAX_PAYLOAD) {
                parse_state = PARSE_WAIT_SOF; /* Invalid length */
            } else if (byte == 0) {
                parse_state = PARSE_WAIT_CHECKSUM;
            } else {
                payload_idx = 0;
                parse_state = PARSE_RECV_PAYLOAD;
            }
            break;

        case PARSE_RECV_PAYLOAD:
            rx_frame.payload[payload_idx++] = byte;
            calc_checksum ^= byte;
            if (payload_idx >= rx_frame.len) {
                parse_state = PARSE_WAIT_CHECKSUM;
            }
            break;

        case PARSE_WAIT_CHECKSUM:
            if (byte == calc_checksum) {
                /* Valid frame — dispatch commands or requests */
                if (rx_frame.type == PROTO_TYPE_COMMAND) {
                    handle_command(&rx_frame);
                } else if (rx_frame.type == PROTO_TYPE_REQUEST) {
                    handle_request(&rx_frame);
                }
            }
            /* Always return to SOF hunt after checksum byte */
            parse_state = PARSE_WAIT_SOF;
            break;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Command Handler / Dispatcher
 * -------------------------------------------------------------------------*/
static void handle_command(const proto_frame_t *frame)
{
    uint8_t resp[PROTO_MAX_PAYLOAD];

    switch (frame->id) {

    /* --- CMD_PUMP_ON --------------------------------------------------- */
    case CMD_PUMP_ON: {
        relay_result_t result = relay_pump_on();
        switch (result) {
        case RELAY_OK:
            resp[0] = RESP_OK;
            comm_send_response(CMD_PUMP_ON, resp, 1);
            break;
        case RELAY_ERR_COOLDOWN: {
            resp[0] = RESP_ERR_COOLDOWN;
            uint16_t remaining = relay_get_cooldown_remaining();
            resp[1] = (uint8_t)(remaining >> 8);
            resp[2] = (uint8_t)(remaining & 0xFF);
            comm_send_response(CMD_PUMP_ON, resp, 3);
            break;
        }
        case RELAY_ERR_ALREADY_ON:
            resp[0] = RESP_ERR_ALREADY_ON;
            comm_send_response(CMD_PUMP_ON, resp, 1);
            break;
        default:
            resp[0] = RESP_ERR_FAULT;
            comm_send_response(CMD_PUMP_ON, resp, 1);
            break;
        }
        break;
    }

    /* --- CMD_PUMP_OFF -------------------------------------------------- */
    case CMD_PUMP_OFF: {
        relay_result_t result = relay_pump_off();
        switch (result) {
        case RELAY_OK:
            resp[0] = RESP_OK;
            break;
        case RELAY_ERR_ALREADY_OFF:
            resp[0] = RESP_ERR_ALREADY_OFF;
            break;
        default:
            resp[0] = RESP_ERR_FAULT;
            break;
        }
        comm_send_response(CMD_PUMP_OFF, resp, 1);
        break;
    }

    /* --- CMD_PING ------------------------------------------------------ */
    case CMD_PING:
        resp[0] = RESP_OK;
        comm_send_response(CMD_PING, resp, 1);
        break;

    /* --- Unknown Command ----------------------------------------------- */
    default:
        resp[0] = RESP_ERR_UNKNOWN_CMD;
        comm_send_response(frame->id, resp, 1);
        break;
    }
}

static void handle_request(const proto_frame_t *frame)
{
    uint8_t resp[PROTO_MAX_PAYLOAD];

    switch (frame->id) {

    /* --- REQ_GET_STATUS ------------------------------------------------ */
    case REQ_GET_STATUS:
        resp[0] = (uint8_t)relay_get_state();
        resp[1] = motor_is_running();
        resp[2] = presence_is_control_unit_present();
        {
            uint16_t cd = relay_get_cooldown_remaining();
            resp[3] = (uint8_t)(cd >> 8);
            resp[4] = (uint8_t)(cd & 0xFF);
        }
        comm_send_response(REQ_GET_STATUS, resp, 5);
        break;

    /* --- REQ_GET_ENERGY ------------------------------------------------ */
    case REQ_GET_ENERGY: {
#if FEATURE_HLW8032
        const hlw8032_data_t *edata = hlw8032_get_data();
        resp[0]  = edata->valid;
        /* Voltage parameter (3 B) + data (3 B) */
        resp[1]  = edata->voltage_reg[0];
        resp[2]  = edata->voltage_reg[1];
        resp[3]  = edata->voltage_reg[2];
        resp[4]  = edata->voltage_data[0];
        resp[5]  = edata->voltage_data[1];
        resp[6]  = edata->voltage_data[2];
        /* Current parameter (3 B) + data (3 B) */
        resp[7]  = edata->current_reg[0];
        resp[8]  = edata->current_reg[1];
        resp[9]  = edata->current_reg[2];
        resp[10] = edata->current_data[0];
        resp[11] = edata->current_data[1];
        resp[12] = edata->current_data[2];
        comm_send_response(REQ_GET_ENERGY, resp, 13);
#else
        resp[0] = RESP_ERR_NOT_AVAILABLE;
        comm_send_response(REQ_GET_ENERGY, resp, 1);
#endif
        break;
    }

    /* --- REQ_GET_VERSION ----------------------------------------------- */
    case REQ_GET_VERSION:
        resp[0] = FW_VERSION_MAJOR;
        resp[1] = FW_VERSION_MINOR;
        resp[2] = FW_VERSION_PATCH;
        comm_send_response(REQ_GET_VERSION, resp, 3);
        break;

    /* --- Unknown Request ----------------------------------------------- */
    default:
        resp[0] = RESP_ERR_UNKNOWN_CMD;
        comm_send_response(frame->id, resp, 1);
        break;
    }
}
