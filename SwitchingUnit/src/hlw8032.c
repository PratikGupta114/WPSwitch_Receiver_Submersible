#include "hlw8032.h"
#include "sw_uart.h"
#include "systick.h"

static hlw8032_data_t hlw_data;
static uint8_t        pkt_buf[24];

typedef enum {
    HLW_STATE_IDLE,
    HLW_STATE_RECVING
} hlw_state_t;

static hlw_state_t hlw_state            = HLW_STATE_IDLE;
static uint32_t    last_read_ms         = 0;
static uint32_t    pkt_start_ms         = 0;
static uint8_t     pkt_idx              = 0;
static uint32_t    last_valid_packet_ms = 0;

void hlw8032_init(void)
{
    sw_uart_init();

    hlw_data.valid        = 0;
    hlw_data.packet_count = 0;
    hlw_data.state_reg    = 0;
    hlw_data.data_update  = 0;

    hlw_state            = HLW_STATE_IDLE;
    last_read_ms         = 0;
    pkt_idx              = 0;
    last_valid_packet_ms = 0;
}

void hlw8032_tick(void)
{
    uint32_t now = millis();

    /* Timeout check: if we haven't seen a valid packet in 5 seconds, invalidate the data */
    if (hlw_data.valid && (now - last_valid_packet_ms > 5000UL)) {
        hlw_data.valid = 0;
    }

    if (hlw_state == HLW_STATE_IDLE) {
        if ((now - last_read_ms) >= 2000UL) {
            hlw_state    = HLW_STATE_RECVING;
            pkt_idx      = 0;
            pkt_start_ms = now;
        }
        return;
    }

    /* Check for packet receive timeout (150 ms limit to receive/align the packet) */
    if ((now - pkt_start_ms) > 150UL) {
        pkt_idx      = 0;
        pkt_start_ms = now;
    }

    /* Poll for a single byte */
    int16_t b = sw_uart_rx_byte();
    if (b < 0) {
        return; /* No start bit detected yet */
    }

    uint8_t byte = (uint8_t)b;

    /* Align and accumulate packet */
    if (pkt_idx == 0) {
        /* Byte 0 must be State REG: 0x55, 0xAA, or 0xFx */
        if (byte == 0x55 || byte == 0xAA || (byte & 0xF0) == 0xF0) {
            pkt_buf[0]   = byte;
            pkt_idx      = 1;
            pkt_start_ms = now; /* restart timeout window when header is seen */
        }
    }
    else if (pkt_idx == 1) {
        /* Byte 1 must be Check REG (0x5A) */
        if (byte == 0x5A) {
            pkt_buf[1] = byte;
            pkt_idx    = 2;
        } else {
            /* If this byte itself happens to be a valid State REG, treat it as byte 0 */
            if (byte == 0x55 || byte == 0xAA || (byte & 0xF0) == 0xF0) {
                pkt_buf[0]   = byte;
                pkt_idx      = 1;
                pkt_start_ms = now;
            } else {
                pkt_idx = 0;
            }
        }
    }
    else {
        /* Bytes 2..23 */
        pkt_buf[pkt_idx++] = byte;

        if (pkt_idx == 24) {
            /* Fully received, process packet */
            uint8_t state = pkt_buf[0];

            /* Verify checksum */
            uint8_t checksum = 0;
            for (uint8_t i = 2; i <= 22; i++) {
                checksum += pkt_buf[i];
            }

            if (checksum == pkt_buf[23]) {
                /* Valid checksum, parse and update registers */
                hlw_data.state_reg = state;

                hlw_data.voltage_reg[0]  = pkt_buf[2];
                hlw_data.voltage_reg[1]  = pkt_buf[3];
                hlw_data.voltage_reg[2]  = pkt_buf[4];
                hlw_data.voltage_data[0] = pkt_buf[5];
                hlw_data.voltage_data[1] = pkt_buf[6];
                hlw_data.voltage_data[2] = pkt_buf[7];

                hlw_data.current_reg[0]  = pkt_buf[8];
                hlw_data.current_reg[1]  = pkt_buf[9];
                hlw_data.current_reg[2]  = pkt_buf[10];
                hlw_data.current_data[0] = pkt_buf[11];
                hlw_data.current_data[1] = pkt_buf[12];
                hlw_data.current_data[2] = pkt_buf[13];

                hlw_data.power_reg[0]  = pkt_buf[14];
                hlw_data.power_reg[1]  = pkt_buf[15];
                hlw_data.power_reg[2]  = pkt_buf[16];
                hlw_data.power_data[0] = pkt_buf[17];
                hlw_data.power_data[1] = pkt_buf[18];
                hlw_data.power_data[2] = pkt_buf[19];

                hlw_data.data_update = pkt_buf[20];

                hlw_data.pf_count[0] = pkt_buf[21];
                hlw_data.pf_count[1] = pkt_buf[22];

                hlw_data.valid = 1;
                hlw_data.packet_count++;
                last_valid_packet_ms = now;

                /* Go to IDLE mode and set last read timestamp to wait 2 seconds */
                hlw_state    = HLW_STATE_IDLE;
                last_read_ms = now;
            } else {
                /* Checksum invalid: reset to find next header */
                pkt_idx = 0;
            }
        }
    }
}

const hlw8032_data_t *hlw8032_get_data(void)
{
    return &hlw_data;
}

void calculate_energy_parameters(const hlw8032_data_t *edata, uint32_t *voltage_mv, uint32_t *current_ma, uint32_t *power_mw)
{
    if (!edata || !edata->valid) {
        *voltage_mv = 0;
        *current_ma = 0;
        *power_mw = 0;
        return;
    }

    uint32_t v_coeff = ((uint32_t)edata->voltage_reg[0] << 16) | ((uint32_t)edata->voltage_reg[1] << 8) | edata->voltage_reg[2];
    uint32_t v_data  = ((uint32_t)edata->voltage_data[0] << 16) | ((uint32_t)edata->voltage_data[1] << 8) | edata->voltage_data[2];

    uint32_t i_coeff = ((uint32_t)edata->current_reg[0] << 16) | ((uint32_t)edata->current_reg[1] << 8) | edata->current_reg[2];
    uint32_t i_data  = ((uint32_t)edata->current_data[0] << 16) | ((uint32_t)edata->current_data[1] << 8) | edata->current_data[2];

    uint32_t p_coeff = ((uint32_t)edata->power_reg[0] << 16) | ((uint32_t)edata->power_reg[1] << 8) | edata->power_reg[2];
    uint32_t p_data  = ((uint32_t)edata->power_data[0] << 16) | ((uint32_t)edata->power_data[1] << 8) | edata->power_data[2];

    *voltage_mv = (v_data == 0) ? 0 : (uint32_t)(((uint64_t)v_coeff * V_SCALE_FACTOR) / v_data);
    *current_ma = (i_data == 0) ? 0 : (uint32_t)(((uint64_t)i_coeff * I_SCALE_FACTOR) / i_data);
    *power_mw   = (p_data == 0) ? 0 : (uint32_t)(((uint64_t)p_coeff * P_SCALE_FACTOR) / p_data);
}

