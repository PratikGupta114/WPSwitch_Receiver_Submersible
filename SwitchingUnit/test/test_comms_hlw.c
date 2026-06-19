#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdint.h>
#include "comm_protocol.h"
#include "hlw8032.h"
#include "relay_ctrl.h"

/* --- Global Mock State --- */
static uint32_t mock_ms = 0;
static int16_t mock_hw_rx_data[256];
static uint8_t mock_hw_rx_len = 0;
static uint8_t mock_hw_rx_idx = 0;

static uint8_t mock_hw_tx_data[256];
static uint8_t mock_hw_tx_len = 0;

static int16_t mock_sw_rx_data[256];
static uint8_t mock_sw_rx_len = 0;
static uint8_t mock_sw_rx_idx = 0;

static relay_state_t mock_relay_state = RELAY_STATE_IDLE;
static uint8_t mock_motor_running = 0;
static uint8_t mock_presence = 1;

/* --- Mock Implementations --- */
uint32_t millis(void) {
    return mock_ms;
}

int16_t hw_uart_read_byte(void) {
    if (mock_hw_rx_idx >= mock_hw_rx_len) {
        return -1;
    }
    return mock_hw_rx_data[mock_hw_rx_idx++];
}

void hw_uart_send_byte(uint8_t b) {
    assert(mock_hw_tx_len < 256);
    mock_hw_tx_data[mock_hw_tx_len++] = b;
}

int16_t sw_uart_rx_byte(void) {
    if (mock_sw_rx_idx >= mock_sw_rx_len) {
        return -1;
    }
    return mock_sw_rx_data[mock_sw_rx_idx++];
}

void sw_uart_init(void) {
    /* Mock sw_uart_init */
}

relay_result_t relay_pump_on(void) {
    if (mock_relay_state == RELAY_STATE_COOLDOWN) {
        return RELAY_ERR_COOLDOWN;
    }
    if (mock_relay_state == RELAY_STATE_LOCKED) {
        return RELAY_ERR_LOCKED;
    }
    mock_relay_state = RELAY_STATE_STARTING;
    return RELAY_OK;
}

relay_result_t relay_pump_off(void) {
    if (mock_relay_state == RELAY_STATE_LOCKED) {
        return RELAY_ERR_LOCKED;
    }
    mock_relay_state = RELAY_STATE_STOPPING;
    return RELAY_OK;
}

relay_result_t relay_pump_lock(void) {
    mock_relay_state = RELAY_STATE_LOCKED;
    return RELAY_OK;
}

relay_result_t relay_pump_unlock(void) {
    if (mock_relay_state != RELAY_STATE_LOCKED) {
        return RELAY_ERR_ALREADY_OFF;
    }
    mock_relay_state = RELAY_STATE_IDLE;
    return RELAY_OK;
}

relay_state_t relay_get_state(void) {
    return mock_relay_state;
}

uint16_t relay_get_cooldown_remaining(void) {
    if (mock_relay_state == RELAY_STATE_COOLDOWN) {
        return 170;
    }
    return 0;
}

uint8_t motor_is_running(void) {
    return mock_motor_running;
}

uint8_t presence_is_control_unit_present(void) {
    return mock_presence;
}

static uint32_t mock_presence_uart_ms = 0;
void presence_record_uart_active(void) {
    mock_presence_uart_ms = mock_ms;
}

/* Helper to feed bytes into hardware UART mock */
void feed_hw_uart(const uint8_t *data, uint8_t len) {
    mock_hw_rx_len = len;
    mock_hw_rx_idx = 0;
    for (uint8_t i = 0; i < len; i++) {
        mock_hw_rx_data[i] = data[i];
    }
}

/* Helper to feed bytes into software UART mock */
void feed_sw_uart(const uint8_t *data, uint8_t len) {
    mock_sw_rx_len = len;
    mock_sw_rx_idx = 0;
    for (uint8_t i = 0; i < len; i++) {
        mock_sw_rx_data[i] = data[i];
    }
}

void reset_tx(void) {
    mock_hw_tx_len = 0;
}

int main(void) {
    printf("==================================================\n");
    printf("   Running Host Unit Tests for Switching Unit\n");
    printf("==================================================\n\n");

    /* =======================================================================
     * Test Case 1: Valid CMD_PING Packet
     * =====================================================================*/
    printf("[Test 1] Valid PING command... ");
    comm_protocol_init();
    reset_tx();
    mock_presence_uart_ms = 999;
    mock_ms = 456;
    
    /* Frame: SOF (0xAA) + Type (0x01) + ID (CMD_PING=0x05) + Len (0x00) + Checksum (0x01 ^ 0x05 ^ 0x00 = 0x04) */
    uint8_t ping_cmd[] = {0xAA, 0x01, 0x05, 0x00, 0x04};
    feed_hw_uart(ping_cmd, sizeof(ping_cmd));
    comm_protocol_tick();
    
    /* Verify response was sent: SOF (0xAA) + Type (RESP=0x02) + ID (CMD_PING=0x05) + Len (0x01) + Payload (RESP_OK=0x00) + Checksum (0x02^0x05^0x01^0x00 = 0x06) */
    assert(mock_hw_tx_len == 6);
    assert(mock_hw_tx_data[0] == 0xAA);
    assert(mock_hw_tx_data[1] == 0x02);
    assert(mock_hw_tx_data[2] == 0x05);
    assert(mock_hw_tx_data[3] == 0x01);
    assert(mock_hw_tx_data[4] == 0x00);
    assert(mock_hw_tx_data[5] == 0x06);
    assert(mock_presence_uart_ms == 456);
    printf("PASSED\n");

    /* =======================================================================
     * Test Case 2: Corrupted Packet Checksum
     * =====================================================================*/
    printf("[Test 2] Corrupted PING checksum... ");
    comm_protocol_init();
    reset_tx();
    mock_presence_uart_ms = 999;
    mock_ms = 456;
    
    /* Frame with wrong checksum (0x05 instead of 0x04) */
    uint8_t bad_ping[] = {0xAA, 0x01, 0x05, 0x00, 0x05};
    feed_hw_uart(bad_ping, sizeof(bad_ping));
    comm_protocol_tick();
    
    /* Verify NO response was sent */
    assert(mock_hw_tx_len == 0);
    assert(mock_presence_uart_ms == 999);
    printf("PASSED\n");

    /* =======================================================================
     * Test Case 3: Protocol Frame Timeout
     * =====================================================================*/
    printf("[Test 3] Protocol timeout... ");
    comm_protocol_init();
    reset_tx();
    
    /* Send part of frame, advance clock by 55 ms, send the rest */
    uint8_t part1[] = {0xAA, 0x01, 0x05};
    feed_hw_uart(part1, sizeof(part1));
    mock_ms = 0;
    comm_protocol_tick();
    
    mock_ms = 55; /* Timeout is 50 ms */
    
    uint8_t part2[] = {0x00, 0x04};
    feed_hw_uart(part2, sizeof(part2));
    comm_protocol_tick();
    
    /* Verify NO response because first frame was timed out/discarded */
    assert(mock_hw_tx_len == 0);
    printf("PASSED\n");

#if FEATURE_HLW8032
    /* =======================================================================
     * Test Case 4: Valid HLW8032 Packet Parsing
     * =====================================================================*/
    printf("[Test 4] Valid HLW8032 packet... ");
    hlw8032_init();
    
    /* Create a valid 24-byte packet */
    uint8_t hlw_packet[24];
    memset(hlw_packet, 0, sizeof(hlw_packet));
    hlw_packet[0] = 0xAA; /* State: normal */
    hlw_packet[1] = 0x5A; /* Check: always 0x5A */
    
    /* Voltage parameters [2..4] and data [5..7] */
    hlw_packet[2] = 0x11; hlw_packet[3] = 0x22; hlw_packet[4] = 0x33;
    hlw_packet[5] = 0x44; hlw_packet[6] = 0x55; hlw_packet[7] = 0x66;
    
    /* Checksum: sum of [2..22] mod 256 */
    uint8_t sum = 0;
    for (uint8_t i = 2; i <= 22; i++) {
        sum += hlw_packet[i];
    }
    hlw_packet[23] = sum;
    
    /* Advance time to trigger reading (we rate limit to once every 2 seconds) */
    mock_ms = 2100;
    
    feed_sw_uart(hlw_packet, 24);
    
    /* Tick 30 times to ingest 24 bytes plus the initial tick */
    for (int i = 0; i < 30; i++) {
        hlw8032_tick();
    }
    
    const hlw8032_data_t *edata = hlw8032_get_data();
    assert(edata->valid == 1);
    assert(edata->state_reg == 0xAA);
    assert(edata->voltage_reg[0] == 0x11);
    assert(edata->voltage_reg[1] == 0x22);
    assert(edata->voltage_reg[2] == 0x33);
    assert(edata->voltage_data[0] == 0x44);
    assert(edata->voltage_data[1] == 0x55);
    assert(edata->voltage_data[2] == 0x66);
    assert(edata->packet_count == 1);
    printf("PASSED\n");

    /* =======================================================================
     * Test Case 5: Invalid HLW8032 Checksum
     * =====================================================================*/
    printf("[Test 5] Invalid HLW8032 checksum... ");
    hlw8032_init();
    
    hlw_packet[23] = sum + 1; /* Corrupt checksum */
    mock_ms = 5000; /* Trigger next read after 2.9 seconds */
    
    feed_sw_uart(hlw_packet, 24);
    for (int i = 0; i < 30; i++) {
        hlw8032_tick();
    }
    
    /* Packet count should still be 0 (init reset the static struct) */
    edata = hlw8032_get_data();
    assert(edata->valid == 0);
    assert(edata->packet_count == 0);
    printf("PASSED\n");
#else
    /* =======================================================================
     * Test Case 6: REQ_GET_ENERGY when disabled
     * =====================================================================*/
    printf("[Test 6] REQ_GET_ENERGY returns NOT_AVAILABLE when disabled... ");
    comm_protocol_init();
    reset_tx();

    /* Frame: SOF (0xAA) + Type (REQ=0x04) + ID (REQ_GET_ENERGY=0x04) + Len (0x00) + Checksum (0x04 ^ 0x04 ^ 0x00 = 0x00) */
    uint8_t get_energy_cmd[] = {0xAA, 0x04, REQ_GET_ENERGY, 0x00, 0x00};
    feed_hw_uart(get_energy_cmd, sizeof(get_energy_cmd));
    comm_protocol_tick();

    /* Verify response: SOF (0xAA) + Type (RESP=0x02) + ID (REQ_GET_ENERGY=0x04) + Len (0x01) + Payload (RESP_ERR_NOT_AVAILABLE=0x06) + Checksum (0x02^0x04^0x01^0x06 = 0x01) */
    assert(mock_hw_tx_len == 6);
    assert(mock_hw_tx_data[0] == 0xAA);
    assert(mock_hw_tx_data[1] == 0x02);
    assert(mock_hw_tx_data[2] == REQ_GET_ENERGY);
    assert(mock_hw_tx_data[3] == 0x01);
    assert(mock_hw_tx_data[4] == RESP_ERR_NOT_AVAILABLE);
    assert(mock_hw_tx_data[5] == 0x01);
    printf("PASSED\n");
#endif

    /* =======================================================================
     * Test Case 7: REQ_GET_STATUS
     * =====================================================================*/
    printf("[Test 7] REQ_GET_STATUS query... ");
    comm_protocol_init();
    reset_tx();
    
    mock_relay_state = RELAY_STATE_COOLDOWN;
    mock_motor_running = 0;
    mock_presence = 1;

    /* Frame: SOF (0xAA) + Type (REQ=0x04) + ID (REQ_GET_STATUS=0x03) + Len (0x00) + Checksum (0x04 ^ 0x03 ^ 0x00 = 0x07) */
    uint8_t get_status_req[] = {0xAA, 0x04, REQ_GET_STATUS, 0x00, 0x07};
    feed_hw_uart(get_status_req, sizeof(get_status_req));
    comm_protocol_tick();

    /* Verify response: 5 bytes payload (no RESP_OK) 
     * Payload: [state=4, motor=0, presence=1, cd_high=0, cd_low=170(0xAA)]
     * Checksum: 0x02 ^ 0x03 ^ 0x05 ^ 0x04 ^ 0x00 ^ 0x01 ^ 0x00 ^ 0xAA = 0xAB
     */
    assert(mock_hw_tx_len == 10);
    assert(mock_hw_tx_data[0] == 0xAA);
    assert(mock_hw_tx_data[1] == 0x02);
    assert(mock_hw_tx_data[2] == REQ_GET_STATUS);
    assert(mock_hw_tx_data[3] == 0x05);
    assert(mock_hw_tx_data[4] == 0x04);
    assert(mock_hw_tx_data[5] == 0x00);
    assert(mock_hw_tx_data[6] == 0x01);
    assert(mock_hw_tx_data[7] == 0x00);
    assert(mock_hw_tx_data[8] == 0xAA);
    assert(mock_hw_tx_data[9] == 0xAB);
    printf("PASSED\n");

    /* =======================================================================
     * Test Case 8: REQ_GET_VERSION
     * =====================================================================*/
    printf("[Test 8] REQ_GET_VERSION query... ");
    comm_protocol_init();
    reset_tx();

    /* Frame: SOF (0xAA) + Type (REQ=0x04) + ID (REQ_GET_VERSION=0x06) + Len (0x00) + Checksum (0x04 ^ 0x06 ^ 0x00 = 0x02) */
    uint8_t get_ver_req[] = {0xAA, 0x04, REQ_GET_VERSION, 0x00, 0x02};
    feed_hw_uart(get_ver_req, sizeof(get_ver_req));
    comm_protocol_tick();

    /* Verify response: 3 bytes payload (no RESP_OK)
     * Payload: [major=0, minor=1, patch=0]
     * Checksum: 0x02 ^ 0x06 ^ 0x03 ^ 0x00 ^ 0x01 ^ 0x00 = 0x06
     */
    assert(mock_hw_tx_len == 8);
    assert(mock_hw_tx_data[0] == 0xAA);
    assert(mock_hw_tx_data[1] == 0x02);
    assert(mock_hw_tx_data[2] == REQ_GET_VERSION);
    assert(mock_hw_tx_data[3] == 0x03);
    assert(mock_hw_tx_data[4] == 0x00);
    assert(mock_hw_tx_data[5] == 0x01);
    assert(mock_hw_tx_data[6] == 0x00);
    assert(mock_hw_tx_data[7] == 0x06);
    printf("PASSED\n");

#if FEATURE_HLW8032
    /* =======================================================================
     * Test Case 9: REQ_GET_ENERGY when enabled
     * =====================================================================*/
    printf("[Test 9] REQ_GET_ENERGY query... ");
    hlw8032_init();
    comm_protocol_init();
    reset_tx();

    /* Feed a valid 24-byte packet to set up valid mock data */
    uint8_t hlw_valid_packet[24];
    memset(hlw_valid_packet, 0, sizeof(hlw_valid_packet));
    hlw_valid_packet[0] = 0xAA;
    hlw_valid_packet[1] = 0x5A;
    hlw_valid_packet[2] = 0x11; hlw_valid_packet[3] = 0x22; hlw_valid_packet[4] = 0x33;
    hlw_valid_packet[5] = 0x44; hlw_valid_packet[6] = 0x55; hlw_valid_packet[7] = 0x66;
    uint8_t cs = 0;
    for (uint8_t i = 2; i <= 22; i++) {
        cs += hlw_valid_packet[i];
    }
    hlw_valid_packet[23] = cs;

    mock_ms = 8000; // Trigger reading
    feed_sw_uart(hlw_valid_packet, 24);
    for (int i = 0; i < 30; i++) {
        hlw8032_tick();
    }

    uint8_t get_energy_req[] = {0xAA, 0x04, REQ_GET_ENERGY, 0x00, 0x00};
    feed_hw_uart(get_energy_req, sizeof(get_energy_req));
    comm_protocol_tick();

    assert(mock_hw_tx_len == 17);
    assert(mock_hw_tx_data[0] == 0xAA);
    assert(mock_hw_tx_data[1] == 0x02);
    assert(mock_hw_tx_data[2] == REQ_GET_ENERGY);
    assert(mock_hw_tx_data[3] == 0x0C);
    
    // Voltage in millivolts = 1886 = 0x0000075E
    assert(mock_hw_tx_data[4] == 0x00);
    assert(mock_hw_tx_data[5] == 0x00);
    assert(mock_hw_tx_data[6] == 0x07);
    assert(mock_hw_tx_data[7] == 0x5E);
    
    // Current in milliamps = 0 = 0x00000000
    assert(mock_hw_tx_data[8] == 0x00);
    assert(mock_hw_tx_data[9] == 0x00);
    assert(mock_hw_tx_data[10] == 0x00);
    assert(mock_hw_tx_data[11] == 0x00);
    
    // Power in milliwatts = 0 = 0x00000000
    assert(mock_hw_tx_data[12] == 0x00);
    assert(mock_hw_tx_data[13] == 0x00);
    assert(mock_hw_tx_data[14] == 0x00);
    assert(mock_hw_tx_data[15] == 0x00);
    
    // Checksum = 0x02 ^ 0x04 ^ 0x0C ^ 0x07 ^ 0x5E = 0x53
    assert(mock_hw_tx_data[16] == 0x53);
    printf("PASSED\n");
#endif

    /* =======================================================================
     * Test Case 10: CMD_PUMP_LOCK
     * =====================================================================*/
    printf("[Test 10] CMD_PUMP_LOCK command... ");
    comm_protocol_init();
    reset_tx();
    mock_relay_state = RELAY_STATE_IDLE;
    mock_ms = 100;

    /* Frame: SOF (0xAA) + Type (CMD=0x01) + ID (CMD_PUMP_LOCK=0x07) + Len (0x00) + Checksum (0x01 ^ 0x07 ^ 0x00 = 0x06) */
    uint8_t lock_cmd[] = {0xAA, 0x01, CMD_PUMP_LOCK, 0x00, 0x06};
    feed_hw_uart(lock_cmd, sizeof(lock_cmd));
    comm_protocol_tick();

    /* Verify response: RESP_OK */
    assert(mock_hw_tx_len == 6);
    assert(mock_hw_tx_data[0] == 0xAA);
    assert(mock_hw_tx_data[1] == 0x02);
    assert(mock_hw_tx_data[2] == CMD_PUMP_LOCK);
    assert(mock_hw_tx_data[3] == 0x01);
    assert(mock_hw_tx_data[4] == RESP_OK);
    assert(mock_relay_state == RELAY_STATE_LOCKED);
    printf("PASSED\n");

    /* =======================================================================
     * Test Case 11: CMD_PUMP_ON while LOCKED
     * =====================================================================*/
    printf("[Test 11] CMD_PUMP_ON while locked... ");
    comm_protocol_init();
    reset_tx();
    /* mock_relay_state is still LOCKED from Test 10 */

    /* Frame: SOF (0xAA) + Type (CMD=0x01) + ID (CMD_PUMP_ON=0x01) + Len (0x00) + Checksum (0x01 ^ 0x01 ^ 0x00 = 0x00) */
    uint8_t pump_on_locked[] = {0xAA, 0x01, CMD_PUMP_ON, 0x00, 0x00};
    feed_hw_uart(pump_on_locked, sizeof(pump_on_locked));
    comm_protocol_tick();

    /* Verify response: RESP_ERR_LOCKED */
    assert(mock_hw_tx_len == 6);
    assert(mock_hw_tx_data[0] == 0xAA);
    assert(mock_hw_tx_data[1] == 0x02);
    assert(mock_hw_tx_data[2] == CMD_PUMP_ON);
    assert(mock_hw_tx_data[3] == 0x01);
    assert(mock_hw_tx_data[4] == RESP_ERR_LOCKED);
    assert(mock_relay_state == RELAY_STATE_LOCKED);
    printf("PASSED\n");

    /* =======================================================================
     * Test Case 12: CMD_PUMP_OFF while LOCKED
     * =====================================================================*/
    printf("[Test 12] CMD_PUMP_OFF while locked... ");
    comm_protocol_init();
    reset_tx();
    /* mock_relay_state is still LOCKED */

    /* Frame: SOF (0xAA) + Type (CMD=0x01) + ID (CMD_PUMP_OFF=0x02) + Len (0x00) + Checksum (0x01 ^ 0x02 ^ 0x00 = 0x03) */
    uint8_t pump_off_locked[] = {0xAA, 0x01, CMD_PUMP_OFF, 0x00, 0x03};
    feed_hw_uart(pump_off_locked, sizeof(pump_off_locked));
    comm_protocol_tick();

    /* Verify response: RESP_ERR_LOCKED */
    assert(mock_hw_tx_len == 6);
    assert(mock_hw_tx_data[0] == 0xAA);
    assert(mock_hw_tx_data[1] == 0x02);
    assert(mock_hw_tx_data[2] == CMD_PUMP_OFF);
    assert(mock_hw_tx_data[3] == 0x01);
    assert(mock_hw_tx_data[4] == RESP_ERR_LOCKED);
    assert(mock_relay_state == RELAY_STATE_LOCKED);
    printf("PASSED\n");

    /* =======================================================================
     * Test Case 13: CMD_PUMP_UNLOCK
     * =====================================================================*/
    printf("[Test 13] CMD_PUMP_UNLOCK command... ");
    comm_protocol_init();
    reset_tx();
    /* mock_relay_state is still LOCKED */

    /* Frame: SOF (0xAA) + Type (CMD=0x01) + ID (CMD_PUMP_UNLOCK=0x08) + Len (0x00) + Checksum (0x01 ^ 0x08 ^ 0x00 = 0x09) */
    uint8_t unlock_cmd[] = {0xAA, 0x01, CMD_PUMP_UNLOCK, 0x00, 0x09};
    feed_hw_uart(unlock_cmd, sizeof(unlock_cmd));
    comm_protocol_tick();

    /* Verify response: RESP_OK */
    assert(mock_hw_tx_len == 6);
    assert(mock_hw_tx_data[0] == 0xAA);
    assert(mock_hw_tx_data[1] == 0x02);
    assert(mock_hw_tx_data[2] == CMD_PUMP_UNLOCK);
    assert(mock_hw_tx_data[3] == 0x01);
    assert(mock_hw_tx_data[4] == RESP_OK);
    assert(mock_relay_state == RELAY_STATE_IDLE);
    printf("PASSED\n");

    printf("\n==================================================\n");
    printf("   All tests completed successfully! \xF0\x9F\x8E\x89\n");
    printf("==================================================\n");
    return 0;
}
