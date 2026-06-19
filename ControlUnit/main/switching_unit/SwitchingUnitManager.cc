#include "SwitchingUnitManager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include <inttypes.h>

static const char* TAG = "SwitchingUnitManager";

// Initialize static member
volatile uint64_t SwitchingUnitManager::_lastPulseTimeUs = 0;

SwitchingUnitManager::SwitchingUnitManager() 
    : _waitingForResponse(false)
    , _expectedResponseType(0)
    , _expectedResponseId(0)
    , _uartRxTaskHandle(NULL)
    , _evtCallback(NULL)
    , _evtCallbackArg(NULL)
    , _motorRunning(false)
    , _keepAliveTimer(NULL)
{
    _txMutex = xSemaphoreCreateMutex();
    _transactionSemaphore = xSemaphoreCreateBinary();
}

SwitchingUnitManager::~SwitchingUnitManager() {
    if (_keepAliveTimer) {
        xTimerStop(_keepAliveTimer, portMAX_DELAY);
        xTimerDelete(_keepAliveTimer, portMAX_DELAY);
    }
    if (_uartRxTaskHandle) {
        vTaskDelete(_uartRxTaskHandle);
    }
    gpio_isr_handler_remove(CONTROL_UNIT_PRESENCE_PIN);
    if (_txMutex) {
        vSemaphoreDelete(_txMutex);
    }
    if (_transactionSemaphore) {
        vSemaphoreDelete(_transactionSemaphore);
    }
}

esp_err_t SwitchingUnitManager::init() {
    ESP_LOGI(TAG, "Initializing SwitchingUnitManager (CH32 Interface)...");

    // Configure UART1
    const uart_config_t uartConfiguration = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT, // Standard for newer ESP-IDF (replaces UART_SCLK_APB)
        .flags = {},
    };
    
    esp_err_t err = uart_param_config(CONTROL_UNIT_UART_PORT, &uartConfiguration);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART params: 0x%x", err);
        return err;
    }
    
    err = uart_set_pin(CONTROL_UNIT_UART_PORT, CONTROL_UNIT_UART_TX_PIN, CONTROL_UNIT_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: 0x%x", err);
        return err;
    }
    
    err = uart_driver_install(CONTROL_UNIT_UART_PORT, 256 * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: 0x%x", err);
        return err;
    }
    
    // Configure Presence Pin
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << CONTROL_UNIT_PRESENCE_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // GPIO35 is input-only and lacks internal pull-up on ESP32, but has external pull-up
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure presence GPIO: 0x%x", err);
        return err;
    }
    
    // Install GPIO ISR service (ignore if already installed)
    gpio_install_isr_service(0);
    err = gpio_isr_handler_add(CONTROL_UNIT_PRESENCE_PIN, presenceIsrHandler, (void*)this);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to add presence ISR handler: 0x%x", err);
        return err;
    }
    
    // Start UART Rx Task
    BaseType_t ret = xTaskCreatePinnedToCore(
        uartRxTask,
        "switching_unit_rx_task",
        3072,
        (void*)this,
        configMAX_PRIORITIES - 2,
        &_uartRxTaskHandle,
        APP_CPU_NUM
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART Rx task");
        return ESP_FAIL;
    }
    
    // Create keep-alive timer for UART-based presence fallback.
    // The SwitchingUnit marks CU as present for 2000 ms after each valid UART frame.
    // Sending a CMD_PING every 1000 ms ensures the CU stays visible to the SU.
    _keepAliveTimer = xTimerCreate(
        "su_keepalive",
        pdMS_TO_TICKS(1000),   // 1 second interval
        pdTRUE,                // Auto-reload
        (void*)this,           // Timer ID = this instance
        keepAliveTimerCallback
    );
    if (_keepAliveTimer != NULL) {
        xTimerStart(_keepAliveTimer, portMAX_DELAY);
        ESP_LOGI(TAG, "Keep-alive timer started (1000 ms interval).");
    } else {
        ESP_LOGW(TAG, "Failed to create keep-alive timer — UART presence fallback will not work.");
    }

    ESP_LOGI(TAG, "SwitchingUnitManager initialized successfully.");
    return ESP_OK;
}

esp_err_t SwitchingUnitManager::sendPumpOn(uint16_t* cooldown_remaining_sec_out) {
    if (!isSwitchingUnitPresent()) {
        ESP_LOGW(TAG, "sendPumpOn: Switching unit not present!");
        return ESP_ERR_INVALID_STATE;
    }
    
    switching_proto_frame_t rx_frame;
    esp_err_t err = executeTransaction(PROTO_TYPE_COMMAND, CMD_PUMP_ON, NULL, 0, rx_frame, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sendPumpOn: Transaction failed: 0x%x", err);
        return err;
    }
    
    if (rx_frame.len >= 1) {
        uint8_t status = rx_frame.payload[0];
        if (status == RESP_OK) {
            ESP_LOGI(TAG, "sendPumpOn: Success.");
            _motorRunning = true;
            return ESP_OK;
        } else if (status == RESP_ERR_COOLDOWN) {
            uint16_t cooldown = 0;
            if (rx_frame.len >= 3) {
                cooldown = (rx_frame.payload[1] << 8) | rx_frame.payload[2];
            } else if (rx_frame.len >= 2) {
                cooldown = rx_frame.payload[1];
            }
            ESP_LOGW(TAG, "sendPumpOn: Rejected due to cooldown (%u seconds remaining).", cooldown);
            if (cooldown_remaining_sec_out) {
                *cooldown_remaining_sec_out = cooldown;
            }
            return ESP_ERR_INVALID_STATE;
        } else if (status == RESP_ERR_LOCKED) {
            ESP_LOGW(TAG, "sendPumpOn: Rejected — pump is LOCKED.");
            return ESP_ERR_NOT_ALLOWED;
        } else {
            ESP_LOGE(TAG, "sendPumpOn: Error response: 0x%02X", status);
            return ESP_FAIL;
        }
    }
    
    ESP_LOGE(TAG, "sendPumpOn: Invalid response format");
    return ESP_FAIL;
}

esp_err_t SwitchingUnitManager::sendPumpOff() {
    if (!isSwitchingUnitPresent()) {
        ESP_LOGW(TAG, "sendPumpOff: Switching unit not present!");
        return ESP_ERR_INVALID_STATE;
    }
    
    switching_proto_frame_t rx_frame;
    esp_err_t err = executeTransaction(PROTO_TYPE_COMMAND, CMD_PUMP_OFF, NULL, 0, rx_frame, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sendPumpOff: Transaction failed: 0x%x", err);
        return err;
    }
    
    if (rx_frame.len >= 1) {
        uint8_t status = rx_frame.payload[0];
        if (status == RESP_OK) {
            ESP_LOGI(TAG, "sendPumpOff: Success.");
            _motorRunning = false;
            return ESP_OK;
        } else if (status == RESP_ERR_LOCKED) {
            ESP_LOGW(TAG, "sendPumpOff: Rejected — pump is LOCKED.");
            return ESP_ERR_NOT_ALLOWED;
        } else if (status == RESP_ERR_ALREADY_OFF) {
            ESP_LOGW(TAG, "sendPumpOff: Pump is already OFF.");
            _motorRunning = false;
            return ESP_OK;
        }
    }
    
    ESP_LOGE(TAG, "sendPumpOff: Failed.");
    return ESP_FAIL;
}

esp_err_t SwitchingUnitManager::sendPing() {
    if (!isSwitchingUnitPresent()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    switching_proto_frame_t rx_frame;
    esp_err_t err = executeTransaction(PROTO_TYPE_COMMAND, CMD_PING, NULL, 0, rx_frame, 500);
    if (err != ESP_OK) {
        return err;
    }
    
    if (rx_frame.len >= 1 && rx_frame.payload[0] == RESP_OK) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t SwitchingUnitManager::lockPump() {
    if (!isSwitchingUnitPresent()) {
        ESP_LOGW(TAG, "lockPump: Switching unit not present!");
        return ESP_ERR_INVALID_STATE;
    }
    
    switching_proto_frame_t rx_frame;
    esp_err_t err = executeTransaction(PROTO_TYPE_COMMAND, CMD_PUMP_LOCK, NULL, 0, rx_frame, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lockPump: Transaction failed: 0x%x", err);
        return err;
    }
    
    if (rx_frame.len >= 1 && rx_frame.payload[0] == RESP_OK) {
        ESP_LOGI(TAG, "lockPump: Pump LOCKED successfully.");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "lockPump: Unexpected response: 0x%02X", rx_frame.len >= 1 ? rx_frame.payload[0] : 0xFF);
    return ESP_FAIL;
}

esp_err_t SwitchingUnitManager::unlockPump() {
    if (!isSwitchingUnitPresent()) {
        ESP_LOGW(TAG, "unlockPump: Switching unit not present!");
        return ESP_ERR_INVALID_STATE;
    }
    
    switching_proto_frame_t rx_frame;
    esp_err_t err = executeTransaction(PROTO_TYPE_COMMAND, CMD_PUMP_UNLOCK, NULL, 0, rx_frame, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "unlockPump: Transaction failed: 0x%x", err);
        return err;
    }
    
    if (rx_frame.len >= 1) {
        uint8_t status = rx_frame.payload[0];
        if (status == RESP_OK) {
            ESP_LOGI(TAG, "unlockPump: Pump UNLOCKED successfully.");
            return ESP_OK;
        } else if (status == RESP_ERR_ALREADY_OFF) {
            ESP_LOGW(TAG, "unlockPump: Pump was not locked.");
            return ESP_OK;  // Idempotent — not an error
        } else if (status == RESP_ERR_FAULT) {
            ESP_LOGE(TAG, "unlockPump: Fault condition on SU.");
            return ESP_FAIL;
        }
    }
    
    ESP_LOGE(TAG, "unlockPump: Unexpected response.");
    return ESP_FAIL;
}

esp_err_t SwitchingUnitManager::queryStatus(switching_unit_status_t& status) {
    if (!isSwitchingUnitPresent()) {
        status.presence_detected = false;
        status.cu_present = false;
        status.motor_running = false;
        status.relay_state = 0;
        status.cooldown_remaining_sec = 0;
        return ESP_ERR_INVALID_STATE;
    }
    
    switching_proto_frame_t rx_frame;
    esp_err_t err = executeTransaction(PROTO_TYPE_REQUEST, REQ_GET_STATUS, NULL, 0, rx_frame, 1000);
    if (err != ESP_OK) {
        status.presence_detected = false;
        return err;
    }
    
    if (rx_frame.len >= 2) {
        status.relay_state = rx_frame.payload[0];
        status.motor_running = rx_frame.payload[1] != 0;
        status.presence_detected = true;
        _motorRunning = status.motor_running;
        // SwitchingUnit response layout: [relay_state, motor_running, cu_present, cd_hi, cd_lo]
        // payload[2] is cu_present (UART-based presence fallback)
        status.cu_present = (rx_frame.len >= 3) ? (rx_frame.payload[2] != 0) : false;
        // cooldown is at payload[3..4]
        if (rx_frame.len >= 5) {
            status.cooldown_remaining_sec = (rx_frame.payload[3] << 8) | rx_frame.payload[4];
        } else {
            status.cooldown_remaining_sec = 0;
        }
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t SwitchingUnitManager::queryEnergy(switching_unit_energy_t& energy) {
    if (!isSwitchingUnitPresent()) {
        energy.valid = false;
        return ESP_ERR_INVALID_STATE;
    }
    
    switching_proto_frame_t rx_frame;
    esp_err_t err = executeTransaction(PROTO_TYPE_REQUEST, REQ_GET_ENERGY, NULL, 0, rx_frame, 1000);
    if (err != ESP_OK) {
        energy.valid = false;
        return err;
    }
    
    // Handle error response: 1-byte payload with RESP_ERR_NOT_AVAILABLE
    // This means HLW8032 data is not yet valid or the module is disabled.
    if (rx_frame.len == 1 && rx_frame.payload[0] == RESP_ERR_NOT_AVAILABLE) {
        ESP_LOGW(TAG, "queryEnergy: Energy data not available (HLW8032 not ready).");
        energy.valid = false;
        return ESP_OK;  // Not a failure — data is just not yet available
    }

    if (rx_frame.len >= 12) {
        energy.valid = true;
        energy.voltage_mv = ((uint32_t)rx_frame.payload[0] << 24) | ((uint32_t)rx_frame.payload[1] << 16) | ((uint32_t)rx_frame.payload[2] << 8) | rx_frame.payload[3];
        energy.current_ma = ((uint32_t)rx_frame.payload[4] << 24) | ((uint32_t)rx_frame.payload[5] << 16) | ((uint32_t)rx_frame.payload[6] << 8) | rx_frame.payload[7];
        energy.power_mw   = ((uint32_t)rx_frame.payload[8] << 24) | ((uint32_t)rx_frame.payload[9] << 16) | ((uint32_t)rx_frame.payload[10] << 8) | rx_frame.payload[11];
        return ESP_OK;
    }
    energy.valid = false;
    return ESP_FAIL;
}

esp_err_t SwitchingUnitManager::queryVersion(uint8_t& major, uint8_t& minor, uint8_t& patch) {
    if (!isSwitchingUnitPresent()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    switching_proto_frame_t rx_frame;
    esp_err_t err = executeTransaction(PROTO_TYPE_REQUEST, REQ_GET_VERSION, NULL, 0, rx_frame, 1000);
    if (err != ESP_OK) {
        return err;
    }
    
    if (rx_frame.len >= 3) {
        major = rx_frame.payload[0];
        minor = rx_frame.payload[1];
        patch = rx_frame.payload[2];
        return ESP_OK;
    }
    return ESP_FAIL;
}

bool SwitchingUnitManager::isSwitchingUnitPresent() const {
    if (_lastPulseTimeUs == 0) {
        return false;
    }
    return (esp_timer_get_time() - _lastPulseTimeUs) < 500000ULL; // 500ms timeout
}

bool SwitchingUnitManager::isMotorRunning() const {
    return _motorRunning;
}

void SwitchingUnitManager::registerEventCallback(EventCallback cb, void* arg) {
    _evtCallback = cb;
    _evtCallbackArg = arg;
}

void SwitchingUnitManager::uartRxTask(void* pvParameters) {
    SwitchingUnitManager* manager = static_cast<SwitchingUnitManager*>(pvParameters);
    while (true) {
        manager->handleUartRx();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void SwitchingUnitManager::handleUartRx() {
    uint8_t byte;
    static ParseState state = PARSE_WAIT_SOF;
    static switching_proto_frame_t rx_frame;
    static uint8_t payload_idx = 0;
    static uint8_t calculated_checksum = 0;
    static int64_t frame_start_us = 0;

    int len = uart_read_bytes(CONTROL_UNIT_UART_PORT, &byte, 1, pdMS_TO_TICKS(10));
    if (len <= 0) {
        // Check frame timeout: if mid-frame and no byte for >50 ms, reset parser.
        // This matches the SwitchingUnit's 50 ms inter-byte timeout and prevents
        // the parser from getting stuck on partial/noisy frames.
        if (state != PARSE_WAIT_SOF && frame_start_us > 0) {
            int64_t elapsed = esp_timer_get_time() - frame_start_us;
            if (elapsed > FRAME_TIMEOUT_US) {
                ESP_LOGW(TAG, "Frame timeout (%" PRId64 " us) — resetting parser", elapsed);
                state = PARSE_WAIT_SOF;
                frame_start_us = 0;
            }
        }
        return;
    }

    switch (state) {
        case PARSE_WAIT_SOF:
            if (byte == PROTO_SOF) {
                frame_start_us = esp_timer_get_time();
                state = PARSE_WAIT_TYPE;
            }
            break;
        case PARSE_WAIT_TYPE:
            rx_frame.type = byte;
            calculated_checksum = byte;
            frame_start_us = esp_timer_get_time();
            state = PARSE_WAIT_ID;
            break;
        case PARSE_WAIT_ID:
            rx_frame.id = byte;
            calculated_checksum ^= byte;  // XOR to match SwitchingUnit comm_protocol
            frame_start_us = esp_timer_get_time();
            state = PARSE_WAIT_LEN;
            break;
        case PARSE_WAIT_LEN:
            rx_frame.len = byte;
            calculated_checksum ^= byte;  // XOR to match SwitchingUnit comm_protocol
            frame_start_us = esp_timer_get_time();
            if (rx_frame.len > PROTO_MAX_PAYLOAD) {
                state = PARSE_WAIT_SOF;
                frame_start_us = 0;
            } else if (rx_frame.len == 0) {
                state = PARSE_WAIT_CHECKSUM;
            } else {
                payload_idx = 0;
                state = PARSE_RECV_PAYLOAD;
            }
            break;
        case PARSE_RECV_PAYLOAD:
            rx_frame.payload[payload_idx++] = byte;
            calculated_checksum ^= byte;  // XOR to match SwitchingUnit comm_protocol
            frame_start_us = esp_timer_get_time();
            if (payload_idx >= rx_frame.len) {
                state = PARSE_WAIT_CHECKSUM;
            }
            break;
        case PARSE_WAIT_CHECKSUM:
            if (byte == calculated_checksum) {
                processFrame(rx_frame);
            } else {
                ESP_LOGW(TAG, "Checksum mismatch: expected 0x%02X, calculated 0x%02X", byte, calculated_checksum);
            }
            state = PARSE_WAIT_SOF;
            frame_start_us = 0;
            break;
    }
}

void SwitchingUnitManager::processFrame(const switching_proto_frame_t& frame) {
    if (frame.type == PROTO_TYPE_RESPONSE) {
        if (_waitingForResponse && frame.id == _expectedResponseId) {
            _pendingResponse = frame;
            _waitingForResponse = false;
            xSemaphoreGive(_transactionSemaphore);
        }
    } else if (frame.type == PROTO_TYPE_EVENT) {
        if (frame.id == EVT_PUMP_STARTED) {
            _motorRunning = true;
        } else if (frame.id == EVT_PUMP_STOPPED) {
            _motorRunning = false;
        }
        if (_evtCallback) {
            _evtCallback(frame.id, frame.payload, frame.len, _evtCallbackArg);
        }
    }
}

void SwitchingUnitManager::sendFrame(uint8_t type, uint8_t id, const uint8_t* payload, uint8_t len) {
    uint8_t tx_buf[24];
    tx_buf[0] = PROTO_SOF;
    tx_buf[1] = type;
    tx_buf[2] = id;
    tx_buf[3] = len;
    
    uint8_t checksum = type ^ id ^ len;  // XOR to match SwitchingUnit comm_protocol
    for (int i = 0; i < len; i++) {
        tx_buf[4 + i] = payload[i];
        checksum ^= payload[i];  // XOR to match SwitchingUnit comm_protocol
    }
    tx_buf[4 + len] = checksum;
    
    if (xSemaphoreTake(_txMutex, portMAX_DELAY) == pdTRUE) {
        uart_write_bytes(CONTROL_UNIT_UART_PORT, tx_buf, 5 + len);
        xSemaphoreGive(_txMutex);
    }
}

void IRAM_ATTR SwitchingUnitManager::presenceIsrHandler(void* arg) {
    _lastPulseTimeUs = esp_timer_get_time();
}

esp_err_t SwitchingUnitManager::executeTransaction(uint8_t type, uint8_t id, const uint8_t* tx_payload, uint8_t tx_len, switching_proto_frame_t& rx_frame, uint32_t timeout_ms) {
    _expectedResponseType = PROTO_TYPE_RESPONSE;
    _expectedResponseId = id;
    _waitingForResponse = true;
    
    xSemaphoreTake(_transactionSemaphore, 0); // Clear any pending semaphore
    
    sendFrame(type, id, tx_payload, tx_len);
    
    if (xSemaphoreTake(_transactionSemaphore, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        rx_frame = _pendingResponse;
        return ESP_OK;
    } else {
        _waitingForResponse = false;
        return ESP_ERR_TIMEOUT;
    }
}

void SwitchingUnitManager::keepAliveTimerCallback(TimerHandle_t xTimer) {
    SwitchingUnitManager* manager = static_cast<SwitchingUnitManager*>(pvTimerGetTimerID(xTimer));
    if (manager == NULL) {
        return;
    }
    // Send a lightweight ping frame to maintain UART-based presence on the SU.
    // Even if the SU is not detected via GPIO presence, the ping frame itself
    // will register CU as present on the SU side for 2000 ms.
    // Use sendFrame directly (non-blocking) to avoid blocking the timer daemon task.
    manager->sendFrame(PROTO_TYPE_COMMAND, CMD_PING, NULL, 0);
}
