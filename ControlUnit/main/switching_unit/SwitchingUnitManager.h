#ifndef SWITCHING_UNIT_MANAGER_H_
#define SWITCHING_UNIT_MANAGER_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "board_pins.h"

// Frame Protocol Constants (binary, compact, checksummed)
#define PROTO_SOF               0xAA
#define PROTO_MAX_PAYLOAD       16

/* --- Message Types --- */
#define PROTO_TYPE_COMMAND      0x01
#define PROTO_TYPE_RESPONSE     0x02
#define PROTO_TYPE_EVENT        0x03
#define PROTO_TYPE_REQUEST      0x04

/* --- Command & Request IDs --- */
#define CMD_PUMP_ON             0x01
#define CMD_PUMP_OFF            0x02
#define REQ_GET_STATUS          0x03
#define REQ_GET_ENERGY          0x04
#define CMD_PING                0x05
#define REQ_GET_VERSION         0x06
#define CMD_PUMP_LOCK           0x07
#define CMD_PUMP_UNLOCK         0x08

/* --- Response Status Codes --- */
#define RESP_OK                 0x00
#define RESP_ERR_COOLDOWN       0x01
#define RESP_ERR_ALREADY_ON     0x02
#define RESP_ERR_ALREADY_OFF    0x03
#define RESP_ERR_FAULT          0x04
#define RESP_ERR_UNKNOWN_CMD    0x05
#define RESP_ERR_NOT_AVAILABLE  0x06
#define RESP_ERR_LOCKED         0x07

/* --- Relay States --- */
#define RELAY_STATE_LOCKED      5

/* --- Event IDs --- */
#define EVT_PUMP_STARTED        0x01
#define EVT_PUMP_STOPPED        0x02
#define EVT_HIGH_ENERGY         0x03
#define EVT_COOLDOWN_STARTED    0x04
#define EVT_COOLDOWN_ENDED      0x05
#define EVT_FAULT               0x06
#define EVT_PRESENCE_CHANGE     0x07

// Struct for Parsed UART Frames
typedef struct {
    uint8_t type;
    uint8_t id;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} switching_proto_frame_t;

// Switching Unit State Struct
typedef struct {
    uint8_t relay_state;
    bool motor_running;
    bool presence_detected;
    bool cu_present;           // SU sees CU as present (UART-based fallback)
    uint16_t cooldown_remaining_sec;
} switching_unit_status_t;

// HLW8032 Energy Data Struct
typedef struct {
    bool valid;
    uint32_t voltage_mv;
    uint32_t current_ma;
    uint32_t power_mw;
} switching_unit_energy_t;

class SwitchingUnitManager {
public:
    // Event callback signatures
    typedef void (*EventCallback)(uint8_t event_id, const uint8_t* payload, uint8_t len, void* arg);

    SwitchingUnitManager();
    ~SwitchingUnitManager();

    esp_err_t init();
    
    // Pump Controls (Blocking API with response validation)
    esp_err_t sendPumpOn(uint16_t* cooldown_remaining_sec_out = nullptr);
    esp_err_t sendPumpOff();
    esp_err_t sendPing();
    esp_err_t lockPump();
    esp_err_t unlockPump();

    // Queries
    esp_err_t queryStatus(switching_unit_status_t& status);
    esp_err_t queryEnergy(switching_unit_energy_t& energy);
    esp_err_t queryVersion(uint8_t& major, uint8_t& minor, uint8_t& patch);

    // Presence / Heartbeat status
    bool isSwitchingUnitPresent() const;
    bool isMotorRunning() const;

    // Event Registration
    void registerEventCallback(EventCallback cb, void* arg);

private:
    enum ParseState {
        PARSE_WAIT_SOF,
        PARSE_WAIT_TYPE,
        PARSE_WAIT_ID,
        PARSE_WAIT_LEN,
        PARSE_RECV_PAYLOAD,
        PARSE_WAIT_CHECKSUM
    };

    // UART Communication Task
    static void uartRxTask(void* pvParameters);
    void handleUartRx();
    void processFrame(const switching_proto_frame_t& frame);
    void sendFrame(uint8_t type, uint8_t id, const uint8_t* payload, uint8_t len);

    // Presence ISR and Timer
    static void IRAM_ATTR presenceIsrHandler(void* arg);

    // Synchronous Command handling
    esp_err_t executeTransaction(uint8_t type, uint8_t id, const uint8_t* tx_payload, uint8_t tx_len, switching_proto_frame_t& rx_frame, uint32_t timeout_ms);

    // Synchronization Mutexes & Semaphores
    SemaphoreHandle_t _txMutex;
    SemaphoreHandle_t _transactionSemaphore;
    switching_proto_frame_t _pendingResponse;
    bool _waitingForResponse;
    uint8_t _expectedResponseType;
    uint8_t _expectedResponseId;

    // Callbacks & Task Handles
    TaskHandle_t _uartRxTaskHandle;
    EventCallback _evtCallback;
    void* _evtCallbackArg;

    // Hardware State variables
    static volatile uint64_t _lastPulseTimeUs;
    volatile bool _motorRunning;

    // Frame timeout tracking (50 ms inter-byte timeout, matching SwitchingUnit)
    static constexpr int64_t FRAME_TIMEOUT_US = 50000;

    // Keep-alive timer for UART-based presence fallback
    TimerHandle_t _keepAliveTimer;
    static void keepAliveTimerCallback(TimerHandle_t xTimer);
};

#endif // SWITCHING_UNIT_MANAGER_H_
