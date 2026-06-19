/**
 * @file MqttCommandProcessor.h
 * @brief MQTT Command Processor for three-task decoupled architecture
 *
 * This module implements a three-task architecture for MQTT message processing:
 * 1. MQTT Event Handler - Minimal, copies data to queue, returns immediately
 * 2. Command Processor Task - Handles rate limiting, JSON parsing, callback execution
 * 3. Publisher Task - Handles non-blocking MQTT publishing
 *
 * The architecture resolves critical crash-causing bugs and rate limiting issues
 * by decoupling message processing from the MQTT event handler.
 */

#ifndef MQTT_COMMAND_PROCESSOR_H
#define MQTT_COMMAND_PROCESSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "../data/dataTypes.h"
#include "MQTTManager.h"

// ============================================================================
// Buffer Size Constants
// ============================================================================

/** Maximum length for MQTT topic strings */
#define MQTT_CMD_MAX_TOPIC_LEN        128

/** 
 * Fixed payload buffer size for most MQTT responses.
 * Sized to accommodate all response types EXCEPT WiFi scan:
 * - Device Info (67): ~150 bytes
 * - App Info (68): ~250 bytes  
 * - Connection Info (69): ~250 bytes
 * - Water Level (71): ~350 bytes
 * - Pump Metadata (72): ~200 bytes
 * 512 bytes provides ~45% margin for future growth.
 */
#define MQTT_CMD_FIXED_PAYLOAD_LEN    512

/** 
 * Maximum payload length for dynamic allocation (WiFi scan only).
 * WiFi scan with 25 networks can reach ~3000 bytes.
 */
#define MQTT_CMD_MAX_PAYLOAD_LEN      3072

/** Maximum length for MQTT response topic strings */
#define MQTT_CMD_MAX_RESPONSE_TOPIC   128

/** Maximum length for device ID strings */
#define MQTT_CMD_MAX_DEVICE_ID_LEN    32

// ============================================================================
// Queue Configuration
// ============================================================================

/** Depth of the incoming command queue */
#define MQTT_CMD_QUEUE_DEPTH          4

/** Depth of the outgoing response queue */
#define MQTT_RESP_QUEUE_DEPTH         4

// ============================================================================
// Task Configuration
// ============================================================================

/** 
 * Stack size for Command Processor Task (bytes).
 * Reduced from 16384 after implementing hybrid payload allocation.
 * Fixed payload buffer reduced from 2560 to 512 bytes.
 * WiFi scan uses dynamic allocation (pointer transfer).
 * 
 * CORRECTED: Increased to 7680 bytes after safety analysis
 * HWM was 2132 bytes (6060 used), requires 50-100% margin for network/TLS tasks
 * New margin: ~1620 bytes (27% safety margin - meets minimum for network tasks)
 */
#define CMD_PROCESSOR_STACK_SIZE      7680  // Network/TLS task - conservative sizing

/** Priority for Command Processor Task (higher than MQTT task priority 5) */
#define CMD_PROCESSOR_PRIORITY        6

/** 
 * Stack size for Publisher Task (bytes).
 * Reduced from 8192 after implementing hybrid payload allocation.
 * MqttOutgoingResponse structure size reduced by ~516 bytes.
 * 
 * REVERTED: Restored to 4096 bytes after safety analysis
 * HWM was 1156 bytes (2940 used), requires 50-100% margin for MQTT/TLS tasks
 * Margin: ~1156 bytes (39% safety margin - meets minimum for network tasks)
 */
#define PUBLISHER_STACK_SIZE          4096  // MQTT/TLS task - original safe sizing

/** Priority for Publisher Task (lower than MQTT task) */
#define PUBLISHER_PRIORITY            4

/** Core to pin MQTT-related tasks (Core 0 where WiFi driver runs) */
#define MQTT_TASKS_CORE               0

// ============================================================================
// Rate Limiting Configuration
// ============================================================================

/** Token bucket capacity for request rate limiting (burst size) */
#define REQUEST_BUCKET_CAPACITY       8

/** Token refill interval in milliseconds (1 token per 200ms = 5/second sustained) */
#define REQUEST_REFILL_INTERVAL_MS    200

/** Minimum interval between commands in milliseconds */
#define COMMAND_RATE_LIMIT_MS         500

/**
 * Rate limit tolerance in milliseconds
 * 
 * Accounts for timing jitter from network delays, task scheduling, and
 * microsecond-level precision. Commands/requests arriving within this
 * tolerance of the limit boundary will be allowed.
 * 
 * Example: With 500ms limit and 5ms tolerance, commands at 495ms+ are allowed.
 * This prevents false rate limiting when commands arrive at exactly 500ms but
 * are measured as 499.9ms due to timing precision.
 */
#define RATE_LIMIT_TOLERANCE_MS       5

// ============================================================================
// Forward Declarations
// ============================================================================

class MQTT5Manager;

// ============================================================================
// OTA Status Callback Type
// ============================================================================

/**
 * @brief Callback type for checking OTA update status
 *
 * This callback is used to check if an OTA update is in progress.
 * When OTA is running, MQTT commands and requests should be rejected.
 *
 * @return true if OTA update is in progress, false otherwise
 */
typedef bool (*OtaStatusCallback)();

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief Fixed-size structure for incoming MQTT commands/requests
 *
 * This structure is used to transfer parsed MQTT message data from the
 * event handler to the command processor task via a FreeRTOS queue.
 * Uses 4-byte alignment for optimal ESP32 memory access.
 *
 * Total size: ~1304 bytes
 */
typedef struct __attribute__((aligned(4))) {
    /** MQTT topic string (null-terminated) */
    char topic[MQTT_CMD_MAX_TOPIC_LEN];

    /** Message payload (JSON, null-terminated) */
    char payload[MQTT_CMD_MAX_PAYLOAD_LEN];

    /** Response topic for replies (null-terminated) */
    char responseTopic[MQTT_CMD_MAX_RESPONSE_TOPIC];

    /** Actual topic string length (excluding null terminator) */
    uint16_t topicLen;

    /** Actual payload length (excluding null terminator) */
    uint16_t payloadLen;

    /** Actual response topic length (excluding null terminator) */
    uint16_t responseTopicLen;

    /** Padding for alignment */
    uint16_t _reserved;

    /** Topic category (request, command, etc.) */
    MQTTtopicCategories category;

    /** Message source (server, control, etc.) */
    MQTTMessageSourceTypes source;

    /** Timestamp when message was received (microseconds since boot) */
    int64_t receivedTimestampUs;
} MqttIncomingCmd;

/**
 * @brief Fixed-size structure for outgoing MQTT responses
 *
 * This structure is used to transfer response data from the command
 * processor task to the publisher task via a FreeRTOS queue.
 * Uses 4-byte alignment for optimal ESP32 memory access.
 *
 * HYBRID PAYLOAD ALLOCATION:
 * - Most responses use fixedPayload (512 bytes) - sufficient for all
 *   response types except WiFi scan
 * - WiFi scan responses use dynamicPayload (pointer to heap-allocated
 *   buffer) to avoid large stack allocation (~3KB)
 * - usesDynamicPayload flag indicates which buffer to use and whether
 *   cleanup is needed
 *
 * Total size: ~680 bytes (reduced from ~1196 bytes)
 */
typedef struct __attribute__((aligned(4))) {
    /** Response topic string (null-terminated) */
    char topic[MQTT_CMD_MAX_RESPONSE_TOPIC];

    /** 
     * Fixed payload buffer for most responses (JSON, null-terminated).
     * Used for: Device Info, App Info, Connection Info, Water Level, Pump Metadata
     */
    char fixedPayload[MQTT_CMD_FIXED_PAYLOAD_LEN];

    /**
     * Dynamic payload pointer for large responses (WiFi scan only).
     * When usesDynamicPayload is true, this points to heap-allocated memory
     * that must be freed by the publisher task after use.
     */
    char* dynamicPayload;

    /** Device ID for the response */
    char deviceId[MQTT_CMD_MAX_DEVICE_ID_LEN];

    /** Actual topic string length (excluding null terminator) */
    uint16_t topicLen;

    /** Actual payload length (excluding null terminator) */
    uint16_t payloadLen;

    /** Quality of Service level for the response */
    QualityOfService qos;

    /** Whether to set the retain flag on the response */
    bool retain;

    /**
     * Flag indicating payload allocation type:
     * - false: Use fixedPayload buffer (no cleanup needed)
     * - true: Use dynamicPayload pointer (must free after use)
     */
    bool usesDynamicPayload;

    /** Padding for alignment */
    uint8_t _reserved[2];
} MqttOutgoingResponse;

/**
 * @brief Token bucket structure for request rate limiting
 *
 * Implements a token bucket algorithm that allows bursts of up to
 * REQUEST_BUCKET_CAPACITY requests while limiting the sustained rate
 * to 1 request per REQUEST_REFILL_INTERVAL_MS.
 */
typedef struct {
    /** Current number of tokens available (0 to REQUEST_BUCKET_CAPACITY) */
    uint8_t tokens;

    /** Padding for alignment */
    uint8_t _reserved[7];

    /** Timestamp of last token refill (microseconds since boot) */
    int64_t lastRefillTimeUs;
} TokenBucket;

// ============================================================================
// MqttCommandProcessor Namespace
// ============================================================================

/**
 * @brief MQTT Command Processor module
 *
 * This namespace provides the interface for the three-task MQTT architecture.
 * It manages the command queue, response queue, command processor task,
 * and publisher task.
 */
namespace MqttCommandProcessor {

    /**
     * @brief Initialize the MQTT Command Processor
     *
     * Creates the command and response queues, and starts the command
     * processor and publisher tasks. Must be called after MQTT connection
     * is established.
     *
     * @param mqttManager Pointer to the MQTT5Manager instance
     * @return ESP_OK on success, error code on failure
     */
    esp_err_t init(MQTT5Manager* mqttManager);

    /**
     * @brief Set the OTA status callback
     *
     * Sets a callback function that will be called to check if an OTA update
     * is in progress. When OTA is running, all incoming MQTT commands and
     * requests will be rejected.
     *
     * @param callback Function pointer that returns true if OTA is in progress
     */
    void setOtaStatusCallback(OtaStatusCallback callback);

    /**
     * @brief Queue an incoming command from the event handler
     *
     * Called by the MQTT event handler to queue a received message for
     * processing. This function copies the command data to the queue
     * and returns immediately (non-blocking with short timeout).
     *
     * @param cmd Pointer to the incoming command structure
     * @return true if command was queued successfully, false if queue full
     */
    bool queueIncomingCommand(const MqttIncomingCmd* cmd);

    /**
     * @brief Queue an outgoing response from the command processor
     *
     * Called by the command processor task to queue a response for
     * publishing. Uses a longer timeout than queueIncomingCommand.
     *
     * @param response Pointer to the outgoing response structure
     * @return true if response was queued successfully, false if queue full
     */
    bool queueOutgoingResponse(const MqttOutgoingResponse* response);

    /**
     * @brief Get the number of free slots in the command queue
     *
     * Useful for monitoring queue utilization.
     *
     * @return Number of free slots in the command queue
     */
    uint32_t getCmdQueueFreeSlots();

    /**
     * @brief Get the number of free slots in the response queue
     *
     * Useful for monitoring queue utilization.
     *
     * @return Number of free slots in the response queue
     */
    uint32_t getRespQueueFreeSlots();

    /**
     * @brief Shutdown the MQTT Command Processor
     *
     * Stops the command processor and publisher tasks, and deletes
     * the queues. Used for cleanup during testing or system shutdown.
     */
    void shutdown();

} // namespace MqttCommandProcessor

#endif // MQTT_COMMAND_PROCESSOR_H
