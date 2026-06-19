/**
 * @file MqttCommandProcessor.cc
 * @brief MQTT Command Processor implementation for three-task decoupled architecture
 *
 * This module implements:
 * - Token bucket rate limiting for MQTT requests (burst of 4, 4/sec sustained)
 * - Strict interval rate limiting for MQTT commands (2 second minimum)
 * - Static queue allocation for command and response queues
 * - Command processor and publisher tasks
 *
 * The architecture resolves critical crash-causing bugs and rate limiting issues
 * by decoupling message processing from the MQTT event handler.
 */

#include "MqttCommandProcessor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"
#include <string.h>
#include <algorithm>

static const char *TAG = "MqttCmdProc";

// ============================================================================
// Rate Limiting State (accessed only from Command Processor Task)
// ============================================================================

/**
 * @brief Token bucket for request rate limiting
 *
 * Initialized with full capacity. Uses lazy initialization for lastRefillTimeUs
 * to ensure accurate timing from first request.
 */
static TokenBucket s_requestBucket = {
    .tokens = REQUEST_BUCKET_CAPACITY,
    ._reserved = {0},
    .lastRefillTimeUs = 0
};

/**
 * @brief Timestamp of last processed command (microseconds since boot)
 *
 * Value of 0 indicates no command has been processed yet (first command allowed).
 */
static int64_t s_lastCommandTimeUs = 0;

/**
 * @brief Flag to track if rate limiting has been initialized
 */
static bool s_rateLimitInitialized = false;

// ============================================================================
// Rate Limiting Constants (in microseconds for internal calculations)
// ============================================================================

/** Token refill interval in microseconds */
static const int64_t REQUEST_REFILL_INTERVAL_US = REQUEST_REFILL_INTERVAL_MS * 1000LL;

/** Command rate limit interval in microseconds */
static const int64_t COMMAND_RATE_LIMIT_US = COMMAND_RATE_LIMIT_MS * 1000LL;

/** Rate limit tolerance in microseconds (accounts for timing jitter) */
static const int64_t RATE_LIMIT_TOLERANCE_US = RATE_LIMIT_TOLERANCE_MS * 1000LL;

// ============================================================================
// Static Queue Storage (Requirements 4.1, 4.2, 4.3)
// ============================================================================

/**
 * @brief Static storage for command queue items
 *
 * Statically allocated buffer for 8 MqttIncomingCmd structures.
 * Using static allocation via xQueueCreateStatic() for reliability.
 */
static uint8_t s_cmdQueueStorage[MQTT_CMD_QUEUE_DEPTH * sizeof(MqttIncomingCmd)] __attribute__((aligned(4)));

/**
 * @brief Static storage for response queue items
 *
 * Statically allocated buffer for 8 MqttOutgoingResponse structures.
 * Using static allocation via xQueueCreateStatic() for reliability.
 */
static uint8_t s_respQueueStorage[MQTT_RESP_QUEUE_DEPTH * sizeof(MqttOutgoingResponse)] __attribute__((aligned(4)));

/**
 * @brief Static queue control structure for command queue
 */
static StaticQueue_t s_cmdQueueStatic;

/**
 * @brief Static queue control structure for response queue
 */
static StaticQueue_t s_respQueueStatic;

/**
 * @brief Handle to the command queue
 */
static QueueHandle_t s_cmdQueue = NULL;

/**
 * @brief Handle to the response queue
 */
static QueueHandle_t s_respQueue = NULL;

/**
 * @brief Flag to track if queues have been initialized
 */
static bool s_queuesInitialized = false;

// ============================================================================
// Static Task Storage (Requirements 5.1, 5.3, 5.5, 5.6)
// ============================================================================

/**
 * @brief Static stack storage for Command Processor Task
 *
 * Statically allocated stack for the command processor task.
 * Size: CMD_PROCESSOR_STACK_SIZE (7680 bytes) - corrected for network/TLS safety
 */
static StackType_t s_cmdProcessorStack[CMD_PROCESSOR_STACK_SIZE / sizeof(StackType_t)];

/**
 * @brief Static task control block for Command Processor Task
 */
static StaticTask_t s_cmdProcessorTaskBuffer;

/**
 * @brief Handle to the Command Processor Task
 */
static TaskHandle_t s_cmdProcessorTaskHandle = NULL;

// ============================================================================
// Static Task Storage for Publisher Task (Requirements 5.2, 5.4, 5.5, 5.6)
// ============================================================================

/**
 * @brief Static stack storage for Publisher Task
 *
 * Statically allocated stack for the publisher task.
 * Size: PUBLISHER_STACK_SIZE (4096 bytes) - REVERTED for TLS safety
 */
static StackType_t s_publisherStack[PUBLISHER_STACK_SIZE / sizeof(StackType_t)];

/**
 * @brief Static task control block for Publisher Task
 */
static StaticTask_t s_publisherTaskBuffer;

/**
 * @brief Handle to the Publisher Task
 */
static TaskHandle_t s_publisherTaskHandle = NULL;

/**
 * @brief Flag to track if tasks are running
 */
static bool s_tasksRunning = false;

// Placeholder for MQTT manager reference
static MQTT5Manager* s_mqttManager = NULL;

// OTA status callback for rejecting commands during OTA updates
static OtaStatusCallback s_otaStatusCallback = NULL;

// ============================================================================
// Rate Limiting Implementation
// ============================================================================

/**
 * @brief Initialize rate limiting state
 *
 * Called on first rate limit check to set up initial timestamps.
 * Uses lazy initialization to ensure accurate timing from first request.
 */
static void initRateLimiting() {
    if (s_rateLimitInitialized) {
        return;
    }

    int64_t now = esp_timer_get_time();
    
    // Initialize token bucket with full capacity and current time
    s_requestBucket.tokens = REQUEST_BUCKET_CAPACITY;
    s_requestBucket.lastRefillTimeUs = now;
    
    // Initialize command timestamp to 0 (first command always allowed)
    s_lastCommandTimeUs = 0;
    
    s_rateLimitInitialized = true;
    
    ESP_LOGI(TAG, "Rate limiting initialized: bucket=%d tokens, refill=%lld ms, cmd_interval=%lld ms",
             REQUEST_BUCKET_CAPACITY, 
             (long long)(REQUEST_REFILL_INTERVAL_MS),
             (long long)(COMMAND_RATE_LIMIT_MS));
}

/**
 * @brief Check and apply token bucket rate limiting for requests
 *
 * Implements token bucket algorithm with time drift prevention:
 * - Refills tokens based on elapsed time since last refill
 * - Advances lastRefillTimeUs by exact token time to preserve fractional time
 * - Allows burst of up to REQUEST_BUCKET_CAPACITY requests
 * - Sustained rate of 1 request per REQUEST_REFILL_INTERVAL_MS
 *
 * @return true if request is allowed (token consumed), false if rate limited
 */
bool checkRequestRateLimit() {
    // Lazy initialization on first call
    if (!s_rateLimitInitialized) {
        initRateLimiting();
    }

    int64_t now = esp_timer_get_time();
    int64_t elapsed = now - s_requestBucket.lastRefillTimeUs;

    // Calculate tokens to add based on elapsed time
    // Using integer division preserves fractional time in lastRefillTimeUs
    int64_t tokensToAdd = elapsed / REQUEST_REFILL_INTERVAL_US;

    if (tokensToAdd > 0) {
        // Add tokens, cap at capacity
        int newTokens = (int)s_requestBucket.tokens + (int)tokensToAdd;
        if (newTokens > REQUEST_BUCKET_CAPACITY) {
            newTokens = REQUEST_BUCKET_CAPACITY;
        }
        s_requestBucket.tokens = (uint8_t)newTokens;

        // KEY: Advance lastRefillTimeUs by exact token time to prevent drift
        // This preserves the fractional time (remainder) for accurate long-term timing
        s_requestBucket.lastRefillTimeUs += tokensToAdd * REQUEST_REFILL_INTERVAL_US;

        ESP_LOGD(TAG, "Token refill: added %lld tokens, now %d tokens",
                 (long long)tokensToAdd, s_requestBucket.tokens);
    }

    // Check if we have tokens available
    if (s_requestBucket.tokens > 0) {
        s_requestBucket.tokens--;
        ESP_LOGD(TAG, "Request allowed, tokens remaining: %d", s_requestBucket.tokens);
        return true;
    }

    // Rate limited - calculate time until next token
    int64_t timeSinceLastRefill = now - s_requestBucket.lastRefillTimeUs;
    int64_t timeUntilNextToken = REQUEST_REFILL_INTERVAL_US - timeSinceLastRefill;
    if (timeUntilNextToken < 0) {
        timeUntilNextToken = 0;
    }

    ESP_LOGW(TAG, "Request rate limited (bucket empty). Next token in %lld ms",
             (long long)(timeUntilNextToken / 1000));
    return false;
}

/**
 * @brief Check and apply strict interval rate limiting for commands
 *
 * Enforces a minimum interval of COMMAND_RATE_LIMIT_MS between commands,
 * with RATE_LIMIT_TOLERANCE_MS tolerance to account for timing jitter.
 * The first command after startup is always allowed.
 *
 * @return true if command is allowed, false if rate limited
 */
bool checkCommandRateLimit() {
    // Lazy initialization on first call
    if (!s_rateLimitInitialized) {
        initRateLimiting();
    }

    int64_t now = esp_timer_get_time();

    // First command after startup is always allowed
    if (s_lastCommandTimeUs == 0) {
        s_lastCommandTimeUs = now;
        ESP_LOGD(TAG, "First command allowed (startup)");
        return true;
    }

    // Calculate elapsed time since last command
    int64_t elapsed = now - s_lastCommandTimeUs;

    // Check if minimum interval has passed (with tolerance for timing jitter)
    // Tolerance accounts for network delays, task scheduling, and microsecond precision
    if (elapsed >= (COMMAND_RATE_LIMIT_US - RATE_LIMIT_TOLERANCE_US)) {
        s_lastCommandTimeUs = now;
        ESP_LOGD(TAG, "Command allowed, interval: %lld ms", (long long)(elapsed / 1000));
        return true;
    }

    // Rate limited - calculate remaining wait time
    int64_t remainingWait = COMMAND_RATE_LIMIT_US - elapsed;
    ESP_LOGW(TAG, "Command rate limited. Wait %lld ms (interval: %lld ms)",
             (long long)(remainingWait / 1000), (long long)(elapsed / 1000));
    return false;
}

// ============================================================================
// Testing Support Functions
// ============================================================================

/**
 * @brief Reset rate limiting state for testing
 *
 * This function is intended for unit testing only. It resets all rate
 * limiting state to initial values.
 */
void resetRateLimitingForTest() {
    s_requestBucket.tokens = REQUEST_BUCKET_CAPACITY;
    s_requestBucket.lastRefillTimeUs = 0;
    s_lastCommandTimeUs = 0;
    s_rateLimitInitialized = false;
    ESP_LOGD(TAG, "Rate limiting state reset for testing");
}

/**
 * @brief Get current token count for testing
 *
 * @return Current number of tokens in the bucket
 */
uint8_t getTokenCountForTest() {
    return s_requestBucket.tokens;
}

/**
 * @brief Get last refill timestamp for testing
 *
 * @return Last refill timestamp in microseconds
 */
int64_t getLastRefillTimeUsForTest() {
    return s_requestBucket.lastRefillTimeUs;
}

/**
 * @brief Get last command timestamp for testing
 *
 * @return Last command timestamp in microseconds
 */
int64_t getLastCommandTimeUsForTest() {
    return s_lastCommandTimeUs;
}

/**
 * @brief Set token bucket state for testing
 *
 * Allows tests to set specific bucket state for edge case testing.
 *
 * @param tokens Number of tokens to set
 * @param lastRefillTimeUs Last refill timestamp to set
 */
void setTokenBucketStateForTest(uint8_t tokens, int64_t lastRefillTimeUs) {
    s_requestBucket.tokens = tokens;
    s_requestBucket.lastRefillTimeUs = lastRefillTimeUs;
    s_rateLimitInitialized = true;
    ESP_LOGD(TAG, "Token bucket state set for testing: tokens=%d, lastRefill=%lld",
             tokens, (long long)lastRefillTimeUs);
}

/**
 * @brief Set command timestamp for testing
 *
 * @param lastCommandTimeUs Last command timestamp to set
 */
void setCommandTimestampForTest(int64_t lastCommandTimeUs) {
    s_lastCommandTimeUs = lastCommandTimeUs;
    s_rateLimitInitialized = true;
    ESP_LOGD(TAG, "Command timestamp set for testing: %lld", (long long)lastCommandTimeUs);
}

// ============================================================================
// Helper Functions for Command Processing
// ============================================================================

/**
 * @brief Extract device ID from MQTT topic
 *
 * Topic format: {source}/{deviceId}/{category}
 * This function extracts the deviceId portion.
 *
 * @param topic The full MQTT topic string
 * @param deviceId Output buffer for device ID (must be at least MQTT_CMD_MAX_DEVICE_ID_LEN bytes)
 * @return true if device ID was extracted successfully, false otherwise
 */
static bool extractDeviceIdFromTopic(const char* topic, char* deviceId) {
    if (topic == NULL || deviceId == NULL) {
        return false;
    }

    // Make a local copy for tokenization
    char localTopic[MQTT_CMD_MAX_TOPIC_LEN];
    strncpy(localTopic, topic, MQTT_CMD_MAX_TOPIC_LEN - 1);
    localTopic[MQTT_CMD_MAX_TOPIC_LEN - 1] = '\0';

    char* savePtr = NULL;
    char* source = strtok_r(localTopic, "/", &savePtr);
    char* devId = strtok_r(NULL, "/", &savePtr);

    if (source == NULL || devId == NULL || strlen(devId) == 0) {
        return false;
    }

    strncpy(deviceId, devId, MQTT_CMD_MAX_DEVICE_ID_LEN - 1);
    deviceId[MQTT_CMD_MAX_DEVICE_ID_LEN - 1] = '\0';
    return true;
}

/**
 * @brief Process an incoming MQTT request
 *
 * Parses the JSON payload, calls the request callback, and queues the response.
 * 
 * HYBRID PAYLOAD ALLOCATION:
 * - WiFi scan responses (REQUEST_TYPE_RECEIVER_WIFI_SCAN) use dynamic allocation
 *   to avoid large stack buffers (~3KB). Ownership of the response string is
 *   transferred to the publisher task.
 * - All other responses use the fixed 512-byte buffer in MqttOutgoingResponse.
 *
 * @param cmd Pointer to the incoming command structure
 */
static void processRequest(const MqttIncomingCmd* cmd) {
    if (cmd == NULL || s_mqttManager == NULL) {
        ESP_LOGE(TAG, "processRequest: Invalid parameters");
        return;
    }

    ESP_LOGD(TAG, "Processing request: topic=%s, payloadLen=%u", cmd->topic, cmd->payloadLen);

    // Parse JSON payload
    cJSON* messageRoot = cJSON_ParseWithLength(cmd->payload, cmd->payloadLen);
    if (messageRoot == NULL) {
        ESP_LOGE(TAG, "Failed to parse request JSON payload");
        return;
    }

    // Extract request type
    if (!cJSON_HasObjectItem(messageRoot, "type")) {
        ESP_LOGE(TAG, "Request missing 'type' field");
        cJSON_Delete(messageRoot);
        return;
    }

    uint8_t requestTypeNumber = (uint8_t)(cJSON_GetObjectItem(messageRoot, "type")->valueint);
    DeviceRequestType requestType = numberToDeviceRequestType(requestTypeNumber);

    if (requestType == REQUEST_TYPE_INVALID) {
        ESP_LOGE(TAG, "Invalid request type: %d", requestTypeNumber);
        cJSON_Delete(messageRoot);
        return;
    }

    ESP_LOGI(TAG, "REQUEST received - Type=%d (%s), ResponseTopic=%s",
             requestTypeNumber, deviceRequestTypeToString(requestType), cmd->responseTopic);

    // Get callback from MQTT manager
    auto callback = s_mqttManager->getOnDeviceRequestReceivedListener();
    if (callback == NULL) {
        ESP_LOGW(TAG, "No request callback registered");
        cJSON_Delete(messageRoot);
        return;
    }

    // Call callback - it returns the response string
    char* response = NULL;
    char* responseTopicCopy = (char*)cmd->responseTopic; // Callback expects non-const
    callback(requestType, responseTopicCopy, response);

    cJSON_Delete(messageRoot);

    // If callback provided a response, queue it
    if (response != NULL) {
        MqttOutgoingResponse outResponse;
        memset(&outResponse, 0, sizeof(outResponse));

        // Copy response topic
        strncpy(outResponse.topic, cmd->responseTopic, MQTT_CMD_MAX_RESPONSE_TOPIC - 1);
        outResponse.topic[MQTT_CMD_MAX_RESPONSE_TOPIC - 1] = '\0';
        outResponse.topicLen = (uint16_t)strlen(outResponse.topic);

        // Extract device ID from topic
        extractDeviceIdFromTopic(cmd->topic, outResponse.deviceId);

        // Set QoS and retain
        outResponse.qos = QOS_1;
        outResponse.retain = false;

        size_t responseLen = strlen(response);

        // HYBRID PAYLOAD ALLOCATION:
        // WiFi scan uses dynamic allocation (transfer ownership of response string)
        // All other requests use fixed buffer (copy and free response string)
        if (requestType == REQUEST_TYPE_RECEIVER_WIFI_SCAN) {
            // WiFi scan: Transfer ownership of dynamically allocated response
            // The callback already allocated this memory, we just pass the pointer through
            if (responseLen >= MQTT_CMD_MAX_PAYLOAD_LEN) {
                ESP_LOGW(TAG, "WiFi scan response truncated from %zu to %d bytes",
                         responseLen, MQTT_CMD_MAX_PAYLOAD_LEN - 1);
                responseLen = MQTT_CMD_MAX_PAYLOAD_LEN - 1;
                response[responseLen] = '\0';  // Truncate in place
            }
            
            outResponse.dynamicPayload = response;
            outResponse.usesDynamicPayload = true;
            outResponse.payloadLen = (uint16_t)responseLen;
            
            ESP_LOGD(TAG, "WiFi scan using dynamic payload: %zu bytes", responseLen);
            
            // Queue the response - if this fails, we must free the memory
            if (MqttCommandProcessor::queueOutgoingResponse(&outResponse)) {
                ESP_LOGI(TAG, "WiFi scan response queued: topic=%s, size=%zu", 
                         outResponse.topic, responseLen);
                // Success: ownership transferred to publisher task, don't free here
            } else {
                // Queue failed: we still own the memory, must free it
                free(outResponse.dynamicPayload);
                ESP_LOGE(TAG, "Failed to queue WiFi scan response, memory freed");
            }
        } else {
            // Other requests: Copy to fixed buffer
            if (responseLen >= MQTT_CMD_FIXED_PAYLOAD_LEN) {
                ESP_LOGW(TAG, "Response payload truncated from %zu to %d bytes",
                         responseLen, MQTT_CMD_FIXED_PAYLOAD_LEN - 1);
                responseLen = MQTT_CMD_FIXED_PAYLOAD_LEN - 1;
            }
            
            memcpy(outResponse.fixedPayload, response, responseLen);
            outResponse.fixedPayload[responseLen] = '\0';
            outResponse.usesDynamicPayload = false;
            outResponse.payloadLen = (uint16_t)responseLen;
            
            // Free the response string allocated by callback
            free(response);

            // Queue the response
            if (MqttCommandProcessor::queueOutgoingResponse(&outResponse)) {
                ESP_LOGI(TAG, "Request response queued: topic=%s", outResponse.topic);
            }
        }
    } else {
        ESP_LOGD(TAG, "Request callback returned no response");
    }
}

/**
 * @brief Process an incoming MQTT command
 *
 * Parses the JSON payload, calls the command callback, and queues the ACK response.
 *
 * @param cmd Pointer to the incoming command structure
 */
static void processCommand(const MqttIncomingCmd* cmd) {
    if (cmd == NULL || s_mqttManager == NULL) {
        ESP_LOGE(TAG, "processCommand: Invalid parameters");
        return;
    }

    ESP_LOGD(TAG, "Processing command: topic=%s, payloadLen=%u", cmd->topic, cmd->payloadLen);

    // Parse JSON payload
    cJSON* messageRoot = cJSON_ParseWithLength(cmd->payload, cmd->payloadLen);
    if (messageRoot == NULL) {
        ESP_LOGE(TAG, "Failed to parse command JSON payload");
        return;
    }

    // Extract command type
    if (!cJSON_HasObjectItem(messageRoot, "type")) {
        ESP_LOGE(TAG, "Command missing 'type' field");
        cJSON_Delete(messageRoot);
        return;
    }

    uint8_t commandTypeNumber = (uint8_t)(cJSON_GetObjectItem(messageRoot, "type")->valueint);
    DeviceCommandType commandType = numberToDeviceCommandType(commandTypeNumber);

    if (commandType == COMMAND_TYPE_INVALID) {
        ESP_LOGE(TAG, "Invalid command type: %d", commandTypeNumber);
        cJSON_Delete(messageRoot);
        return;
    }

    // Extract value and valueType if present
    uint32_t* valuePtr = NULL;
    uint8_t valueType = 0xFF;

    if (cJSON_HasObjectItem(messageRoot, "valueType") && cJSON_HasObjectItem(messageRoot, "value")) {
        valueType = (uint8_t)cJSON_GetObjectItem(messageRoot, "valueType")->valueint;

        if (valueType == 0x01) {
            // Float value
            valuePtr = (uint32_t*)malloc(sizeof(uint32_t));
            if (valuePtr != NULL) {
                float val = (float)cJSON_GetObjectItem(messageRoot, "value")->valuedouble;
                memcpy(valuePtr, &val, sizeof(uint32_t));
                ESP_LOGI(TAG, "COMMAND received - Type=%d (%s), ValueType=FLOAT, Value=%.2f",
                         commandTypeNumber, deviceCommandTypeToString(commandType), val);
            }
        } else if (valueType == 0x02) {
            // Integer value
            valuePtr = (uint32_t*)malloc(sizeof(uint32_t));
            if (valuePtr != NULL) {
                *valuePtr = (uint32_t)cJSON_GetObjectItem(messageRoot, "value")->valueint;
                ESP_LOGI(TAG, "COMMAND received - Type=%d (%s), ValueType=INTEGER, Value=%lu",
                         commandTypeNumber, deviceCommandTypeToString(commandType), *valuePtr);
            }
        } else {
            ESP_LOGE(TAG, "Invalid value type: %d (0x%02x)", valueType, valueType);
        }
    } else {
        ESP_LOGI(TAG, "COMMAND received - Type=%d (%s) (no value)",
                 commandTypeNumber, deviceCommandTypeToString(commandType));
    }

    // Get callback from MQTT manager
    auto callback = s_mqttManager->getOnDeviceCommandReceivedListener();
    bool success = false;

    if (callback != NULL) {
        char* responseTopicCopy = (char*)cmd->responseTopic;
        success = callback(commandType, responseTopicCopy, valuePtr, valueType);
    } else {
        ESP_LOGW(TAG, "No command callback registered");
    }

    cJSON_Delete(messageRoot);

    // Build ACK response
    cJSON* responseRoot = cJSON_CreateObject();
    if (responseRoot == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object for command ACK");
        if (valuePtr != NULL) free(valuePtr);
        return;
    }

    cJSON_AddNumberToObject(responseRoot, "type", (double)commandType);
    cJSON_AddNumberToObject(responseRoot, "status",
                            success ? COMMAND_ACK_TYPE_COMMAND_SUCCEEDED : COMMAND_ACK_TYPE_COMMAND_FAILED);

    char* responseString = cJSON_PrintUnformatted(responseRoot);
    cJSON_Delete(responseRoot);

    if (responseString == NULL) {
        ESP_LOGE(TAG, "Failed to serialize command ACK JSON");
        if (valuePtr != NULL) free(valuePtr);
        return;
    }

    // Build and queue response
    MqttOutgoingResponse outResponse;
    memset(&outResponse, 0, sizeof(outResponse));

    // Copy response topic
    strncpy(outResponse.topic, cmd->responseTopic, MQTT_CMD_MAX_RESPONSE_TOPIC - 1);
    outResponse.topic[MQTT_CMD_MAX_RESPONSE_TOPIC - 1] = '\0';
    outResponse.topicLen = (uint16_t)strlen(outResponse.topic);

    // Copy response payload (command ACKs are always small, use fixed buffer)
    size_t responseLen = strlen(responseString);
    if (responseLen >= MQTT_CMD_FIXED_PAYLOAD_LEN) {
        ESP_LOGW(TAG, "Command ACK truncated from %zu to %d bytes",
                 responseLen, MQTT_CMD_FIXED_PAYLOAD_LEN - 1);
        responseLen = MQTT_CMD_FIXED_PAYLOAD_LEN - 1;
    }
    memcpy(outResponse.fixedPayload, responseString, responseLen);
    outResponse.fixedPayload[responseLen] = '\0';
    outResponse.usesDynamicPayload = false;
    outResponse.payloadLen = (uint16_t)responseLen;

    // Extract device ID from topic
    extractDeviceIdFromTopic(cmd->topic, outResponse.deviceId);

    // Set QoS and retain
    outResponse.qos = QOS_1;
    outResponse.retain = false;

    // Queue the response
    if (MqttCommandProcessor::queueOutgoingResponse(&outResponse)) {
        ESP_LOGI(TAG, "Command ACK queued: topic=%s, status=%s",
                 outResponse.topic, success ? "SUCCESS" : "FAILED");
    }

    // Cleanup
    cJSON_free(responseString);
    if (valuePtr != NULL) free(valuePtr);
}

// ============================================================================
// Command Processor Task (Requirements 3.2, 5.1, 5.3, 5.5, 5.6)
// ============================================================================

/**
 * @brief Command Processor Task main function
 *
 * This task:
 * - Blocks on the command queue waiting for incoming messages
 * - Applies rate limiting (token bucket for requests, interval for commands)
 * - Parses JSON payloads
 * - Executes appropriate callbacks
 * - Queues responses for publishing
 *
 * @param pvParameters Task parameters (unused)
 */
static void commandProcessorTask(void* pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "Command Processor Task started (priority=%d, core=%d, stack=%d bytes)",
             CMD_PROCESSOR_PRIORITY, MQTT_TASKS_CORE, CMD_PROCESSOR_STACK_SIZE);

    MqttIncomingCmd cmd;

    while (true) {
        // Block waiting for command from queue
        if (xQueueReceive(s_cmdQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "Received message: category=%d, source=%d, topic=%.32s",
                     (int)cmd.category, (int)cmd.source, cmd.topic);

            // Check if OTA update is in progress - reject all commands/requests during OTA
            if (s_otaStatusCallback != NULL && s_otaStatusCallback()) {
                ESP_LOGW(TAG, "OTA update in progress - rejecting MQTT %s (topic: %.32s)",
                         cmd.category == TOPIC_CATEGORY_COMMAND ? "command" : "request",
                         cmd.topic);
                continue;
            }

            // Process based on category with appropriate rate limiting
            if (cmd.category == TOPIC_CATEGORY_REQUEST) {
                // Apply token bucket rate limiting for requests
                if (!checkRequestRateLimit()) {
                    // Rate limited - message already logged in checkRequestRateLimit()
                    continue;
                }
                processRequest(&cmd);
            } else if (cmd.category == TOPIC_CATEGORY_COMMAND) {
                // Apply strict interval rate limiting for commands
                if (!checkCommandRateLimit()) {
                    // Rate limited - message already logged in checkCommandRateLimit()
                    continue;
                }
                processCommand(&cmd);
            } else {
                ESP_LOGW(TAG, "Unknown message category: %d", (int)cmd.category);
            }
        }
    }
}

/**
 * @brief Create and start the Command Processor Task
 *
 * Creates the task using static allocation for reliability.
 *
 * @return ESP_OK on success, ESP_FAIL if task creation fails
 */
static esp_err_t createCommandProcessorTask() {
    if (s_cmdProcessorTaskHandle != NULL) {
        ESP_LOGW(TAG, "Command Processor Task already created");
        return ESP_OK;
    }

    s_cmdProcessorTaskHandle = xTaskCreateStaticPinnedToCore(
        commandProcessorTask,                           // Task function
        "MqttCmdProc",                                  // Task name
        CMD_PROCESSOR_STACK_SIZE / sizeof(StackType_t), // Stack size in words
        NULL,                                           // Task parameters
        CMD_PROCESSOR_PRIORITY,                         // Task priority
        s_cmdProcessorStack,                            // Static stack buffer
        &s_cmdProcessorTaskBuffer,                      // Static task buffer
        MQTT_TASKS_CORE                                 // Core to pin task to
    );

    if (s_cmdProcessorTaskHandle == NULL) {
        ESP_LOGE(TAG, "Failed to create Command Processor Task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Command Processor Task created: priority=%d, core=%d, stack=%d bytes",
             CMD_PROCESSOR_PRIORITY, MQTT_TASKS_CORE, CMD_PROCESSOR_STACK_SIZE);

    return ESP_OK;
}

// ============================================================================
// Publisher Task (Requirements 3.3, 5.2, 5.4, 5.5, 5.6)
// ============================================================================

/**
 * @brief Publisher Task main function
 *
 * This task:
 * - Blocks on the response queue waiting for outgoing messages
 * - Builds MQTT5Message from MqttOutgoingResponse
 * - Handles hybrid payload allocation (fixed buffer vs dynamic pointer)
 * - Calls esp_mqtt_client_enqueue() for non-blocking publish
 * - Cleans up dynamic payloads after publishing
 * - Logs success/failure of publish operations
 *
 * @param pvParameters Task parameters (unused)
 */
static void publisherTask(void* pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "Publisher Task started (priority=%d, core=%d, stack=%d bytes)",
             PUBLISHER_PRIORITY, MQTT_TASKS_CORE, PUBLISHER_STACK_SIZE);

    MqttOutgoingResponse response;

    while (true) {
        // Block waiting for response from queue with 1 second timeout
        // Timeout allows periodic checking of task state and prevents indefinite blocking
        if (xQueueReceive(s_respQueue, &response, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_LOGD(TAG, "Publishing response: topic=%s, payloadLen=%u, dynamic=%d",
                     response.topic, (unsigned)response.payloadLen, response.usesDynamicPayload);

            // Select payload source based on allocation type
            const char* payloadSource = response.usesDynamicPayload 
                ? response.dynamicPayload 
                : response.fixedPayload;

            // Validate payload source
            if (payloadSource == NULL) {
                ESP_LOGE(TAG, "Payload source is NULL (dynamic=%d)", response.usesDynamicPayload);
                // No cleanup needed for NULL dynamic payload
                continue;
            }

            // Check if MQTT manager is available
            if (s_mqttManager == NULL) {
                ESP_LOGE(TAG, "MQTT manager not available, dropping response");
                // Cleanup dynamic payload if present
                if (response.usesDynamicPayload && response.dynamicPayload != NULL) {
                    free(response.dynamicPayload);
                    ESP_LOGD(TAG, "Freed dynamic payload after MQTT manager unavailable");
                }
                continue;
            }

            // Check if connected to broker
            if (!s_mqttManager->isConnected()) {
                ESP_LOGW(TAG, "Not connected to MQTT broker, dropping response (topic: %.32s...)",
                         response.topic);
                // Cleanup dynamic payload if present
                if (response.usesDynamicPayload && response.dynamicPayload != NULL) {
                    free(response.dynamicPayload);
                    ESP_LOGD(TAG, "Freed dynamic payload after broker disconnect");
                }
                continue;
            }

            // Build MQTT5Message from MqttOutgoingResponse
            // IMPORTANT: publish() takes ownership and frees the message, so allocate on heap
            MQTT5Message* mqttMsg = (MQTT5Message*)malloc(sizeof(MQTT5Message));
            if (mqttMsg == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for MQTT5Message");
                // Cleanup dynamic payload if present
                if (response.usesDynamicPayload && response.dynamicPayload != NULL) {
                    free(response.dynamicPayload);
                    ESP_LOGD(TAG, "Freed dynamic payload after MQTT5Message alloc failure");
                }
                continue;
            }
            memset(mqttMsg, 0, sizeof(MQTT5Message));

            // Allocate and copy topic
            mqttMsg->topic = (char*)malloc(response.topicLen + 1);
            if (mqttMsg->topic == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for topic");
                free(mqttMsg);
                // Cleanup dynamic payload if present
                if (response.usesDynamicPayload && response.dynamicPayload != NULL) {
                    free(response.dynamicPayload);
                    ESP_LOGD(TAG, "Freed dynamic payload after topic alloc failure");
                }
                continue;
            }
            memcpy(mqttMsg->topic, response.topic, response.topicLen);
            mqttMsg->topic[response.topicLen] = '\0';

            // Allocate and copy message payload from the appropriate source
            mqttMsg->message = (char*)malloc(response.payloadLen + 1);
            if (mqttMsg->message == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for message");
                free(mqttMsg->topic);
                free(mqttMsg);
                // Cleanup dynamic payload if present
                if (response.usesDynamicPayload && response.dynamicPayload != NULL) {
                    free(response.dynamicPayload);
                    ESP_LOGD(TAG, "Freed dynamic payload after message alloc failure");
                }
                continue;
            }
            memcpy(mqttMsg->message, payloadSource, response.payloadLen);
            mqttMsg->message[response.payloadLen] = '\0';

            // Allocate and set content type (required by publish())
            static const char contentTypeStr[] = "application/json";
            mqttMsg->contentType = (char*)malloc(sizeof(contentTypeStr));
            if (mqttMsg->contentType == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for contentType");
                free(mqttMsg->message);
                free(mqttMsg->topic);
                free(mqttMsg);
                // Cleanup dynamic payload if present
                if (response.usesDynamicPayload && response.dynamicPayload != NULL) {
                    free(response.dynamicPayload);
                    ESP_LOGD(TAG, "Freed dynamic payload after contentType alloc failure");
                }
                continue;
            }
            strcpy(mqttMsg->contentType, contentTypeStr);

            // Allocate and copy device ID
            size_t deviceIdLen = strlen(response.deviceId);
            if (deviceIdLen > 0) {
                mqttMsg->deviceID = (char*)malloc(deviceIdLen + 1);
                if (mqttMsg->deviceID != NULL) {
                    memcpy(mqttMsg->deviceID, response.deviceId, deviceIdLen);
                    mqttMsg->deviceID[deviceIdLen] = '\0';
                }
            }

            // Set QoS and retain flag
            mqttMsg->qos = response.qos;
            mqttMsg->retain = response.retain;

            // Response topic is not used for responses (set to NULL)
            mqttMsg->responseTopic = NULL;

            // Publish using the MQTT manager's publish method
            // NOTE: publish() takes ownership of mqttMsg and frees it (success or failure)
            bool success = s_mqttManager->publish(mqttMsg);

            if (success) {
                ESP_LOGI(TAG, "Response published: topic=%s, qos=%d",
                         response.topic, (int)response.qos);
            } else {
                ESP_LOGE(TAG, "Failed to publish response: topic=%s",
                         response.topic);
            }
            
            // Cleanup dynamic payload after publish (success or failure)
            // The MQTT5Message was already freed by publish(), but we still own the dynamic payload
            if (response.usesDynamicPayload && response.dynamicPayload != NULL) {
                free(response.dynamicPayload);
                ESP_LOGD(TAG, "Freed dynamic payload after publish");
            }
            
            // Do NOT free mqttMsg or its members here - publish() handles cleanup
        }
        // Timeout occurred - this is normal, just continue the loop
        // This allows the task to check for shutdown signals periodically
    }
}

/**
 * @brief Create and start the Publisher Task
 *
 * Creates the task using static allocation for reliability.
 *
 * @return ESP_OK on success, ESP_FAIL if task creation fails
 */
static esp_err_t createPublisherTask() {
    if (s_publisherTaskHandle != NULL) {
        ESP_LOGW(TAG, "Publisher Task already created");
        return ESP_OK;
    }

    s_publisherTaskHandle = xTaskCreateStaticPinnedToCore(
        publisherTask,                              // Task function
        "MqttPub",                                  // Task name
        PUBLISHER_STACK_SIZE / sizeof(StackType_t), // Stack size in words
        NULL,                                       // Task parameters
        PUBLISHER_PRIORITY,                         // Task priority
        s_publisherStack,                           // Static stack buffer
        &s_publisherTaskBuffer,                     // Static task buffer
        MQTT_TASKS_CORE                             // Core to pin task to
    );

    if (s_publisherTaskHandle == NULL) {
        ESP_LOGE(TAG, "Failed to create Publisher Task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Publisher Task created: priority=%d, core=%d, stack=%d bytes",
             PUBLISHER_PRIORITY, MQTT_TASKS_CORE, PUBLISHER_STACK_SIZE);

    return ESP_OK;
}

// ============================================================================
// Queue Initialization (Requirements 4.1, 4.2, 4.3)
// ============================================================================

/**
 * @brief Initialize static queues for command and response transfer
 *
 * Creates command and response queues using xQueueCreateStatic() for
 * reliability. Static allocation ensures queues are always available
 * and avoids heap fragmentation.
 *
 * @return ESP_OK on success, ESP_FAIL if queue creation fails
 */
static esp_err_t initQueues() {
    if (s_queuesInitialized) {
        ESP_LOGW(TAG, "Queues already initialized");
        return ESP_OK;
    }

    // Create command queue with static storage (Requirement 4.1)
    s_cmdQueue = xQueueCreateStatic(
        MQTT_CMD_QUEUE_DEPTH,           // Queue depth: 8 items
        sizeof(MqttIncomingCmd),        // Item size
        s_cmdQueueStorage,              // Static storage buffer
        &s_cmdQueueStatic               // Static queue structure
    );

    if (s_cmdQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create command queue");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Command queue created: depth=%d, item_size=%u bytes",
             MQTT_CMD_QUEUE_DEPTH, (unsigned)sizeof(MqttIncomingCmd));

    // Create response queue with static storage (Requirement 4.2)
    s_respQueue = xQueueCreateStatic(
        MQTT_RESP_QUEUE_DEPTH,          // Queue depth: 8 items
        sizeof(MqttOutgoingResponse),   // Item size
        s_respQueueStorage,             // Static storage buffer
        &s_respQueueStatic              // Static queue structure
    );

    if (s_respQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create response queue");
        // Note: We don't delete cmdQueue here as it's statically allocated
        s_cmdQueue = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Response queue created: depth=%d, item_size=%u bytes",
             MQTT_RESP_QUEUE_DEPTH, (unsigned)sizeof(MqttOutgoingResponse));

    s_queuesInitialized = true;
    return ESP_OK;
}

// ============================================================================
// MqttCommandProcessor Namespace Implementation
// ============================================================================

namespace MqttCommandProcessor {

esp_err_t init(MQTT5Manager* mqttManager) {
    if (mqttManager == NULL) {
        ESP_LOGE(TAG, "init: mqttManager is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_mqttManager = mqttManager;
    
    // Initialize rate limiting
    initRateLimiting();

    // Initialize static queues (Requirements 4.1, 4.2, 4.3)
    esp_err_t ret = initQueues();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize queues");
        return ret;
    }

    // Create Command Processor Task (Requirements 5.1, 5.3, 5.5, 5.6)
    ret = createCommandProcessorTask();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create Command Processor Task");
        return ret;
    }

    // Create Publisher Task (Requirements 5.2, 5.4, 5.5, 5.6)
    ret = createPublisherTask();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create Publisher Task");
        return ret;
    }

    s_tasksRunning = true;
    ESP_LOGI(TAG, "MqttCommandProcessor initialized successfully");
    return ESP_OK;
}

void setOtaStatusCallback(OtaStatusCallback callback) {
    s_otaStatusCallback = callback;
    ESP_LOGI(TAG, "OTA status callback %s", callback != NULL ? "registered" : "cleared");
}

bool queueIncomingCommand(const MqttIncomingCmd* cmd) {
    if (cmd == NULL) {
        ESP_LOGE(TAG, "queueIncomingCommand: cmd is NULL");
        return false;
    }

    if (s_cmdQueue == NULL) {
        ESP_LOGE(TAG, "queueIncomingCommand: Queue not initialized");
        return false;
    }

    // Copy command to queue with 10ms timeout (Requirement 4.4)
    BaseType_t result = xQueueSend(s_cmdQueue, cmd, pdMS_TO_TICKS(10));

    if (result != pdTRUE) {
        // Queue full - log warning and drop message (Requirement 4.4)
        ESP_LOGW(TAG, "Command queue full, dropping message (topic: %.32s...)",
                 cmd->topic);
        return false;
    }

    ESP_LOGD(TAG, "Command queued: category=%d, source=%d, topic=%.32s",
             (int)cmd->category, (int)cmd->source, cmd->topic);
    return true;
}

bool queueOutgoingResponse(const MqttOutgoingResponse* response) {
    if (response == NULL) {
        ESP_LOGE(TAG, "queueOutgoingResponse: response is NULL");
        return false;
    }

    if (s_respQueue == NULL) {
        ESP_LOGE(TAG, "queueOutgoingResponse: Queue not initialized");
        return false;
    }

    // Copy response to queue with 100ms timeout (Requirement 4.5)
    BaseType_t result = xQueueSend(s_respQueue, response, pdMS_TO_TICKS(100));

    if (result != pdTRUE) {
        // Queue full after timeout - log error (Requirement 4.5)
        ESP_LOGE(TAG, "Response queue full after 100ms timeout, response lost (topic: %.32s...)",
                 response->topic);
        return false;
    }

    ESP_LOGD(TAG, "Response queued: topic=%.32s, payload_len=%u",
             response->topic, (unsigned)response->payloadLen);
    return true;
}

uint32_t getCmdQueueFreeSlots() {
    if (s_cmdQueue == NULL) {
        return 0;
    }
    return (uint32_t)uxQueueSpacesAvailable(s_cmdQueue);
}

uint32_t getRespQueueFreeSlots() {
    if (s_respQueue == NULL) {
        return 0;
    }
    return (uint32_t)uxQueueSpacesAvailable(s_respQueue);
}

void shutdown() {
    // Stop tasks first
    s_tasksRunning = false;

    // Delete Command Processor Task if running
    // Note: Static tasks cannot be deleted with vTaskDelete, but we can
    // signal them to stop. For now, we just reset the handle.
    // In a real implementation, we would signal the task to exit its loop.
    if (s_cmdProcessorTaskHandle != NULL) {
        // For static tasks, we cannot delete them, but we can reset the handle
        // The task will continue running but will be orphaned
        // In production, we would need a proper shutdown signal mechanism
        ESP_LOGW(TAG, "Command Processor Task handle reset (static task cannot be deleted)");
        s_cmdProcessorTaskHandle = NULL;
    }

    // Delete Publisher Task if running
    if (s_publisherTaskHandle != NULL) {
        ESP_LOGW(TAG, "Publisher Task handle reset (static task cannot be deleted)");
        s_publisherTaskHandle = NULL;
    }

    // Reset rate limiting state
    resetRateLimitingForTest();
    
    // Reset queue state
    // Note: Static queues don't need to be deleted, but we reset the handles
    // and initialized flag to allow re-initialization
    s_cmdQueue = NULL;
    s_respQueue = NULL;
    s_queuesInitialized = false;
    
    s_mqttManager = NULL;
    
    ESP_LOGI(TAG, "MqttCommandProcessor shutdown");
}

} // namespace MqttCommandProcessor
