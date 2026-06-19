#include "MQTTManager.h"
#include "MqttCommandProcessor.h"
#include "esp_random.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "string.h"
#include "cJSON.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"
#include <time.h>

#define TAG "MQTTManager"

#define CORRELATION_DATA_BUFFER_SIZE ((sizeof(uint32_t) * 2) + 1)

// Note: topicString was previously a global buffer which caused race conditions
// when multiple MQTT messages arrived concurrently. It has been replaced with
// a local buffer in mqtt5_event_handler() for thread safety.

class MQTT5Manager; // Forward declaration

// Timer callback function to attempt MQTT reconnection

/**
 * @brief Frees all dynamically allocated fields of an MQTT5Message and the struct itself.
 *
 * @param message Pointer to the MQTT5Message to free. Safe to call with NULL.
 */
void freeMqttMessage(MQTT5Message *message)
{
    if (message != NULL)
    {
        if (message->contentType != NULL)
        {
            free(message->contentType);
            message->contentType = NULL;
        }
        if (message->deviceID != NULL)
        {
            free(message->deviceID);
            message->deviceID = NULL;
        }
        if (message->message != NULL)
        {
            free(message->message);
            message->message = NULL;
        }
        if (message->responseTopic != NULL)
        {
            free(message->responseTopic);
            message->responseTopic = NULL;
        }
        if (message->topic != NULL)
        {
            free(message->topic);
            message->topic = NULL;
        }
        free(message);
        message = NULL;
    }
}

const char *topic_category_to_string(MQTTtopicCategories category)
{
    switch (category)
    {
    case TOPIC_CATEGORY_WATER_LEVEL_DATA:
        return "water-level-data";
    case TOPIC_CATEGORY_DEVICE_LOGS:
        return "device-logs";
    case TOPIC_CATEGORY_REQUEST:
        return "request";
    case TOPIC_CATEGORY_RESPONSE:
        return "response";
    case TOPIC_CATEGORY_COMMAND:
        return "command";
    case TOPIC_CATEGORY_COMMAND_ACK:
        return "commandACK";
    case TOPIC_CATEGORY_EVENT:
        return "event";
    case TOPIC_CATEGORY_EVENT_ACK:
        return "eventACK";
    case TOPIC_CATEGORY_INVALID:
        return "INV";
    }
    return "";
}

const char *message_source_to_string(MQTTMessageSourceTypes source)
{
    switch (source)
    {
    case MESSAGE_SOURCE_DEVICE:
        return "src-dev";
    case MESSAGE_SOURCE_SERVER:
        return "src-server";
    case MESSAGE_SOURCE_CONTROL:
        return "src-control";
    case MESSAGE_SOURCE_INVALID:
        return "INV";
    }
    return "";
}

MQTTtopicCategories string_to_topic_category(char *str)
{
    if (str == NULL || strlen(str) == 0)
        return TOPIC_CATEGORY_INVALID;

    if (strcmp(str, "water-level-data") == 0)
        return TOPIC_CATEGORY_WATER_LEVEL_DATA;
    else if (strcmp(str, "device-logs") == 0)
        return TOPIC_CATEGORY_DEVICE_LOGS;
    else if (strcmp(str, "request") == 0)
        return TOPIC_CATEGORY_REQUEST;
    else if (strcmp(str, "response") == 0)
        return TOPIC_CATEGORY_RESPONSE;
    else if (strcmp(str, "command") == 0)
        return TOPIC_CATEGORY_COMMAND;
    else if (strcmp(str, "commandACK") == 0)
        return TOPIC_CATEGORY_COMMAND_ACK;
    else
        return TOPIC_CATEGORY_INVALID;
}

MQTTMessageSourceTypes string_to_message_source(char *str)
{
    if (str == NULL || strlen(str) == 0)
        return MESSAGE_SOURCE_INVALID;
    if (strcmp(str, "src-dev") == 0)
        return MESSAGE_SOURCE_DEVICE;
    else if (strcmp(str, "src-server") == 0)
        return MESSAGE_SOURCE_SERVER;
    else if (strcmp(str, "src-control") == 0)
        return MESSAGE_SOURCE_CONTROL;
    else
        return MESSAGE_SOURCE_INVALID;
}

// This is the event handler for all the mqtt related events
void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    MQTT5Manager *mqttManager = (MQTT5Manager *)handler_args;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
        ESP_LOGI(TAG, "MQTT Event : MQTT_EVENT_CONNECTED");

        xEventGroupClearBits(mqttManager->mqttEventGroupHandle, MQTT_BROKER_DISCONNECTED_BIT);
        mqttManager->connectedToBroker = true;
        xEventGroupSetBits(mqttManager->mqttEventGroupHandle, MQTT_BROKER_CONNECTED_BIT);
        if (mqttManager->onMQTTBrokerConnectedListener != NULL)
            mqttManager->onMQTTBrokerConnectedListener();
    }
    break;
    case MQTT_EVENT_DISCONNECTED:
    {
        ESP_LOGW(TAG, "MQTT Broker disconnected. Client will reconnect automatically.");
        ESP_LOGW(TAG, "Disconnect reason: %s", esp_err_to_name(event->error_handle->esp_tls_last_esp_err));
        ESP_LOGW(TAG, "Disconnect details: error_type=%d, connect_return_code=%d, esp_tls_stack_err=%d, esp_transport_sock_errno=%d",
                 event->error_handle->error_type,
                 event->error_handle->connect_return_code,
                 event->error_handle->esp_tls_stack_err,
                 event->error_handle->esp_transport_sock_errno);

        mqttManager->connectedToBroker = false;
        xEventGroupSetBits(mqttManager->mqttEventGroupHandle, MQTT_BROKER_DISCONNECTED_BIT);
        if (mqttManager->onMQTTBrokerDisconnectedListener != NULL)
        {
            mqttManager->onMQTTBrokerDisconnectedListener();
        }
    }
    break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT Event: SUBSCRIBED - MsgID=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT Event: UNSUBSCRIBED - MsgID=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        // Log only for QoS > 0 to reduce UART traffic while keeping important confirmations
        if (event->qos > 0)
        {
            ESP_LOGD(TAG, "MQTT Event: PUBLISHED - MsgID=%d, QoS=%d", event->msg_id, event->qos);
        }
        break;
    case MQTT_EVENT_DATA:
    {
        /**
         * Simplified MQTT Event Handler - Queue-Only Operation
         * 
         * This handler implements the three-task decoupled architecture:
         * - Copies event data to a fixed-size MqttIncomingCmd structure
         * - Queues the command for processing by the Command Processor Task
         * - Returns immediately without blocking operations
         * 
         * NO callbacks, NO mutex acquisition, NO publish calls
         * (Requirements 3.1, 3.4, 3.5, 3.6)
         */
        ESP_LOGI(TAG, "MQTT Event: DATA_RECEIVED - Topic=%.*s, DataLen=%d", 
                 event->topic_len, event->topic, event->data_len);

        // Step 1: Use local buffer for thread-safe topic parsing (Requirement 2.1)
        char localTopicBuffer[MAX_MQTT_TOPIC_LENGTH];
        memset(localTopicBuffer, '\0', MAX_MQTT_TOPIC_LENGTH);
        
        // Safely copy topic data, ensuring we don't overflow the buffer
        size_t copyLen = (event->topic_len < MAX_MQTT_TOPIC_LENGTH - 1) 
                         ? event->topic_len 
                         : MAX_MQTT_TOPIC_LENGTH - 1;
        memcpy(localTopicBuffer, event->topic, copyLen);

        // Step 2: Parse topic using strtok_r for thread safety (Requirement 2.2)
        char *savePtr = NULL;
        char *messageSourceString = strtok_r(localTopicBuffer, "/", &savePtr);
        char *deviceID = strtok_r(NULL, "/", &savePtr);
        char *categoryString = strtok_r(NULL, "/", &savePtr);

        // Step 3: Validate parsed components
        MQTTMessageSourceTypes messageSource = string_to_message_source(messageSourceString);
        MQTTtopicCategories topicCategory = string_to_topic_category(categoryString);
        
        if (deviceID == NULL || strlen(deviceID) == 0)
        {
            ESP_LOGE(TAG, "Received Invalid Data - missing deviceID");
            return;
        }

        // Validate message source (must be SERVER or CONTROL)
        if (messageSource != MESSAGE_SOURCE_SERVER && messageSource != MESSAGE_SOURCE_CONTROL)
        {
            ESP_LOGE(TAG, "Received message from invalid source: %s", messageSourceString);
            return;
        }

        // Validate topic category (must be REQUEST or COMMAND)
        if (topicCategory != TOPIC_CATEGORY_REQUEST && topicCategory != TOPIC_CATEGORY_COMMAND)
        {
            ESP_LOGE(TAG, "Received invalid topic category: %s", categoryString);
            return;
        }

        // Step 4: Validate response topic exists (required for requests/commands)
        if (event->property->response_topic == NULL || event->property->response_topic_len == 0)
        {
            ESP_LOGE(TAG, "Missing response topic for %s", categoryString);
            return;
        }

        // Step 5: Fill MqttIncomingCmd structure (Requirement 2.4 - copy to local buffer)
        MqttIncomingCmd cmd;
        memset(&cmd, 0, sizeof(cmd));
        
        // Copy original topic (re-copy since strtok_r modified localTopicBuffer)
        size_t topicCopyLen = ((size_t)event->topic_len < MQTT_CMD_MAX_TOPIC_LEN - 1) 
                              ? (size_t)event->topic_len 
                              : MQTT_CMD_MAX_TOPIC_LEN - 1;
        memcpy(cmd.topic, event->topic, topicCopyLen);
        cmd.topic[topicCopyLen] = '\0';
        cmd.topicLen = (uint16_t)topicCopyLen;
        
        // Copy payload
        size_t payloadCopyLen = ((size_t)event->data_len < MQTT_CMD_MAX_PAYLOAD_LEN - 1) 
                                ? (size_t)event->data_len 
                                : MQTT_CMD_MAX_PAYLOAD_LEN - 1;
        memcpy(cmd.payload, event->data, payloadCopyLen);
        cmd.payload[payloadCopyLen] = '\0';
        cmd.payloadLen = (uint16_t)payloadCopyLen;
        
        // Copy response topic
        size_t respTopicCopyLen = ((size_t)event->property->response_topic_len < MQTT_CMD_MAX_RESPONSE_TOPIC - 1) 
                                  ? (size_t)event->property->response_topic_len 
                                  : MQTT_CMD_MAX_RESPONSE_TOPIC - 1;
        memcpy(cmd.responseTopic, event->property->response_topic, respTopicCopyLen);
        cmd.responseTopic[respTopicCopyLen] = '\0';
        cmd.responseTopicLen = (uint16_t)respTopicCopyLen;
        
        // Set metadata
        cmd.category = topicCategory;
        cmd.source = messageSource;
        cmd.receivedTimestampUs = esp_timer_get_time();

        ESP_LOGD(TAG, "Queuing %s from %s: topic=%s, responseTopic=%s",
                 topic_category_to_string(topicCategory),
                 message_source_to_string(messageSource),
                 cmd.topic, cmd.responseTopic);

        // Step 6: Queue command and return immediately (Requirement 3.1)
        if (!MqttCommandProcessor::queueIncomingCommand(&cmd))
        {
            ESP_LOGW(TAG, "Failed to queue incoming %s (queue full)", 
                     topic_category_to_string(topicCategory));
        }
        else
        {
            ESP_LOGD(TAG, "Queued %s successfully", topic_category_to_string(topicCategory));
        }
        
        // Return immediately - no callbacks, no mutex, no publish (Requirements 3.4, 3.5, 3.6)
    }
    break;
    case MQTT_EVENT_ERROR:
    {
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGE(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGE(TAG, "Last error code reported from tls stack: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGE(TAG, "Last error captured during transport connection: %s", esp_err_to_name(event->error_handle->esp_transport_sock_errno));
        }
        else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
        {
            ESP_LOGE(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        }
        else
        {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
    }
    break;
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
        break;
    case MQTT_USER_EVENT:
        ESP_LOGI(TAG, "MQTT_USER_EVENT");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%" PRId32, event_id);
        break;
    }
}

MQTT5Manager::MQTT5Manager()
{
    this->connectedToBroker = false;
    this->mqttEventGroupHandle = xEventGroupCreate();
    this->mqtt5ClientHandle = NULL;
    this->onDataReceivedListener = NULL;
    this->onDeviceCommandReceivedListener = NULL;
    this->onDeviceRequestReceivedListener = NULL;
    this->onMQTTBrokerConnectedListener = NULL;
    this->onMQTTBrokerDisconnectedListener = NULL;
    // Note: Rate limiting is now handled by MqttCommandProcessor
}
bool MQTT5Manager::connect(const char *deviceID, const char *url, const char *userID, const char *password)
{
    if (this->mqttEventGroupHandle == NULL)
    {
        // Event group may have been deleted in a previous disconnect().
        this->mqttEventGroupHandle = xEventGroupCreate();
        if (this->mqttEventGroupHandle == NULL)
        {
            ESP_LOGE(TAG, "Failed to (re)create MQTT event group!");
            return false;
        }
    }

    // Check if the deviceID is not null
    if (url == NULL || strlen(deviceID) == 0)
    {
        ESP_LOGE(TAG, "Invalid Device ID !");
        return false;
    }

    // Step 1. validate url user and password
    if (url == NULL || userID == NULL || password == NULL || strlen(url) == 0 || strlen(userID) == 0 || strlen(password) == 0)
    {
        ESP_LOGE(TAG, "Invalid url, userID or password !");
        return false;
    }

    char lastWillMessage[50];
    sprintf(lastWillMessage, "Device %s disconnected", deviceID);

    esp_mqtt_client_config_t mqtt5Configuration = {}; // Zero-initialize all fields
    mqtt5Configuration.broker.address.uri = url;
#if USE_DEVELOPMENT_SERVER == 0
    mqtt5Configuration.broker.verification.certificate = (const char *)broker_pem_start;
    mqtt5Configuration.broker.verification.certificate_len = (size_t)(broker_pem_end - broker_pem_start);
#endif
    mqtt5Configuration.credentials.username = userID;
    mqtt5Configuration.credentials.client_id = deviceID;
    mqtt5Configuration.credentials.authentication.password = password;
    mqtt5Configuration.session.last_will.topic = MQTT_CONFIG_LAST_WILL_TOPIC;
    mqtt5Configuration.session.last_will.msg = "{\"type\":1,\"verbosity\":3}";
    mqtt5Configuration.session.last_will.qos = MQTT_CONFIG_LAST_WILL_QOS;
    mqtt5Configuration.session.last_will.retain = MQTT_CONFIG_LAST_WILL_RETAIN;
    mqtt5Configuration.session.keepalive = MQTT_CONFIG_KEEP_ALIVE_INTERVAL_SECONDS;
    mqtt5Configuration.session.protocol_ver = MQTT_PROTOCOL_V_5;
    mqtt5Configuration.network.reconnect_timeout_ms = MQTT_RECONNECT_TIMER_PERIOD_MS;
    mqtt5Configuration.network.timeout_ms = MQTT_CONFIG_NETWORK_TIMEOUT_MS;
    mqtt5Configuration.network.disable_auto_reconnect = false;
    mqtt5Configuration.task.priority = MQTT_CONFIG_TASK_PRIORITY;
    mqtt5Configuration.task.stack_size = MQTT_CONFIG_TASK_STACK_DEPTH;
    mqtt5Configuration.buffer.size = MQTT_CONFIG_INPUT_BUFFER_SIZE;
    mqtt5Configuration.buffer.out_size = MQTT_CONFIG_OUTPUT_BUFFER_SIZE;

    this->mqtt5ClientHandle = esp_mqtt_client_init(&mqtt5Configuration);
    if (this->mqtt5ClientHandle == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return false;
    }

    /* Set connection properties and user properties */
    // esp_mqtt5_client_set_user_property(&connect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    // esp_mqtt5_client_set_user_property(&connect_property.will_user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    // esp_mqtt5_client_set_connect_property(client, &connect_property);

    /* If you call esp_mqtt5_client_set_user_property to set user properties, DO NOT forget to delete them.
     * esp_mqtt5_client_set_connect_property will malloc buffer to store the user_property and you can delete it after
     */
    // esp_mqtt5_client_delete_user_property(connect_property.user_property);
    // esp_mqtt5_client_delete_user_property(connect_property.will_user_property);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(this->mqtt5ClientHandle, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt5_event_handler, this);
    esp_mqtt_client_start(this->mqtt5ClientHandle);

    // Step 4. wait till the connect success or failed bits are set
    EventBits_t waitBits = xEventGroupWaitBits(
        this->mqttEventGroupHandle,
        MQTT_BROKER_CONNECTED_BIT | MQTT_BROKER_CONNECT_FAILED_BIT,
        pdTRUE,
        pdFALSE,
        portMAX_DELAY);

    // Step 5. based on the bits set, return true / false.
    if ((waitBits & MQTT_BROKER_CONNECT_FAILED_BIT) != 0)
    {
        // Failed to connect to the Access Point
        ESP_LOGE(TAG, "Failed to connect to the mqtt broker");
        return false;
    }
    if ((waitBits & MQTT_BROKER_CONNECTED_BIT) != 0)
    {
        ESP_LOGI(TAG, "Connected to mqtt broker");
        return true;
    }

    return true;
}

bool MQTT5Manager::disconnect()
{
    if (this->mqtt5ClientHandle == NULL)
    {
        ESP_LOGI(TAG, "MQTT client already disconnected/not initialized.");
        return true;
    }

    esp_mqtt_client_disconnect(this->mqtt5ClientHandle);

    esp_err_t ret = esp_mqtt_client_stop(this->mqtt5ClientHandle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to stop mqtt client %s", esp_err_to_name(ret));
    }

    ret = esp_mqtt_client_destroy(this->mqtt5ClientHandle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to destroy mqtt client %s", esp_err_to_name(ret));
    }

    // // Wait for the disconnect event
    // EventBits_t waitBits = xEventGroupWaitBits(
    //     this->mqttEventGroupHandle,
    //     MQTT_BROKER_DISCONNECTED_BIT,
    //     pdTRUE,
    //     pdFALSE,
    //     pdMS_TO_TICKS(portMAX_DELAY) // Wait indefinitely
    // );

    // if ((waitBits & MQTT_BROKER_DISCONNECTED_BIT) == 0)
    // {
    //     ESP_LOGW(TAG, "Timed out waiting for MQTT disconnect confirmation.");
    //     // Even on timeout, proceed with cleanup
    // }

    // Delete the event group to free up resources
    if (this->mqttEventGroupHandle != NULL)
    {
        vEventGroupDelete(this->mqttEventGroupHandle);
        this->mqttEventGroupHandle = NULL;
    }

    this->mqtt5ClientHandle = NULL;
    this->connectedToBroker = false;

    ESP_LOGI(TAG, "Disconnected from the mqtt broker and client destroyed.");

    return true;
}

/**
 * @brief Publishes an MQTT5Message to the broker. Always frees the message struct and its fields.
 *
 * @param mqttMessage Pointer to a dynamically allocated MQTT5Message. Ownership is always taken and the struct is freed (even on failure).
 * @return true if published successfully, false otherwise.
 */
bool MQTT5Manager::publish(MQTT5Message *mqttMessage)
{
    // Step 1. Check if the message is not null
    if (mqttMessage == NULL)
    {
        ESP_LOGE(TAG, "mqtt message cannot be null");
        // freeMqttMessage(mqttMessage);
        return false;
    }

    if (this->connectedToBroker == false)
    {
        ESP_LOGW(TAG, "Not connected to the broker");
        freeMqttMessage(mqttMessage);
        return false;
    }

    // // Step 2. validate the fields in the mqttMessage.
    // if (mqttMessage->responseTopic == NULL || strlen(mqttMessage->responseTopic) == 0)
    // {
    //     ESP_LOGE(TAG, " responseTopic not specified ( is null ) ");
    //     freeMqttMessage(mqttMessage);
    //     return false;
    // }

    if (mqttMessage->topic == NULL || strlen(mqttMessage->topic) == 0)
    {
        ESP_LOGE(TAG, "topic not specified ( is null ) ");
        freeMqttMessage(mqttMessage);
        return false;
    }

    // Validate topic length against MAX_MQTT_TOPIC_LENGTH constraint
    size_t topicLength = strlen(mqttMessage->topic);
    if (topicLength >= MAX_MQTT_TOPIC_LENGTH)
    {
        ESP_LOGE(TAG, "Topic length (%zu) exceeds maximum allowed length (%d): %s",
                 topicLength, MAX_MQTT_TOPIC_LENGTH, mqttMessage->topic);
        freeMqttMessage(mqttMessage);
        return false;
    }

    if (mqttMessage->message == NULL || mqttMessage->contentType == NULL || strlen(mqttMessage->message) == 0 || strlen(mqttMessage->contentType) == 0)
    {
        ESP_LOGE(TAG, "message or content-type not specified.");
        freeMqttMessage(mqttMessage);
        return false;
    }

    if (mqttMessage->deviceID == NULL || strlen(mqttMessage->deviceID) == 0)
    {
        ESP_LOGE(TAG, "deviceID not specified (is null)");
        freeMqttMessage(mqttMessage);
        return false;
    }

    char *correlationData = NULL;
    size_t correlationDataLength = 0;
    getCorrelationData(correlationData, correlationDataLength);

    // Step 3. configure the mqtt packet and send the payload.
    esp_mqtt5_publish_property_config_t publishProperty = {
        .payload_format_indicator = 1,
        .message_expiry_interval = 1000,
        .topic_alias = 0,
        .response_topic = mqttMessage->responseTopic,
        .correlation_data = correlationData,
        .correlation_data_len = (uint16_t)correlationDataLength,
        .content_type = mqttMessage->contentType,
        .user_property = NULL};

    esp_err_t ret = esp_mqtt5_client_set_publish_property(this->mqtt5ClientHandle, &publishProperty);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to publish message to mqtt topic %s", esp_err_to_name(ret));
        freeMqttMessage(mqttMessage);
        if (correlationData != NULL) {
            free(correlationData);  // Free correlation data to prevent memory leak
        }
        return false;
    }
    // ESP_LOGI(TAG, "---------------------------------------------------------------------");
    // ESP_LOGI(TAG, "Topic : %s", mqttMessage->topic);
    // ESP_LOGI(TAG, "Reponse Topic : %s ", mqttMessage->responseTopic);
    // ESP_LOGI(TAG, "Correlation Data : %s ", correlationData);
    // ESP_LOGI(TAG, "deviceID : %s ", mqttMessage->deviceID);
    // ESP_LOGI(TAG, "---------------------------------------------------------------------");

#if (MQTT_CONFIG_ENQUEUE_MESSAGES == true)
    int messageID = esp_mqtt_client_enqueue(
        this->mqtt5ClientHandle,
        mqttMessage->topic,
        mqttMessage->message,
        0,
        mqttMessage->qos,
        mqttMessage->retain == true ? 1 : 0,
        true);
#else
    int messageID = esp_mqtt_client_publish(
        this->mqtt5ClientHandle,
        mqttMessage->topic,
        mqttMessage->message,
        0,
        mqttMessage->qos,
        mqttMessage->retain == true ? 1 : 0);
#endif

    // Store topic and message for logging before freeing
    char topicCopy[MAX_MQTT_TOPIC_LENGTH];
    strncpy(topicCopy, mqttMessage->topic, MAX_MQTT_TOPIC_LENGTH - 1);
    topicCopy[MAX_MQTT_TOPIC_LENGTH - 1] = '\0';
    
    // Truncate message for logging if too long (max 200 chars)
    const size_t maxLogLen = 200;
    size_t msgLen = strlen(mqttMessage->message);
    char msgPreview[maxLogLen + 4]; // +4 for "..." and null terminator
    if (msgLen > maxLogLen) {
        strncpy(msgPreview, mqttMessage->message, maxLogLen);
        msgPreview[maxLogLen] = '\0';
        strcat(msgPreview, "...");
    } else {
        strncpy(msgPreview, mqttMessage->message, maxLogLen);
        msgPreview[msgLen] = '\0';
    }
    
    int qosLevel = mqttMessage->qos;
    
    freeMqttMessage(mqttMessage);
    if (correlationData != NULL)
        free(correlationData);
    if (messageID == -1)
    {
        ESP_LOGE(TAG, "Failed to publish message !");
        return false;
    }

    // CRITICAL: Reduced logging verbosity to prevent UART TX buffer blocking
    // During heavy packet loss, verbose logging can block for >5 seconds
    // Removed data payload from log to reduce UART traffic
    // Using WARNING level to make MQTT publishes stand out in logs
    ESP_LOGW(TAG, "MQTT pub: %s, ID: %d, QoS: %d", topicCopy, messageID, qosLevel);

    return true;
}

bool MQTT5Manager::isConnected()
{
    // return the boolean variable that is updated in the event handlers.
    return this->connectedToBroker;
}

bool MQTT5Manager::subscribe(char *topic, QualityOfService qos)
{
    // Step 1. validate the topic name
    if (topic == NULL || strlen(topic) == 0)
    {
        ESP_LOGE(TAG, "Topic name cannot be null or empty ");
        return false;
    }

    // Validate topic length against MAX_MQTT_TOPIC_LENGTH constraint
    size_t topicLength = strlen(topic);
    if (topicLength >= MAX_MQTT_TOPIC_LENGTH)
    {
        ESP_LOGE(TAG, "Subscription topic length (%zu) exceeds maximum allowed length (%d): %s",
                 topicLength, MAX_MQTT_TOPIC_LENGTH, topic);
        return false;
    }

    // Step 2. subscribe to the topic.
    // QoS 2 (Exactly Once Delivery) is used for server-to-device subscriptions to ensure:
    // - Critical commands are delivered exactly once without duplicates
    // - Commands like pump control, configuration changes are not executed multiple times
    // - Provides the highest level of delivery assurance with 4-way handshake
    // - Essential for operations that must not be repeated (e.g., pump start/stop commands)
    // - Prevents potential system damage from duplicate command execution
    int messageID = esp_mqtt_client_subscribe(this->mqtt5ClientHandle, topic, qos);

    // Step 3. based on the message id returned by the above call, return true / false.
    if (messageID == -1)
    {
        ESP_LOGE(TAG, "Failed to subscribe to topic %s", topic);
        return false;
    }

    return true;
}

bool MQTT5Manager::unsubscribe(char *topic)
{
    // Step 1. Send a message for subcription

    // Step 2. wait till the diconnect success or failed bits are set

    // Step 3. based on the bits set, return true / false.
    return false;
}

// This functions takes the pointer to a buffer
// generates a uin32_t random number and then prints the hex representation of
bool MQTT5Manager::getCorrelationData(char *&correlationDataBuffer, size_t &length)
{
    // Step 1. Check if the buffer holder is not null
    if (correlationDataBuffer != NULL)
    {
        ESP_LOGE(TAG, "%s pointer to buffer, passed to this function shall be null", __func__);
        return false;
    }

    // Step 2. Allocate memory of 10 bytes
    correlationDataBuffer = (char *)malloc(CORRELATION_DATA_BUFFER_SIZE);
    if (correlationDataBuffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for correlation data buffer (%d bytes)", 
                 CORRELATION_DATA_BUFFER_SIZE);
        length = 0;
        return false;
    }

    // Step 3. Generate a uint32_t number and sprintf this number to buffer as hex without the 0x prefix
    uint32_t randomNumber = esp_random();
    // Step 4. store the strlen of the string inside length variable and return true;
    length = snprintf(correlationDataBuffer, CORRELATION_DATA_BUFFER_SIZE, "%08lx", randomNumber);
    return true;
}

void MQTT5Manager::setOnDataReceivedListener(function<void(MQTT5Message *)> onDataReceivedListener)
{
    this->onDataReceivedListener = onDataReceivedListener;
}
void MQTT5Manager::setOnDeviceCommandReceivedListener(function<bool(DeviceCommandType, char *, uint32_t *value, uint8_t valueType)> onDeviceCommandReceivedListener)
{
    this->onDeviceCommandReceivedListener = onDeviceCommandReceivedListener;
}
void MQTT5Manager::setOnDeviceRequestReceivedListener(function<bool(DeviceRequestType, char *, char *&)> onDeviceRequestReceivedListener)
{
    this->onDeviceRequestReceivedListener = onDeviceRequestReceivedListener;
}
void MQTT5Manager::setOnMQTTBrokerConnectedListener(function<void()> onMQTTBrokerConnectedListener)
{
    this->onMQTTBrokerConnectedListener = onMQTTBrokerConnectedListener;
}
void MQTT5Manager::setOnMQTTBrokerDisconnectedListener(function<void()> onMQTTBrokerDisconnectedListener)
{
    this->onMQTTBrokerDisconnectedListener = onMQTTBrokerDisconnectedListener;
}

void MQTT5Manager::reconnect()
{
    if (!this->isConnected() && this->mqtt5ClientHandle != NULL)
    {
        ESP_LOGI(TAG, "Stopping client before reconnection attempt...");
        esp_err_t err = esp_mqtt_client_stop(this->mqtt5ClientHandle);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to stop MQTT client before reconnection (this may be expected if already stopped): %s", esp_err_to_name(err));
        }

        ESP_LOGI(TAG, "Attempting to restart MQTT client for reconnection...");
        err = esp_mqtt_client_start(this->mqtt5ClientHandle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start MQTT client for reconnection: %s", esp_err_to_name(err));
        }
    }
}

/**
 * @brief Publish lockout feature state change event
 *
 * Publishes an event when the lockout feature is enabled or disabled.
 * Uses QoS 1 (at-least-once delivery) and retained flag to ensure
 * subscribers receive the latest state even if they connect later.
 *
 * Topic format: src-dev/{deviceId}/event
 * Payload format: {"event":"lockout_feature_state_change","enabled":true/false,"changed_by":"source","timestamp":1234567890}
 *
 * @param deviceId Device identifier (MAC-based)
 * @param enabled New lockout feature state (true=enabled, false=disabled)
 * @param changedBy Source of the change (e.g., "mqtt_command", "local_button", "system")
 * @return true if message was queued successfully, false otherwise
 */
bool MQTT5Manager::publishLockoutFeatureStateChange(const char* deviceId, bool enabled, const char* changedBy)
{
    if (deviceId == NULL || strlen(deviceId) == 0) {
        ESP_LOGE(TAG, "publishLockoutFeatureStateChange: deviceId is NULL or empty");
        return false;
    }

    if (changedBy == NULL || strlen(changedBy) == 0) {
        ESP_LOGE(TAG, "publishLockoutFeatureStateChange: changedBy is NULL or empty");
        return false;
    }

    if (!this->connectedToBroker) {
        ESP_LOGW(TAG, "publishLockoutFeatureStateChange: Not connected to broker");
        return false;
    }

    // Build JSON payload
    cJSON* root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "publishLockoutFeatureStateChange: Failed to create JSON object");
        return false;
    }

    cJSON_AddStringToObject(root, "event", "lockout_feature_state_change");
    cJSON_AddBoolToObject(root, "enabled", enabled);
    cJSON_AddStringToObject(root, "changed_by", changedBy);
    cJSON_AddNumberToObject(root, "timestamp", (double)time(NULL));

    char* jsonString = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (jsonString == NULL) {
        ESP_LOGE(TAG, "publishLockoutFeatureStateChange: Failed to serialize JSON");
        return false;
    }

    // Build MQTT5Message
    MQTT5Message* message = (MQTT5Message*)malloc(sizeof(MQTT5Message));
    if (message == NULL) {
        ESP_LOGE(TAG, "publishLockoutFeatureStateChange: Failed to allocate MQTT5Message");
        cJSON_free(jsonString);
        return false;
    }
    memset(message, 0, sizeof(MQTT5Message));

    // Allocate and set topic: src-dev/{deviceId}/event
    size_t topicLen = strlen("src-dev/") + strlen(deviceId) + strlen("/event") + 1;
    message->topic = (char*)malloc(topicLen);
    if (message->topic == NULL) {
        ESP_LOGE(TAG, "publishLockoutFeatureStateChange: Failed to allocate topic");
        free(message);
        cJSON_free(jsonString);
        return false;
    }
    snprintf(message->topic, topicLen, "src-dev/%s/event", deviceId);

    // Validate topic length
    if (strlen(message->topic) >= MAX_MQTT_TOPIC_LENGTH) {
        ESP_LOGE(TAG, "publishLockoutFeatureStateChange: Topic too long (%zu >= %d)",
                 strlen(message->topic), MAX_MQTT_TOPIC_LENGTH);
        free(message->topic);
        free(message);
        cJSON_free(jsonString);
        return false;
    }

    // Set message payload (transfer ownership from cJSON)
    message->message = jsonString;

    // Allocate and set content type
    const char* contentTypeStr = "application/json";
    message->contentType = (char*)malloc(strlen(contentTypeStr) + 1);
    if (message->contentType == NULL) {
        ESP_LOGE(TAG, "publishLockoutFeatureStateChange: Failed to allocate contentType");
        free(message->topic);
        free(message);
        cJSON_free(jsonString);
        return false;
    }
    strcpy(message->contentType, contentTypeStr);

    // Allocate and set device ID
    message->deviceID = (char*)malloc(strlen(deviceId) + 1);
    if (message->deviceID == NULL) {
        ESP_LOGE(TAG, "publishLockoutFeatureStateChange: Failed to allocate deviceID");
        free(message->contentType);
        free(message->topic);
        free(message);
        cJSON_free(jsonString);
        return false;
    }
    strcpy(message->deviceID, deviceId);

    // Set QoS 1 (at-least-once delivery) and retained flag
    message->qos = QOS_1;
    message->retain = true;

    // Response topic not used for events
    message->responseTopic = NULL;

    ESP_LOGI(TAG, "Publishing lockout feature state change: enabled=%s, changed_by=%s",
             enabled ? "true" : "false", changedBy);

    // Publish (takes ownership of message and frees it)
    bool success = this->publish(message);

    if (!success) {
        ESP_LOGE(TAG, "publishLockoutFeatureStateChange: Failed to publish");
    }

    return success;
}

/**
 * @brief Publish lockout configuration response
 *
 * Publishes complete lockout configuration in response to a query request.
 * Uses QoS 1 (at-least-once delivery) but NOT retained (it's a response, not state).
 *
 * Topic: Provided as parameter (response topic from the request)
 * Payload format: {"lockout_feature_enabled":true,"lockout_duration_ms":1800000,
 *                  "lockout_currently_active":false,"lockout_trigger_time":0,
 *                  "lockout_clear_time":0,"timestamp":1234567890}
 *
 * @param deviceId Device identifier (MAC-based)
 * @param responseTopic Topic to publish response to (from request)
 * @param lockoutFeatureEnabled Is lockout feature enabled?
 * @param lockoutDurationMs Configured lockout duration in milliseconds
 * @param lockoutCurrentlyActive Is pump currently in lockout state?
 * @param lockoutTriggerTime Unix timestamp when lockout was triggered (0 if not active)
 * @param lockoutClearTime Unix timestamp when lockout was cleared (0 if not cleared)
 * @param timestamp Current device time (Unix timestamp)
 * @return true if message was queued successfully, false otherwise
 */
bool MQTT5Manager::publishLockoutConfiguration(const char* deviceId, const char* responseTopic,
                                               bool lockoutFeatureEnabled, uint32_t lockoutDurationMs,
                                               bool lockoutCurrentlyActive, uint32_t lockoutTriggerTime,
                                               uint32_t lockoutClearTime, uint32_t timestamp)
{
    if (deviceId == NULL || strlen(deviceId) == 0) {
        ESP_LOGE(TAG, "publishLockoutConfiguration: deviceId is NULL or empty");
        return false;
    }

    if (responseTopic == NULL || strlen(responseTopic) == 0) {
        ESP_LOGE(TAG, "publishLockoutConfiguration: responseTopic is NULL or empty");
        return false;
    }

    if (!this->connectedToBroker) {
        ESP_LOGW(TAG, "publishLockoutConfiguration: Not connected to broker");
        return false;
    }

    // Build JSON payload
    cJSON* root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "publishLockoutConfiguration: Failed to create JSON object");
        return false;
    }

    cJSON_AddBoolToObject(root, "lockout_feature_enabled", lockoutFeatureEnabled);
    cJSON_AddNumberToObject(root, "lockout_duration_ms", (double)lockoutDurationMs);
    cJSON_AddBoolToObject(root, "lockout_currently_active", lockoutCurrentlyActive);
    cJSON_AddNumberToObject(root, "lockout_trigger_time", (double)lockoutTriggerTime);
    cJSON_AddNumberToObject(root, "lockout_clear_time", (double)lockoutClearTime);
    cJSON_AddNumberToObject(root, "timestamp", (double)timestamp);

    char* jsonString = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (jsonString == NULL) {
        ESP_LOGE(TAG, "publishLockoutConfiguration: Failed to serialize JSON");
        return false;
    }

    // Build MQTT5Message
    MQTT5Message* message = (MQTT5Message*)malloc(sizeof(MQTT5Message));
    if (message == NULL) {
        ESP_LOGE(TAG, "publishLockoutConfiguration: Failed to allocate MQTT5Message");
        cJSON_free(jsonString);
        return false;
    }
    memset(message, 0, sizeof(MQTT5Message));

    // Allocate and set topic (use provided response topic)
    message->topic = (char*)malloc(strlen(responseTopic) + 1);
    if (message->topic == NULL) {
        ESP_LOGE(TAG, "publishLockoutConfiguration: Failed to allocate topic");
        free(message);
        cJSON_free(jsonString);
        return false;
    }
    strcpy(message->topic, responseTopic);

    // Validate topic length
    if (strlen(message->topic) >= MAX_MQTT_TOPIC_LENGTH) {
        ESP_LOGE(TAG, "publishLockoutConfiguration: Topic too long (%zu >= %d)",
                 strlen(message->topic), MAX_MQTT_TOPIC_LENGTH);
        free(message->topic);
        free(message);
        cJSON_free(jsonString);
        return false;
    }

    // Set message payload (transfer ownership from cJSON)
    message->message = jsonString;

    // Allocate and set content type
    const char* contentTypeStr = "application/json";
    message->contentType = (char*)malloc(strlen(contentTypeStr) + 1);
    if (message->contentType == NULL) {
        ESP_LOGE(TAG, "publishLockoutConfiguration: Failed to allocate contentType");
        free(message->topic);
        free(message);
        cJSON_free(jsonString);
        return false;
    }
    strcpy(message->contentType, contentTypeStr);

    // Allocate and set device ID
    message->deviceID = (char*)malloc(strlen(deviceId) + 1);
    if (message->deviceID == NULL) {
        ESP_LOGE(TAG, "publishLockoutConfiguration: Failed to allocate deviceID");
        free(message->contentType);
        free(message->topic);
        free(message);
        cJSON_free(jsonString);
        return false;
    }
    strcpy(message->deviceID, deviceId);

    // Set QoS 1 (at-least-once delivery) but NOT retained (it's a response)
    message->qos = QOS_1;
    message->retain = false;

    // Response topic not used for responses (already in topic field)
    message->responseTopic = NULL;

    ESP_LOGI(TAG, "Publishing lockout configuration: feature_enabled=%s, duration_ms=%lu, currently_active=%s",
             lockoutFeatureEnabled ? "true" : "false", lockoutDurationMs,
             lockoutCurrentlyActive ? "true" : "false");

    // Publish (takes ownership of message and frees it)
    bool success = this->publish(message);

    if (!success) {
        ESP_LOGE(TAG, "publishLockoutConfiguration: Failed to publish");
    }

    return success;
}

MQTT5Manager::~MQTT5Manager()
{
    if (this->mqtt5ClientHandle != NULL)
    {
        esp_mqtt_client_destroy(this->mqtt5ClientHandle);
        this->mqtt5ClientHandle = NULL;
    }
    if (this->mqttEventGroupHandle != NULL)
    {
        vEventGroupDelete(this->mqttEventGroupHandle);
        this->mqttEventGroupHandle = NULL;
    }
    // Note: Rate limiting is now handled by MqttCommandProcessor
}
