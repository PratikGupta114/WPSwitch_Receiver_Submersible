#ifndef MQTTMANAGER_H_
#define MQTTMANAGER_H_

/*
 * MQTT Quality of Service (QoS) Strategy Documentation
 * ==================================================
 *
 * This system implements a carefully designed QoS strategy to ensure reliable
 * communication while optimizing performance and preventing duplicate operations:
 *
 * Device-to-Server Communication (QoS 1 - At Least Once):
 * - Sensor data publishing (water level measurements)
 * - Event publishing (pump state changes, system alerts)
 * - Request responses (device info, configuration responses)
 * - Command acknowledgments (pump control confirmations)
 *
 * Server-to-Device Communication (QoS 2 - Exactly Once):
 * - Command subscriptions (pump control commands)
 * - Request subscriptions (configuration requests, device info requests)
 *
 * Topic Length Constraints:
 * - Maximum topic length: 45 characters (MAX_MQTT_TOPIC_LENGTH)
 * - Validation implemented in all topic generation functions
 * - Error logging when limits are exceeded
 * - Warning logging when approaching 80% of limit
 */

#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "mqtt_client.h"
#include "mqtt5_client.h"
#include "sdkconfig.h"
#include "../secrets.h"
#include "../data/dataTypes.h"

using namespace std;

#define MQTT_CONFIG_TASK_STACK_DEPTH CONFIG_MQTT_TASK_STACK_DEPTH
#define MQTT_CONFIG_TASK_PRIORITY CONFIG_WPSWITCH_MQTT_TASK_PRIORITY

// 8883 for TLS secured MQTT connections
// 1883 for unsecure MQTT connections
// 8084 for WebSocket connections with TLS / SSL
#if USE_DEVELOPMENT_SERVER == 1
#define MQTT_BROKER_CREDENTIAL_USERNAME MQTT_BROKER_USERNAME_SECRET_DEV
#define MQTT_BROKER_CREDENTIAL_PASSWORD MQTT_BROKER_PASSWORD_SECRET_DEV
#define MQTT_BROKER_URL CONFIG_MQTT_BROKER_URL_DEV
#define MQTT_BROKER_PORT CONFIG_MQTT_BROKER_PORT_DEV
#else
#define MQTT_BROKER_CREDENTIAL_USERNAME MQTT_BROKER_USERNAME_SECRET
#define MQTT_BROKER_CREDENTIAL_PASSWORD MQTT_BROKER_PASSWORD_SECRET
#define MQTT_BROKER_URL CONFIG_MQTT_BROKER_URL
#define MQTT_BROKER_PORT CONFIG_MQTT_BROKER_PORT
#endif

// #define MQTT_BROKER_URL "mqtt://jetson-server.local:1883"
// #define MQTT_BROKER_URL "mqtt://192.168.1.107:1883"
// #define MQTT_BROKER_URL "mqtt://192.168.0.112:1883"
// #define MQTT_BROKER_URL "mqtt://192.168.43.132:1883"

// #define MQTT_BROKER_URL "mqtt://192.168.1.110:1883"

// MQTT 5 Connection Properties
#define MQTT_PROPERTY_SESSION_EXPIRY_INTERVAL CONFIG_MQTT_SESSION_EXPIRY_INTERVAL
#define MQTT_PROPERTY_MAXIMUM_PACKET_SIZE CONFIG_MQTT_MAXIMUM_PACKET_SIZE
#define MQTT_PROPERTY_RECEIVE_MAXIMUM 65535
#define MQTT_PROPERTY_SHOULD_REQUEST_RESP_INFO true
#define MQTT_PROPERTY_SHOULD_REQUEST_PROBLEM_INFO true
#define MQTT_PROPERTY_WILL_DELAY_INTERVAL CONFIG_MQTT_WILL_DELAY_INTERVAL
#define MQTT_PROPERTY_PAYLOAD_FORMAT_INDICATOR true
#define MQTT_PROPERTY_MESSAGE_EXPIRY_INTERVAL CONFIG_MQTT_MESSAGE_EXPIRY_INTERVAL

// MQTT 5 Client Configurations
#define MQTT_CONFIG_DISABLE_AUTO_RECONNECT false
#define MQTT_CONFIG_LAST_WILL_TOPIC "/topic/will"
#define MQTT_CONFIG_LAST_WILL_MSG "I will disconnect"
#define MQTT_CONFIG_LAST_WILL_MSG_LEN 17
#define MQTT_CONFIG_LAST_WILL_QOS QOS_1
#define MQTT_CONFIG_LAST_WILL_RETAIN false

#define MQTT_CONFIG_DISABLE_CLEAN_SESSION false
#define MQTT_CONFIG_DISABLE_KEEP_ALIVE false
#define MQTT_CONFIG_KEEP_ALIVE_INTERVAL_SECONDS CONFIG_MQTT_KEEP_ALIVE_INTERVAL_SECONDS

#define MQTT_CONFIG_MESSAGE_RETRANSMISSION_PERIOD CONFIG_MQTT_MESSAGE_RETRANSMISSION_PERIOD
#define MQTT_CONFIG_ENQUEUE_MESSAGES true
#define MQTT_CONFIG_INPUT_BUFFER_SIZE CONFIG_MQTT_INPUT_BUFFER_SIZE
#define MQTT_CONFIG_OUTPUT_BUFFER_SIZE CONFIG_MQTT_OUTPUT_BUFFER_SIZE

// Maximum MQTT topic length constraint (45 characters)
// This limit ensures:
// - Compatibility with MQTT brokers that may have topic length restrictions
// - Prevention of buffer overflows in topic string operations
// - Consistent topic naming across the system
// - Efficient memory usage for topic string storage
//
// Topic length validation is implemented in:
// - MQTT5Manager::publish() - validates outgoing message topics
// - MQTT5Manager::subscribe() - validates subscription topics
// - publishData() - validates generated topic and response topic strings
// - mqttEventPublisher lambda - validates generated event topic strings
//
// When topic length exceeds this limit:
// - Error is logged with ESP_LOGE indicating the violation
// - Operation is rejected to prevent system instability
// - When approaching 80% of limit, warning is logged with ESP_LOGW
#define MAX_MQTT_TOPIC_LENGTH CONFIG_MQTT_MAX_TOPIC_LENGTH
#define MQTT_RECONNECT_TIMER_PERIOD_MS CONFIG_MQTT_RECONNECT_TIMER_PERIOD_MS
#define MQTT_CONFIG_NETWORK_TIMEOUT_MS CONFIG_MQTT_NETWORK_TIMEOUT_MS

// MQTT Rate Limiting Configuration
// Minimum interval between consecutive MQTT commands or requests to prevent flooding
// This protects the device from being overwhelmed by rapid command/request sequences
#define MQTT_RATE_LIMIT_INTERVAL_MS 2000 // 2 seconds between messages
#define MQTT_RATE_LIMIT_INTERVAL_US (MQTT_RATE_LIMIT_INTERVAL_MS * 1000) // Convert to microseconds

// MQTT Event Bits definitions
#define MQTT_BROKER_CONNECTED_BIT BIT0
#define MQTT_BROKER_DISCONNECTED_BIT BIT1

#define MQTT_BROKER_CONNECT_FAILED_BIT BIT6
#define MQTT_BROKER_DISCONNECT_FAILED_BIT BIT7

#define MQTT_RECONNECT_BIT BIT8

extern const char broker_pem_start[] asm("_binary_emqxsl_ca_crt_start");
extern const char broker_pem_end[] asm("_binary_emqxsl_ca_crt_end");

// MQTT Quality of Service levels used in this system:
//
// QOS_0: At most once delivery (fire and forget) - not used in this implementation
//        Messages may be lost but no duplicates occur
//
// QOS_1: At least once delivery - used for device-to-server publishing
//        Used for: sensor data, events, responses, command acknowledgments
//        Ensures messages reach the server but may have duplicates
//        Suitable for data where occasional duplicates can be handled by server logic
//
// QOS_2: Exactly once delivery - used for server-to-device subscriptions
//        Used for: commands, requests from server/control systems
//        Ensures critical commands are delivered exactly once without duplicates
//        Essential for commands that must not be executed multiple times
//        Provides highest level of delivery assurance but with increased overhead
typedef enum quality_of_service
{
    QOS_0 = 0, // At most once delivery (not used in this implementation)
    QOS_1 = 1, // At least once delivery (device-to-server publishing)
    QOS_2 = 2  // Exactly once delivery (server-to-device subscriptions)
} QualityOfService;

typedef struct mqtt_message
{
    // reference to the dynamically allocated string respresenting the mqtt topic
    char *topic;
    // Qualify of sevice for this message
    QualityOfService qos;
    // reference to the dynamically allocated string respresenting the mqtt response topic (if any)
    char *responseTopic;
    // reference to the dynamically allocated string respresenting the mimetype of the message content.
    char *contentType;
    // bool flag to configure whether the message should retain of not
    bool retain;
    // reference to the dynamically allocated string respresenting the mqtt message
    char *message;
    // reference to the dynamically allocated string representing the deviceID
    char *deviceID;

} MQTT5Message;

const char *topic_category_to_string(MQTTtopicCategories category);
const char *message_source_to_string(MQTTMessageSourceTypes messageSource);

class MQTT5Manager
{
    friend void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

private:
    bool connectedToBroker = false;
    EventGroupHandle_t mqttEventGroupHandle = NULL;

    esp_mqtt_client_handle_t mqtt5ClientHandle = NULL;

    // esp_mqtt5_user_property_item_t user_property_arr[] = {
    //     {"board", "esp32"},
    //     {"u", "user"},
    //     {"p", "password"}};

    function<void(MQTT5Message *)> onDataReceivedListener = NULL;
    function<bool(DeviceCommandType, char *, uint32_t *value, uint8_t valueType)> onDeviceCommandReceivedListener = NULL;
    function<bool(DeviceRequestType, char *, char *&)> onDeviceRequestReceivedListener = NULL;
    function<void()> onMQTTBrokerConnectedListener = NULL;
    function<void()> onMQTTBrokerDisconnectedListener = NULL;
    
    // char *mqttTopicCategoryEnumToString(MQTTtopicCategories category);
    // char *mqttMessageSourceTypesToString(MQTTMessageSourceTypes messageSource);

public:
    bool connect(const char *deviceID, const char *url, const char *userID, const char *password);
    bool subscribe(char *topic, QualityOfService qos);
    bool unsubscribe(char *topic);
    bool disconnect();
    bool publish(MQTT5Message *message);
    bool isConnected();
    void reconnect();
    bool getCorrelationData(char *&correlationDataBuffer, size_t &length);

    void setOnDataReceivedListener(function<void(MQTT5Message *)> onDataReceivedListener);
    void setOnDeviceCommandReceivedListener(function<bool(DeviceCommandType, char *, uint32_t *value, uint8_t valueType)> onDeviceCommandReceivedListener);
    void setOnDeviceRequestReceivedListener(function<bool(DeviceRequestType, char *, char *&)> onDeviceRequestReceivedListener);
    void setOnMQTTBrokerConnectedListener(function<void()> onMQTTBrokerConnectedListener);
    void setOnMQTTBrokerDisconnectedListener(function<void()> onMQTTBrokerDisconnectedListener);

    // Getter methods for callbacks (used by MqttCommandProcessor)
    function<bool(DeviceRequestType, char *, char *&)> getOnDeviceRequestReceivedListener() const { return onDeviceRequestReceivedListener; }
    function<bool(DeviceCommandType, char *, uint32_t *value, uint8_t valueType)> getOnDeviceCommandReceivedListener() const { return onDeviceCommandReceivedListener; }

    // Lockout feature state change publishing
    // Topic: {source}/{deviceId}/event
    // QoS: 1 (at-least-once), Retained: true
    // Publishes lockout feature state change events with timestamp and source
    bool publishLockoutFeatureStateChange(const char* deviceId, bool enabled, const char* changedBy);

    // Lockout configuration response publishing
    // Topic: provided as parameter (response topic from request)
    // QoS: 1 (at-least-once), Retained: false
    // Publishes complete lockout configuration in response to queries
    bool publishLockoutConfiguration(const char* deviceId, const char* responseTopic, 
                                     bool lockoutFeatureEnabled, uint32_t lockoutDurationMs,
                                     bool lockoutCurrentlyActive, uint32_t lockoutTriggerTime,
                                     uint32_t lockoutClearTime, uint32_t timestamp);

    MQTT5Manager();
    ~MQTT5Manager();
};

#endif
