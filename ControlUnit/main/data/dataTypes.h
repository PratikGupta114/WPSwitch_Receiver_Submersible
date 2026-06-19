#ifndef COMMANDTYPES_H_
#define COMMANDTYPES_H_

#include "stdint.h"

// RF Module Configuration Enums
typedef enum rf_mod_baudrate_t
{
    BAUD_9600 = 0,
    BAUD_19200 = 1,
    BAUD_38400 = 2,
    BAUD_57600 = 3,
    BAUD_115200 = 4,
    BAUD_MAX = 5,
} RFModuleBaudrate;

typedef enum rf_mod_carrier_t
{
    FREQ_865MHZ = 0,
    FREQ_865_5_MHZ = 3,
    FREQ_866MHZ = 6,
    FREQ_866_5_MHZ = 9,
    FREQ_869MHZ = 24,
    FREQ_MAX = 25,
} RFModuleCarrierFrequency;

typedef enum rf_mod_air_data_rate_t
{
    DATA_RATE_19_2_KBPS = 0,
    DATA_RATE_38_4_KBPS = 1,
    DATA_RATE_76_8_KBPS = 2,
    DATA_RATE_MAX = 3,
} RFModuleAirDataRate;

typedef enum rf_mod_power_transmit_level_t
{
    POWER_LEVEL_PLUS_17_DB = 0,
    POWER_LEVEL_PLUS_10_DB = 1,
    POWER_LEVEL_PLUS_4_DB = 2,
    POWER_LEVEL_MINUS_2_DB = 3,
    POWER_LEVEL_MINUS_8_DB = 4,
    POWER_LEVEL_MINUS_14_DB = 5,
    POWER_LEVEL_MINUS_20_DB = 6,
    POWER_LEVEL_MINUS_24_DB = 7,
    POWER_LEVEL_MAX = 8,
} RFModulePowerTransmitLevel;

typedef enum fr_mod_signal_strength_limit_t
{
    SIGNAL_STRENGTH_MINUS_100_DBM = 0,
    SIGNAL_STRENGTH_MINUS_90_DBM = 1,
    SIGNAL_STRENGTH_MINUS_80_DBM = 2,
    SIGNAL_STRENGTH_MINUS_70_DBM = 3,
    SIGNAL_STRENGTH_MINUS_60_DBM = 4,
    SIGNAL_STRENGTH_MINUS_50_DBM = 5,
    SIGNAL_STRENGTH_MINUS_40_DBM = 6,
    SIGNAL_STRENGTH_MINUS_30_DBM = 7,
    SIGNAL_STRENGTH_MAX = 8,
} RFModuleSignalStrengthLimit;

// Specifies the type of command that the receiver device sends to the transmitter device
typedef enum command_type
{
    COMMAND_TYPE_RESET_DEVICE = 0xA1,
    COMMAND_TYPE_UPDATE_PARAM,
    COMMAND_TYPE_UPDATE_SECURITY_KEYS,
} CommandType;

// Specifies the type of request that the receiver device sends to the transmitter device
typedef enum request_type
{
    REQUEST_TYPE_APP_INTO = 0xA5,
    REQUEST_TYPE_DEVICE_INFO,
} RequestType;

typedef enum param_type
{
    PARAM_TYPE_TEMPERATURE_COVARIANCE = 0xB4,
    PARAM_TYPE_HUMIDITY_COVARIANCE,
    PARAM_TYPE_DISTANCE_COVARIANCE,
    PARAM_TYPE_SENSOR_HEIGHT,
    PARAM_TYPE_MIN_LEVEL,
    PARAM_TYPE_MAX_LEVEL,
    PARAM_TYPE_WATER_TANK_AREA,
    PARAM_TYPE_WATER_TANK_TYPE,
} ParamType;

typedef enum key_type
{
    KEY_TYPE_DHM_KEY = 0xC4,
    KEY_TYPE_ENCRYPTION_KEY,
} KeyType;

typedef enum water_tank_type
{
    WATER_TANK_TYPE_CYNDRICAL = 0x54,
    WATER_TANK_TYPE_CUBOID = 0x45,
    WATER_TANK_TYPE_INVALID = 0x00,
} WaterTankType;

typedef enum data_type
{
    TYPE_SENSOR_DATA = 0x93,
    TYPE_RESPONSE_DATA,
    TYPE_REQUEST_DATA,
    TYPE_COMMAND_DATA,
} DataType;

typedef enum data_fetch_result
{
    DATA_FETCH_SUCCESS = 0,
    ERROR_DATA_EXPIRED = 0x3C,
    ERROR_INVALID_ARG,
    ERROR_TIMEOUT,  // Mutex acquisition timeout
} DataFetchResult;

static const char *paramTypeToString(ParamType paramType) __attribute__((unused));
static const char *paramTypeToString(ParamType paramType)
{
    switch (paramType)
    {
    case PARAM_TYPE_TEMPERATURE_COVARIANCE:
        return "Temperature Covariance";
    case PARAM_TYPE_HUMIDITY_COVARIANCE:
        return "Humidity Covariance";
    case PARAM_TYPE_DISTANCE_COVARIANCE:
        return "Distance Covariance";
    case PARAM_TYPE_SENSOR_HEIGHT:
        return "Sensor Height";
    case PARAM_TYPE_MIN_LEVEL:
        return "Min Level Height";
    case PARAM_TYPE_MAX_LEVEL:
        return "Max Level Height";
    case PARAM_TYPE_WATER_TANK_AREA:
        return "Water Tank Area";
    case PARAM_TYPE_WATER_TANK_TYPE:
        return "Water Tank Type";
    default:
        return "";
    }
}

static const char *keyTypeToString(KeyType keyType) __attribute__((unused));
static const char *keyTypeToString(KeyType keyType)
{
    switch (keyType)
    {
    case KEY_TYPE_DHM_KEY:
        return "DHM Key";
    case KEY_TYPE_ENCRYPTION_KEY:
        return "Encryption Key";
    default:
        return "Unknown Key";
    }
}

static const char *commandTypeToString(CommandType commandType) __attribute__((unused));
static const char *commandTypeToString(CommandType commandType)
{
    switch (commandType)
    {
    case COMMAND_TYPE_RESET_DEVICE:
        return "Reset Device";
    case COMMAND_TYPE_UPDATE_PARAM:
        return "Update Parameter";
    case COMMAND_TYPE_UPDATE_SECURITY_KEYS:
        return "Update Security Key";
    default:
        return "Unknown Command";
    }
}

static const char *waterTankTypeToString(WaterTankType waterTankType) __attribute__((unused));
static const char *waterTankTypeToString(WaterTankType waterTankType)
{
    switch (waterTankType)
    {
    case WATER_TANK_TYPE_CYNDRICAL:
        return "Cylindrical Water Tank";
    case WATER_TANK_TYPE_CUBOID:
        return "Cuboid Water Tank";
    case WATER_TANK_TYPE_INVALID:
        return "Invalid Water Tank";
    default:
        return "Unknown Water Tank";
    }
}

typedef struct sensor_data_t
{
    uint8_t dataType;
    uint64_t sequenceNumber;
    uint8_t waterLevel;
    uint8_t temperatureCelsius;
    uint8_t humidity;
    uint8_t deviceID[6];
    uint8_t checksum;
} __attribute__((packed, aligned(1))) SensorData;

typedef struct reset_command_data_t
{
    uint8_t dataType;
    uint8_t commandType;
    uint8_t checksum;
} __attribute__((packed, aligned(1))) ResetCommandData;

typedef struct param_update_command_data_t
{
    uint8_t dataType;    // DataType enum
    uint8_t commandType; // CommandType enum
    uint8_t paramType;   // ParamType enum
    uint8_t dataLength;  // Contains data size (min 1 | max 4)
    uint32_t data;       // 4 byte variable to hold the data.
    uint8_t checksum;
} __attribute__((packed, aligned(1))) ParamUpdateCommandData;

typedef struct key_update_commandData
{
    uint8_t dataType;    // DataType enum
    uint8_t commandType; // CommandType enum
    uint8_t keyType;     // KeyType enum
    uint8_t dataLength;  // Contains data size (min 1 | max 32)
    uint8_t data[32];    // 4 byte variable to hold the data.
    uint8_t checksum;
} KeyUpdateCommandData;

typedef enum mqtt_topic_categories
{
    TOPIC_CATEGORY_WATER_LEVEL_DATA = 0x35,
    TOPIC_CATEGORY_DEVICE_LOGS,
    TOPIC_CATEGORY_REQUEST,
    TOPIC_CATEGORY_RESPONSE,
    TOPIC_CATEGORY_COMMAND,
    TOPIC_CATEGORY_COMMAND_ACK,
    TOPIC_CATEGORY_EVENT,
    TOPIC_CATEGORY_EVENT_ACK,
    TOPIC_CATEGORY_INVALID,
} MQTTtopicCategories;

typedef enum mqtt_message_source_types
{
    MESSAGE_SOURCE_DEVICE = 0x65,
    MESSAGE_SOURCE_SERVER,
    MESSAGE_SOURCE_CONTROL,
    MESSAGE_SOURCE_INVALID,
} MQTTMessageSourceTypes;

// Specifies the type of request that the server sends the device
typedef enum server_to_device_request_type
{
    // -------------------------------------------------------------------------------------
    // Command types that target the receiver device.
    // -------------------------------------------------------------------------------------
    // This request is targetted to the receiver device to obtain the receiver device's info
    // That includes that device ID, device type, free heap size, number of cores, uptime, etc.
    REQUEST_TYPE_RECEIVER_DEVICE_INFO = 0x43,

    // This request is targetted to the receiver device to obtain the receiver device's
    // app / firmware information which includes version, checksum, compilation date / time
    // etc.
    REQUEST_TYPE_RECEIVER_APP_INFO,

    // This request is targetted to the receiver device to obtain the receiver device's
    // wireless connection info which includes the WiFi SSID of connected AP, rssi, mac, IP etc.
    REQUEST_TYPE_RECEIVER_CONNECTION_INFO,

    // This request is targetted to the receiver device to obtain the list of Access Points
    // around the receiver device detected after a wifi scan.
    REQUEST_TYPE_RECEIVER_WIFI_SCAN,

    // This request is targetted to the receiver device to obtain the recent water level data
    // which the receiver device holds along with temperature and humidity values
    REQUEST_TYPE_RECEIVER_WATER_LEVEL_INFO,

    // This request is targetted to the receiver device to obtain the last pump run's metadata
    REQUEST_TYPE_LAST_PUMP_RUN_METADATA,

    // This request is targetted to the receiver device to obtain the lockout configuration
    REQUEST_TYPE_LOCKOUT_CONFIGURATION,

    // -------------------------------------------------------------------------------------
    // Command types that target the transmitter device.
    // -------------------------------------------------------------------------------------
    // This request is targetted to the transmitter device to obtain the transmitter device's
    // app / firmware information which includes version, checksum, compilation date / time
    // etc.
    REQUEST_TYPE_TRANSMITTER_APP_INFO,

    // This request is targetted to the transmitter device to obtain the transmitter device's info
    // That includes that chip ID, device type, free heap size, number of cores, uptime, etc.
    REQUEST_TYPE_TRANSMITTER_DEVICE_INFO,

    REQUEST_TYPE_INVALID,

} DeviceRequestType;

// Specifies the type of commands that the server sends the device
typedef enum server_to_device_command_type
{
    // -------------------------------------------------------------------------------------
    // Command types that target the receiver device.
    // -------------------------------------------------------------------------------------
    // This command is targetted for the receiver device to control gpio
    // connected to the water pump switch
    COMMAND_TYPE_CONTROL_SWITCH = 0x93,

    // This command is targetted for the receiver device which forces the
    // receiver device to check for a new firmware update.
    COMMAND_TYPE_CHECK_NEW_FIRMWARE,

    // This command is targetted for the receiver device to update the timeout
    // period for a timer that gets triggered when data is not received by the transmitter
    COMMAND_TYPE_UPDATE_DATA_RECEPTION_TIMEOUT,

    // This command is targetted for the receiver device to update the minimum
    // water level at which the peripheral connected to the water pump is set to 'ON'.
    COMMAND_TYPE_UPDATE_MIN_WATER_LEVEL,

    // This command is targetted for the receiver device to update the minimum
    // water level at which the peripheral connected to the water pump is set to 'OFF'.
    COMMAND_TYPE_UPDATE_MAX_WATER_LEVEL,

    // This command is targetted for the receiver device to update the minimum
    // water level at which the peripheral connected to the water pump, can be set to 'ON'
    // when command to do so is recevied from the server
    COMMAND_TYPE_UPDATE_ALLOWED_WL_CONTROL_PUMP,

    // This command is targetted for the receiver device to restart itself
    COMMAND_TYPE_RESET_RECEIVER_DEVICE,

    // -------------------------------------------------------------------------------------
    // Commands that target the transmitter device.
    // -------------------------------------------------------------------------------------

    // This command is targetted for the transmitter device to restart itself
    COMMAND_TYPE_RESET_TRANSMITTER_DEVICE,

    // This command is targetted for the transmitter device to update the temperature covariance
    COMMAND_TYPE_UPDATE_TEMPERATURE_COVARIANCE,

    // This command is targetted for the transmitter device to update the humidity covariance
    COMMAND_TYPE_UPDATE_HUMIDITY_COVARIANCE,

    // This command is targetted for the transmitter device to update the humidity covariance
    COMMAND_TYPE_UPDATE_WATER_LEVEL_COVARIANCE,

    // This command is targetted for the transmitter device to update the sensor height
    COMMAND_TYPE_UPDATE_SENSOR_HEIGHT,

    // This command is targetted for the transmitter device to update the min Height
    COMMAND_TYPE_UPDATE_MIN_HEIGHT,

    // This command is targetted for the transmitter device to update the max Height
    COMMAND_TYPE_UPDATE_MAX_HEIGHT,

    // This command is targetted for the transmitter device to update the water tank area
    COMMAND_TYPE_UPDATE_WATER_TANK_AREA,

    // This command is targetted for the transmitter device to update the water tank type
    COMMAND_TYPE_UPDATE_WATER_TANK_TYPE,

    // This command is targetted for the receiver device to update the manual lockout duration
    COMMAND_TYPE_UPDATE_MANUAL_LOCKOUT_DURATION,

    // This command is targetted for the receiver device to update the max seconds per percent rise
    COMMAND_TYPE_UPDATE_MAX_SECONDS_PER_PERCENT_RISE,

    // This command is targetted for the receiver device to recalibrate the sensor
    COMMAND_TYPE_RECALIBRATE,

    // This command is targetted for the receiver device to enable or disable the lockout feature
    COMMAND_TYPE_ENABLE_LOCKOUT_FEATURE,

    COMMAND_TYPE_INVALID,

} DeviceCommandType;

typedef enum device_request_response_type
{
    RESPONSE_TYPE_REQUEST_SUCCEEDED = 0xFF,
    RESPONSE_TYPE_REQUEST_FAILED = 0x00,
} DeviceRequestResponseType;

typedef enum device_command_ack_type
{
    COMMAND_ACK_TYPE_COMMAND_SUCCEEDED = 0xFF,
    COMMAND_ACK_TYPE_COMMAND_FAILED = 0x00,
} DeviceCommandACKType;

typedef enum pump_stop_reason
{
    PUMP_STOP_REASON_MANUAL,
    PUMP_STOP_REASON_REMOTE_COMMAND,
    PUMP_STOP_REASON_WATER_LEVEL_REACHED,
    PUMP_STOP_REASON_STALL_DETECTED,
    PUMP_STOP_REASON_CONFIG_MODE,
    PUMP_STOP_REASON_UNKNOWN,
} PumpStopReason;

typedef enum pump_start_reason
{
    PUMP_START_REASON_MANUAL,
    PUMP_START_REASON_REMOTE_COMMAND,
    PUMP_START_REASON_WATER_LEVEL_REACHED,
    PUMP_START_REASON_LOCKOUT_EXPIRED,
    PUMP_START_REASON_SCHEDULED_RUN,
    PUMP_START_REASON_UNKNOWN,
} PumpStartReason;

typedef struct last_pump_run_meta_data
{
    uint64_t startedAt;
    int64_t runDurationMs;
    int8_t waterLevelDelta;
    PumpStartReason startReason;
    PumpStopReason stopReason;
} LastPumpRunMetaData;

static const char *pumpStopReasonToString(PumpStopReason reason) __attribute__((unused));
static const char *pumpStopReasonToString(PumpStopReason reason)
{
    switch (reason)
    {
    case PUMP_STOP_REASON_MANUAL:
        return "manual";
    case PUMP_STOP_REASON_REMOTE_COMMAND:
        return "remote_command";
    case PUMP_STOP_REASON_WATER_LEVEL_REACHED:
        return "water_level_reached";
    case PUMP_STOP_REASON_STALL_DETECTED:
        return "stall_detected";
    case PUMP_STOP_REASON_CONFIG_MODE:
        return "config_mode";
    default:
        return "unknown";
    }
}

static const char *pumpStartReasonToString(PumpStartReason reason) __attribute__((unused));
static const char *pumpStartReasonToString(PumpStartReason reason)
{
    switch (reason)
    {
    case PUMP_START_REASON_MANUAL:
        return "manual";
    case PUMP_START_REASON_REMOTE_COMMAND:
        return "remote_command";
    case PUMP_START_REASON_WATER_LEVEL_REACHED:
        return "water_level_reached";
    case PUMP_START_REASON_LOCKOUT_EXPIRED:
        return "lockout_expired";
    case PUMP_START_REASON_SCHEDULED_RUN:
        return "scheduled_run";
    default:
        return "unknown";
    }
}

typedef enum device_event_type
{
    EVENT_TYPE_WATER_PUMP_ON = 0x8A,
    EVENT_TYPE_WATER_PUMP_OFF,
    EVENT_TYPE_LOCKOUT_DEACTIVATED,
    EVENT_TYPE_DATA_RECEPTION_TIMEOUT,
    EVENT_TYPE_WL_NO_INCREASE_TIMEOUT,
    EVENT_TYPE_DEVICE_CONNECTED,
    EVENT_TYPE_DEVICE_DISCONNECTED,
    // OTA Update Events (0x91 - 0x94)
    EVENT_TYPE_OTA_STARTED = 0x91,
    EVENT_TYPE_OTA_PROGRESS,
    EVENT_TYPE_OTA_COMPLETED,
    EVENT_TYPE_OTA_FAILED,
} DeviceEventType;

typedef enum event_verbosity_level
{
    EVENT_LEVEL_VERBOSE,
    EVENT_LEVEL_INFO,
    EVENT_LEVEL_ALERT,
} DeviceEventVerbosity;

static DeviceRequestType numberToDeviceRequestType(uint8_t number) __attribute__((unused));
static DeviceRequestType numberToDeviceRequestType(uint8_t number)
{
    switch (number)
    {
    case (uint8_t)REQUEST_TYPE_RECEIVER_DEVICE_INFO:
        return REQUEST_TYPE_RECEIVER_DEVICE_INFO;
    case (uint8_t)REQUEST_TYPE_RECEIVER_APP_INFO:
        return REQUEST_TYPE_RECEIVER_APP_INFO;
    case (uint8_t)REQUEST_TYPE_RECEIVER_CONNECTION_INFO:
        return REQUEST_TYPE_RECEIVER_CONNECTION_INFO;
    case (uint8_t)REQUEST_TYPE_RECEIVER_WIFI_SCAN:
        return REQUEST_TYPE_RECEIVER_WIFI_SCAN;
    case (uint8_t)REQUEST_TYPE_RECEIVER_WATER_LEVEL_INFO:
        return REQUEST_TYPE_RECEIVER_WATER_LEVEL_INFO;
    case (uint8_t)REQUEST_TYPE_LAST_PUMP_RUN_METADATA:
        return REQUEST_TYPE_LAST_PUMP_RUN_METADATA;
    case (uint8_t)REQUEST_TYPE_LOCKOUT_CONFIGURATION:
        return REQUEST_TYPE_LOCKOUT_CONFIGURATION;
    case (uint8_t)REQUEST_TYPE_TRANSMITTER_APP_INFO:
        return REQUEST_TYPE_TRANSMITTER_APP_INFO;
    case (uint8_t)REQUEST_TYPE_TRANSMITTER_DEVICE_INFO:
        return REQUEST_TYPE_TRANSMITTER_DEVICE_INFO;
    default:
        return REQUEST_TYPE_INVALID;
    }
}

static DeviceCommandType numberToDeviceCommandType(uint8_t number) __attribute__((unused));
static DeviceCommandType numberToDeviceCommandType(uint8_t number)
{
    switch (number)
    {
    case (uint8_t)COMMAND_TYPE_CONTROL_SWITCH:
        return COMMAND_TYPE_CONTROL_SWITCH;
    case (uint8_t)COMMAND_TYPE_CHECK_NEW_FIRMWARE:
        return COMMAND_TYPE_CHECK_NEW_FIRMWARE;
    case (uint8_t)COMMAND_TYPE_UPDATE_DATA_RECEPTION_TIMEOUT:
        return COMMAND_TYPE_UPDATE_DATA_RECEPTION_TIMEOUT;
    case (uint8_t)COMMAND_TYPE_UPDATE_MIN_WATER_LEVEL:
        return COMMAND_TYPE_UPDATE_MIN_WATER_LEVEL;
    case (uint8_t)COMMAND_TYPE_UPDATE_MAX_WATER_LEVEL:
        return COMMAND_TYPE_UPDATE_MAX_WATER_LEVEL;
    case (uint8_t)COMMAND_TYPE_UPDATE_ALLOWED_WL_CONTROL_PUMP:
        return COMMAND_TYPE_UPDATE_ALLOWED_WL_CONTROL_PUMP;
    case (uint8_t)COMMAND_TYPE_RESET_RECEIVER_DEVICE:
        return COMMAND_TYPE_RESET_RECEIVER_DEVICE;
    case (uint8_t)COMMAND_TYPE_RESET_TRANSMITTER_DEVICE:
        return COMMAND_TYPE_RESET_TRANSMITTER_DEVICE;
    case (uint8_t)COMMAND_TYPE_UPDATE_TEMPERATURE_COVARIANCE:
        return COMMAND_TYPE_UPDATE_TEMPERATURE_COVARIANCE;
    case (uint8_t)COMMAND_TYPE_UPDATE_HUMIDITY_COVARIANCE:
        return COMMAND_TYPE_UPDATE_HUMIDITY_COVARIANCE;
    case (uint8_t)COMMAND_TYPE_UPDATE_WATER_LEVEL_COVARIANCE:
        return COMMAND_TYPE_UPDATE_WATER_LEVEL_COVARIANCE;
    case (uint8_t)COMMAND_TYPE_UPDATE_SENSOR_HEIGHT:
        return COMMAND_TYPE_UPDATE_SENSOR_HEIGHT;
    case (uint8_t)COMMAND_TYPE_UPDATE_MIN_HEIGHT:
        return COMMAND_TYPE_UPDATE_MIN_HEIGHT;
    case (uint8_t)COMMAND_TYPE_UPDATE_MAX_HEIGHT:
        return COMMAND_TYPE_UPDATE_MAX_HEIGHT;
    case (uint8_t)COMMAND_TYPE_UPDATE_WATER_TANK_AREA:
        return COMMAND_TYPE_UPDATE_WATER_TANK_AREA;
    case (uint8_t)COMMAND_TYPE_UPDATE_WATER_TANK_TYPE:
        return COMMAND_TYPE_UPDATE_WATER_TANK_TYPE;
    case (uint8_t)COMMAND_TYPE_UPDATE_MANUAL_LOCKOUT_DURATION:
        return COMMAND_TYPE_UPDATE_MANUAL_LOCKOUT_DURATION;
    case (uint8_t)COMMAND_TYPE_UPDATE_MAX_SECONDS_PER_PERCENT_RISE:
        return COMMAND_TYPE_UPDATE_MAX_SECONDS_PER_PERCENT_RISE;
    case (uint8_t)COMMAND_TYPE_RECALIBRATE:
        return COMMAND_TYPE_RECALIBRATE;
    case (uint8_t)COMMAND_TYPE_ENABLE_LOCKOUT_FEATURE:
        return COMMAND_TYPE_ENABLE_LOCKOUT_FEATURE;
    default:
        return COMMAND_TYPE_INVALID;
    }
}

static const char *deviceCommandTypeToString(DeviceCommandType commandType) __attribute__((unused));
static const char *deviceCommandTypeToString(DeviceCommandType commandType)
{
    switch (commandType)
    {
    case COMMAND_TYPE_CONTROL_SWITCH:
        return "Control Switch Command";
    case COMMAND_TYPE_CHECK_NEW_FIRMWARE:
        return "Check New Firmware Command";
    case COMMAND_TYPE_UPDATE_DATA_RECEPTION_TIMEOUT:
        return "Update Data reception Timeout Command";
    case COMMAND_TYPE_UPDATE_MIN_WATER_LEVEL:
        return "Update Min Water Level Command";
    case COMMAND_TYPE_UPDATE_MAX_WATER_LEVEL:
        return "Update Max Water Level Command";
    case COMMAND_TYPE_UPDATE_ALLOWED_WL_CONTROL_PUMP:
        return "Update WL Control Pump Command";
    case COMMAND_TYPE_RESET_RECEIVER_DEVICE:
        return "Reset Receiver Device Command";
    case COMMAND_TYPE_RESET_TRANSMITTER_DEVICE:
        return "Reset Transmitter Command";
    case COMMAND_TYPE_UPDATE_TEMPERATURE_COVARIANCE:
        return "Update Temperature Covariance Command";
    case COMMAND_TYPE_UPDATE_HUMIDITY_COVARIANCE:
        return "Update Humidity Covariance Command";
    case COMMAND_TYPE_UPDATE_WATER_LEVEL_COVARIANCE:
        return "Update Water Level Command";
    case COMMAND_TYPE_UPDATE_SENSOR_HEIGHT:
        return "Update Sensor Height Command";
    case COMMAND_TYPE_UPDATE_MIN_HEIGHT:
        return "Update Min Height Command";
    case COMMAND_TYPE_UPDATE_MAX_HEIGHT:
        return "Update Max Height Command";
    case COMMAND_TYPE_UPDATE_WATER_TANK_AREA:
        return "Update Water Tank Area Command";
    case COMMAND_TYPE_UPDATE_WATER_TANK_TYPE:
        return "Update Water Tank Type Command";
    case COMMAND_TYPE_UPDATE_MANUAL_LOCKOUT_DURATION:
        return "Update Manual Lockout Duration Command";
    case COMMAND_TYPE_UPDATE_MAX_SECONDS_PER_PERCENT_RISE:
        return "Update Max Seconds Per Percent Rise Command";
    case COMMAND_TYPE_RECALIBRATE:
        return "Recalibrate Command";
    case COMMAND_TYPE_ENABLE_LOCKOUT_FEATURE:
        return "Enable Lockout Feature Command";
    case COMMAND_TYPE_INVALID:
        return "Invalid Command";
    }
    return "Invalid Command";
}

static const char *deviceRequestTypeToString(DeviceRequestType requestType) __attribute__((unused));
static const char *deviceRequestTypeToString(DeviceRequestType requestType)
{
    switch (requestType)
    {
    case REQUEST_TYPE_RECEIVER_DEVICE_INFO:
        return "Receiver Device Info Request";
    case REQUEST_TYPE_RECEIVER_APP_INFO:
        return "Receiver App Info Request";
    case REQUEST_TYPE_RECEIVER_CONNECTION_INFO:
        return "Receiver Connection Info Request";
    case REQUEST_TYPE_RECEIVER_WIFI_SCAN:
        return "Receiver WiFi Scan Request";
    case REQUEST_TYPE_RECEIVER_WATER_LEVEL_INFO:
        return "Receiver Water Level Info Request";
    case REQUEST_TYPE_TRANSMITTER_APP_INFO:
        return "Transmitter App Info Request";
    case REQUEST_TYPE_TRANSMITTER_DEVICE_INFO:
        return "Transmitter Device Info Request";
    case REQUEST_TYPE_LAST_PUMP_RUN_METADATA:
        return "Last Pump Run Metadata Request";
    case REQUEST_TYPE_LOCKOUT_CONFIGURATION:
        return "Lockout Configuration Request";
    case REQUEST_TYPE_INVALID:
        return "Invalid Request";
    }
    return "Invalid Request";
}

#endif