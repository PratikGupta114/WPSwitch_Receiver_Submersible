#include <stdio.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "app_events.h"
#include <string>

#include "esp_log.h"
#include "esp_mac.h"
#include "driver/gpio.h"      // For early GPIO initialization
#include "esp_rom_sys.h"      // For esp_rom_delay_us()

#include "peripherals/PeripheralManager.h"
#include "peripherals/UartManager.h"
#include "https/HttpsServer.h"
#include "storage/nvsManager.h"
#include "wifi/WiFiManager.h"
#include "https/HttpsClient.h"
#include "ota/otaManager.h"
#include "mqtt/MQTTManager.h"
#include "mqtt/MqttCommandProcessor.h"
#include "peripherals/SensorDataRepo.h"
#include "peripherals/BuzzerControl.h"
#include "data/dataTypes.h"
#include "pump_controller/PumpController.h" // ADDED
#include "ledIndication/LedIndication.h"
#include "peripherals/I2CManager.h"
#include "utils/HeapMonitor.h"
#include "filters/WaterLevelMedianFilter.h"
#include "filters/WaterLevelKalmanFilter.h"
#include "switching_unit/SwitchingUnitManager.h"

#include "esp_heap_caps.h"
#include "esp_heap_trace.h"

#ifdef CONFIG_HEAP_TRACING_STANDALONE
#define HEAP_TRACE_START() ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS))
#define HEAP_TRACE_STOP_DUMP()              \
    do                                      \
    {                                       \
        ESP_ERROR_CHECK(heap_trace_stop()); \
        heap_trace_dump();                  \
    } while (0)
#else
#define HEAP_TRACE_START()
#define HEAP_TRACE_STOP_DUMP()
#endif
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <sys/time.h>
// #include "bluetooth/BluetoothManager.h"
#include "cJSON.h"

#include "esp_sntp.h"
#include <inttypes.h>

#include "esp_event.h"
#include "nvs_flash.h"

#include "secrets.h"

#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

#define TAG "Main"
#define HEAP_TRACE_NUM_RECORDS 200
// static heap_trace_record_t trace_records[HEAP_TRACE_NUM_RECORDS];

// Pin numbers for the LCD
// #define PIN_CS GPIO_NUM_5     // RS		| CS
// #define PIN_MOSI GPIO_NUM_23  // R/W    | DATA
// #define PIN_CLK GPIO_NUM_18   // ENABLE	| CLOCK
// #define PIN_RESET GPIO_NUM_22 // RESET

#define SAMPLE_DEVICE_ID "f82819250b1b"
#define DEVICE_ID_CHAR_LEN 12
#define DEVICE_ID_BUFFER_LEN (DEVICE_ID_CHAR_LEN + 1)
#define DEVICE_ID2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define DEVICE_ID_STR "%02x%02x%02x%02x%02x%02x"

#define RESTART_TASK_STACK_DEPTH 2048  // httpsServer.stop() does TLS cleanup requiring adequate stack
#define RESTART_DELAY_MS 4000

// #define TRIG_PIN GPIO_NUM_13
// #define ECHO_PIN GPIO_NUM_12
// #define WATER_LEVEL_NO_CHANGE_TIMEOUT_MILLIS 60000

#ifdef __cplusplus
extern "C"
{
#endif
    void app_main();
#ifdef __cplusplus
}
#endif

// Event group for application-level signals
static EventGroupHandle_t app_event_group;

// Heap monitoring timer (Requirement 1.2, 1.6)
#define HEAP_MONITOR_INTERVAL_MS (60 * 1000)  // 60 seconds
static TimerHandle_t heapMonitorTimer = NULL;

char deviceID[DEVICE_ID_BUFFER_LEN];
// TimerHandle_t wlNoChangeTimer; // MOVED to PumpController

WiFiManager wifiManager;
PeripheralManager peripheralManager;
SwitchingUnitManager switchingUnitManager;
HttpsServer httpsServer;
HttpsClient httpsClient;
UartManager uartManager;
NVSManager nvsManager;
OTAManager otaManager(&httpsClient);
// OTAManager otaManager(&httpsClient, 30000);
MQTT5Manager mqttManager;
BuzzerControl buzzerControl;

// static uint8_t timerCallbackTriggerCount = 0; // MOVED to PumpController
static bool motorToggleRoutineRunning = false;
// bool manualPumpRestartRequired = false; // MOVED to PumpController
static bool isInConfigMode = false;

SensorDataRepo sensorDataRepo; // Declare globally
std::function<void(DeviceEventType, DeviceEventVerbosity, const cJSON *)> mqttEventPublisher;

I2CManager i2cManager;
static bool initialTimeSyncFromRTC = false;

LedIndication ledIndication;
NVSManager nvs;

// Placeholder initial values, these should be read from NVS in app_main
// and then updated in pumpController via its update methods.
uint8_t initial_min_level_start_pump = 10;
uint8_t initial_max_level_stop_pump = 90;

// ============================================================================
// MQTT Publish Filtering Configuration - Median Filter Based Publishing
// ============================================================================
// Reduces unnecessary MQTT publishes caused by sensor noise while capturing
// all real 1% water level changes.
//
// Problem: RF sensor has ±1% noise causing water level to fluctuate between
// adjacent values (e.g., 67% ↔ 68%), triggering excessive MQTT publishes.
// Water level changes in 1% increments, so simple threshold filtering fails.
//
// Solution: 7-sample median filter
// - Window size: 7 samples (3.5 seconds at 500ms sensor rate)
// - Filters noise oscillations and single-sample glitches by requiring majority consensus
// - Publishes when filtered level changes by >= 1%
// - Improved rejection of transient sensor errors compared to 5-sample window
//
// Publishing Triggers:
// - Water level: Filtered level changes by >= 1% (integer comparison)
// - Pump state: Any change (ON/OFF) - critical for monitoring
// - Heartbeat: Every 5 minutes - ensures periodic temp/humidity updates
// - First publish: Always publish on boot
//
// Note: Temperature and humidity are included in every message but do NOT
// trigger publishes. The server polls these values via heartbeat interval.
// ============================================================================

// Heartbeat interval - ensures periodic updates even without changes
#define PUBLISH_HEARTBEAT_INTERVAL_MS (5 * 60 * 1000)  // 5 minutes

// Kalman filter for water level noise smoothing (FIRST STAGE)
// Smooths high-frequency oscillations (37%↔38%) before median filtering
// Based on backend analysis: R=10, Q=1 reduces noise by 15.94% (MSE)
// Memory: ~20 bytes, CPU: O(1) per update
static WaterLevelKalmanFilter waterLevelKalmanFilter;
static bool kalmanFilterInitialized = false;

// Median filter for water level noise filtering (SECOND STAGE)
// The median filter (7-sample window) rejects burst oscillations and single-sample glitches.
// Combined with Kalman smoothing and trend confirmation, this provides
// very smooth data for visualization while capturing all real 1% changes.
static WaterLevelMedianFilter waterLevelMedianFilter;
static bool medianFilterInitialized = false;

// Last water level from NVS (persisted across reboots)
// Used to determine if water level changed significantly during restart
// -1 = invalid/first boot, 0-100 = last published water level
static int16_t lastWaterLevelFromNVS = -1;

// Tracking variables for last published values
static uint8_t lastPublishedWaterLevel = 255;         // Invalid sentinel, forces first publish
static int8_t lastPublishedTemperature = -128;        // Tracked but doesn't trigger publishes
static uint8_t lastPublishedHumidity = 255;           // Tracked but doesn't trigger publishes
static PeripheralState lastPublishedPumpState = OFF;  // Assume OFF at boot
static uint64_t lastPublishTimestamp = 0;             // Forces first publish via heartbeat
static bool firstPublishDone = false;                 // Explicit first-publish flag

// Trend confirmation: Pending water level waiting for next reading to confirm it's not oscillation
// -1 = no pending change, 0-100 = level waiting for confirmation
static int16_t pendingWaterLevel = -1;

// Oscillation detection: Track last 3 published values to detect alternating patterns
static uint8_t publishHistory[3] = {255, 255, 255};  // 255 = invalid/uninitialized
static uint8_t publishHistoryCount = 0;              // Number of valid entries (0-3)
static bool inOscillation = false;                   // Currently suppressing oscillation
static uint8_t oscillationLow = 0;                   // Lower bound of oscillation range
static uint8_t oscillationHigh = 0;                  // Upper bound of oscillation range

// Spike detection: Track potential spikes (rapid large changes that may reverse)
static int16_t pendingSpike = -1;                    // -1 = no pending spike, 0-100 = spike value waiting for confirmation
static const uint8_t SPIKE_THRESHOLD = 10;           // Minimum change (%) to be considered a spike



/**
 * @brief Timer callback for periodic heap status logging
 * 
 * Called every HEAP_MONITOR_INTERVAL_MS (60 seconds) to log heap status.
 * Also logs task stack watermarks when WATERMARK_LOGGING_ENABLED is set.
 * 
 * Validates: Requirements 1.2, 1.3, 1.6
 */
static void heapMonitorTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;  // Unused parameter
    HeapMonitor::logHeapStatus();
    
#if WATERMARK_LOGGING_ENABLED
    HeapMonitor::logTaskStackWatermarks();
#endif
}

void delayed_restart_task(void *pvParameters)
{
    // Stop the HTTPS server before restarting
    if (httpsServer.isRunning())
    {
        ESP_LOGE(TAG, "Stopping HTTPS server ..");
        httpsServer.stop();
    }

    if (mqttManager.isConnected())
    {
        ESP_LOGI(TAG, "Disconnecting from MQTT broker ..");
        mqttManager.disconnect();
        // ESP_LOGI(TAG, "Broker disconnected now disconnecting from wifi ..");
    }

    if (wifiManager.isWiFiConnected())
    {
        wifiManager.disconnectFromApSync();
        ESP_LOGI(TAG, "WiFI AP disconnected now restarting");
    }

    // Delay to allow for cleanup
    uint32_t delay_ms = (uint32_t)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    // Restart the device
    esp_restart();
}

void getDeviceID(char *deviceIDStringHolder, size_t maxBufferSize)
{
    if (deviceIDStringHolder == NULL)
    {
        ESP_LOGE(TAG, "Device ID string holder is null");
        return;
    }
    uint8_t macAddr[6]; // Use stack allocation instead of malloc
    esp_read_mac(macAddr, ESP_MAC_WIFI_STA);
    snprintf(deviceIDStringHolder, maxBufferSize, DEVICE_ID_STR, DEVICE_ID2STR(macAddr));
}

const char *chipModelToString(esp_chip_model_t chipMode)
{
    switch (chipMode)
    {
    case CHIP_ESP32:
        return "ESP32";
    case CHIP_ESP32S2:
        return "ESP32S2";
    case CHIP_ESP32S3:
        return "ESP32S3";
    case CHIP_ESP32C3:
        return "ESP32C3";
    case CHIP_ESP32H2:
        return "ESP32H2";
    case CHIP_ESP32C2:
        return "ESP32C2";
    default:
        return "UNKNOWN";
    }
    return "";
}

void printMacAddresses()
{
    uint8_t macAddr[6]; // Use stack allocation instead of malloc
    esp_read_mac(macAddr, ESP_MAC_WIFI_STA);
    ESP_LOGW(TAG, "ESP_MAC_WIFI_STA \t-> " MACSTR, MAC2STR(macAddr));
    esp_read_mac(macAddr, ESP_MAC_WIFI_SOFTAP);
    ESP_LOGW(TAG, "ESP_MAC_WIFI_SOFTAP \t-> " MACSTR, MAC2STR(macAddr));
    esp_read_mac(macAddr, ESP_MAC_BT);
    ESP_LOGW(TAG, "ESP_MAC_BT \t\t-> " MACSTR, MAC2STR(macAddr));
    esp_read_mac(macAddr, ESP_MAC_ETH);
    ESP_LOGW(TAG, "ESP_MAC_ETH \t\t-> " MACSTR, MAC2STR(macAddr));
}

/**
 * @brief Determines if water level data should be published to MQTT
 * 
 * Uses trend confirmation algorithm to distinguish oscillation from real changes.
 * Temperature and humidity are included in the message but do NOT trigger publishes.
 * 
 * Publishing Logic:
 * - Always publish: First time, pump state change, heartbeat (5 min)
 * - Water level: Trend confirmation with 1-reading lookahead
 * 
 * Trend Confirmation Algorithm:
 * Waits for the NEXT reading to confirm a change is not oscillation:
 * - Change detected → Mark as PENDING, wait for next reading
 * - Next reading continues trend → CONFIRM and publish pending change
 * - Next reading reverses → CANCEL pending (oscillation detected)
 * 
 * Example: Oscillation at 37% ↔ 38%
 * - Published: 37%
 * - Reading: 38% → PENDING (wait for confirmation)
 * - Reading: 37% → CANCEL (reversed, oscillation detected)
 * - Reading: 38% → PENDING again
 * - Reading: 37% → CANCEL again
 * Result: Oscillation completely suppressed, 0 publishes
 * 
 * Example: Real upward trend 37% → 38% → 39%
 * - Published: 37%
 * - Reading: 38% → PENDING
 * - Reading: 39% → CONFIRM 38% (continued upward), publish 38%, 39% now PENDING
 * - Reading: 40% → CONFIRM 39%, publish 39%
 * Result: All changes captured with 500ms delay (1 reading)
 * 
 * Example: Single spike 37% → 50% → 37%
 * - Published: 37%
 * - Reading: 50% → PENDING
 * - Reading: 37% → CANCEL (returned to original, spike rejected)
 * Result: Spike suppressed, 0 publishes
 * 
 * Benefits:
 * - Captures ALL real 1% changes (no missed trends)
 * - Suppresses ALL oscillation noise (37% ↔ 38% never publishes)
 * - No tuning parameters (robust, simple)
 * - Minimal delay (500ms = 1 sensor reading)
 * - Minimal memory (2 bytes for pending level)
 * 
 * @param filtered_level Median-filtered water level estimate (0-100)
 * @param sensorData Current sensor readings (included in message)
 * @param currentPumpState Current pump on/off state
 * @param currentTimestamp Current time in milliseconds
 * @return true if data should be published, false otherwise
 */
/**
 * @brief Determines if water level data should be published to MQTT
 * 
 * Multi-stage filtering pipeline to suppress noise while capturing real changes:
 * 1. Median filter (7 samples) - rejects burst noise and single-sample glitches
 * 2. Trend confirmation (1-reading lookahead) - detects fast oscillations
 * 3. Oscillation detection - suppresses persistent alternating patterns (65%↔66%)
 * 4. Spike detection - suppresses rapid large changes that reverse quickly
 * 
 * Publishing triggers:
 * - First publish after boot
 * - Pump state change (critical for monitoring)
 * - Heartbeat interval (ensures periodic temp/humidity updates)
 * - Water level change confirmed by trend + oscillation + spike checks
 * 
 * Trend confirmation algorithm (1-reading lookahead):
 * - Detects oscillation: 65% → 66% → 65% (suppressed)
 * - Confirms trend: 65% → 66% → 66% (published)
 * - Confirms continued trend: 65% → 66% → 67% (both published)
 * 
 * Oscillation detection:
 * - Tracks last 2-3 published values
 * - Detects alternation between two adjacent values (differ by 1%)
 * - Suppresses until value breaks out of range (±2%)
 * 
 * Spike detection:
 * - Detects changes ≥10% from last published value
 * - Waits for confirmation (stays at spike value → real change)
 * - Suppresses if returns to baseline (spike rejected)
 * 
 * Benefits:
 * - Suppresses ALL oscillation noise (37% ↔ 38% never publishes after initial detection)
 * - Suppresses transient spikes (65% → 80% → 65% never publishes 80%)
 * - Captures ALL real 1% changes without artificial delays
 * - No tuning parameters (robust, simple)
 * - Minimal delay (500ms = 1 sensor reading for trend confirmation)
 * - Minimal memory (13 bytes total state)
 * 
 * @param filtered_level Median-filtered water level estimate (0-100)
 * @param sensorData Current sensor readings (included in message)
 * @param currentPumpState Current pump on/off state
 * @param currentTimestamp Current time in milliseconds
 * @return true if data should be published, false otherwise
 */
static bool shouldPublishWaterLevelData(
    uint8_t filtered_level,
    const SensorData* sensorData,
    PeripheralState currentPumpState,
    uint64_t currentTimestamp)
{
    if (sensorData == NULL) {
        ESP_LOGW(TAG, "shouldPublish: NULL sensor data");
        return false;
    }

    // CRITICAL: Never publish the same water level twice consecutively
    // This prevents duplicate MQTT publishes when trend confirmation logic
    // might trigger multiple times for the same value
    if (firstPublishDone && filtered_level == lastPublishedWaterLevel && currentPumpState == lastPublishedPumpState) {
        ESP_LOGD(TAG, "Filtered level %d%% unchanged from last publish (pump: %s), skipping", 
                 filtered_level, currentPumpState == ON ? "ON" : "OFF");
        return false;
    }

    // First publish after boot - check if water level changed from last reboot
    if (!firstPublishDone) {
        // Check if we have a previous water level from NVS
        if (lastWaterLevelFromNVS != -1 && filtered_level == (uint8_t)lastWaterLevelFromNVS) {
            // Water level unchanged from last publish before reboot - skip publish
            ESP_LOGI(TAG, "First publish skipped: Water level unchanged at %d%% (same as before reboot)", filtered_level);
            
            // Initialize tracking variables (same as updateLastPublishedValues but without publish)
            publishHistory[0] = filtered_level;
            publishHistoryCount = 1;
            lastPublishedWaterLevel = filtered_level;
            lastPublishedTemperature = sensorData->temperatureCelsius;
            lastPublishedHumidity = sensorData->humidity;
            lastPublishedPumpState = currentPumpState;
            lastPublishTimestamp = currentTimestamp;
            firstPublishDone = true;
            
            return false;  // Don't publish
        }
        
        // Water level changed or first boot - publish
        ESP_LOGI(TAG, "Publish: First publish after boot (level: %d%%, previous: %d%%)", 
                 filtered_level, lastWaterLevelFromNVS);
        // Initialize publish history with first value
        publishHistory[0] = filtered_level;
        publishHistoryCount = 1;
        return true;
    }
    
    // Always publish pump state changes (critical for monitoring)
    if (currentPumpState != lastPublishedPumpState) {
        ESP_LOGI(TAG, "Publish: Pump state changed (%s -> %s)", 
                 lastPublishedPumpState == ON ? "ON" : "OFF",
                 currentPumpState == ON ? "ON" : "OFF");
        // Clear any pending on pump state change
        pendingWaterLevel = -1;
        pendingSpike = -1;
        // Add to publish history
        if (publishHistoryCount < 3) {
            publishHistory[publishHistoryCount++] = filtered_level;
        } else {
            publishHistory[0] = publishHistory[1];
            publishHistory[1] = publishHistory[2];
            publishHistory[2] = filtered_level;
        }
        return true;
    }
    
    // Check heartbeat interval (ensures periodic temp/humidity updates)
    uint64_t timeSinceLastPublish = currentTimestamp - lastPublishTimestamp;
    if (timeSinceLastPublish >= PUBLISH_HEARTBEAT_INTERVAL_MS) {
        ESP_LOGI(TAG, "Publish: Heartbeat (%" PRIu64 " ms since last publish)", 
                 timeSinceLastPublish);
        // Clear any pending on heartbeat
        pendingWaterLevel = -1;
        pendingSpike = -1;
        // Add to publish history
        if (publishHistoryCount < 3) {
            publishHistory[publishHistoryCount++] = filtered_level;
        } else {
            publishHistory[0] = publishHistory[1];
            publishHistory[1] = publishHistory[2];
            publishHistory[2] = filtered_level;
        }
        return true;
    }
    
    // ========== STAGE 1: TREND CONFIRMATION ==========
    
    // Case 1: No change from last published - clear any pending
    if (filtered_level == lastPublishedWaterLevel) {
        if (pendingWaterLevel != -1) {
            ESP_LOGD(TAG, "Trend cancelled: Returned to published level %d%% (pending %d%% cancelled)",
                     filtered_level, pendingWaterLevel);
            pendingWaterLevel = -1;
        }
        if (pendingSpike != -1) {
            ESP_LOGD(TAG, "Spike cancelled: Returned to published level %d%% (pending spike %d%% cancelled)",
                     filtered_level, pendingSpike);
            pendingSpike = -1;
        }
        return false;
    }
    
    // Case 2: No pending change - this is a new change, mark as pending
    if (pendingWaterLevel == -1) {
        pendingWaterLevel = filtered_level;
        int delta = (int)filtered_level - (int)lastPublishedWaterLevel;
        ESP_LOGD(TAG, "Trend pending: %d%% → %d%% (delta: %+d%%, waiting for confirmation)",
                 lastPublishedWaterLevel, filtered_level, delta);
        return false;
    }
    
    // Case 3: Level stayed at pending value - TREND CONFIRMED, proceed to oscillation/spike checks
    if (filtered_level == pendingWaterLevel) {
        int delta = (int)pendingWaterLevel - (int)lastPublishedWaterLevel;
        
        // ========== STAGE 2: SPIKE DETECTION ==========
        
        // Check if this is a large change (potential spike)
        if (abs(delta) >= SPIKE_THRESHOLD) {
            if (pendingSpike == -1) {
                // First large change - mark as pending spike
                pendingSpike = pendingWaterLevel;
                ESP_LOGD(TAG, "Potential spike detected: %d%% → %d%% (delta: %+d%%), waiting for confirmation",
                         lastPublishedWaterLevel, pendingSpike, delta);
                return false;
            } else if (pendingWaterLevel == pendingSpike) {
                // Stayed at spike value - it's real, not a spike
                ESP_LOGI(TAG, "Publish: Large change confirmed (stayed at %d%%): %d%% → %d%% (delta: %+d%%)",
                         pendingSpike, lastPublishedWaterLevel, pendingSpike, delta);
                pendingSpike = -1;
                // Continue to oscillation detection
            }
        } else {
            // Small change - clear any pending spike
            if (pendingSpike != -1) {
                ESP_LOGD(TAG, "Spike recovery: returned to normal changes (spike suppressed)");
                pendingSpike = -1;
                return false;
            }
        }
        
        // ========== STAGE 3: OSCILLATION DETECTION ==========
        
        // Check if we're already in an oscillation - if so, only allow breakout
        if (inOscillation) {
            if (pendingWaterLevel < oscillationLow - 1 || pendingWaterLevel > oscillationHigh + 1) {
                // Breaking out of oscillation range - allow publish and clear flag
                inOscillation = false;
                ESP_LOGI(TAG, "Broke out of oscillation range (%d%%-%d%%), publishing %d%%",
                         oscillationLow, oscillationHigh, pendingWaterLevel);
                // Add to publish history
                if (publishHistoryCount < 3) {
                    publishHistory[publishHistoryCount++] = pendingWaterLevel;
                } else {
                    publishHistory[0] = publishHistory[1];
                    publishHistory[1] = publishHistory[2];
                    publishHistory[2] = pendingWaterLevel;
                }
                ESP_LOGI(TAG, "Publish: Water level %s from %d%% to %d%% (delta: %+d%%, broke out of oscillation)",
                         delta > 0 ? "rose" : "fell", lastPublishedWaterLevel, pendingWaterLevel, delta);
                return true;
            } else {
                // Still within oscillation range - BLOCK publish
                ESP_LOGD(TAG, "Still in oscillation range (%d%%-%d%%), suppressing %d%%",
                         oscillationLow, oscillationHigh, pendingWaterLevel);
                return false;
            }
        }
        
        // Not currently in oscillation - check if we're starting one
        if (publishHistoryCount >= 2) {
            uint8_t last = publishHistory[publishHistoryCount - 1];
            uint8_t secondLast = publishHistory[publishHistoryCount - 2];
            int diff = abs((int)last - (int)secondLast);
            
            // Adjacent values (differ by 1%)
            if (diff == 1) {
                // Check if current value continues alternation
                if (pendingWaterLevel == last || pendingWaterLevel == secondLast) {
                    // Oscillation pattern detected - enter oscillation mode and BLOCK this publish
                    inOscillation = true;
                    oscillationLow = (last < secondLast) ? last : secondLast;
                    oscillationHigh = (last > secondLast) ? last : secondLast;
                    ESP_LOGI(TAG, "Oscillation detected between %d%% and %d%%, suppressing publishes until breakout",
                             oscillationLow, oscillationHigh);
                    return false;  // BLOCK the publish
                }
            }
        }
        
        // Not in oscillation or spike, allow publish
        ESP_LOGI(TAG, "Publish: Water level %s from %d%% to %d%% (delta: %+d%%, confirmed by stable reading)",
                 delta > 0 ? "rose" : "fell", lastPublishedWaterLevel, pendingWaterLevel, delta);
        // Add to publish history
        if (publishHistoryCount < 3) {
            publishHistory[publishHistoryCount++] = pendingWaterLevel;
        } else {
            publishHistory[0] = publishHistory[1];
            publishHistory[1] = publishHistory[2];
            publishHistory[2] = pendingWaterLevel;
        }
        return true;
    }
    
    // Case 4: Level continued past pending - CONFIRMED + new pending
    int pendingDelta = (int)pendingWaterLevel - (int)lastPublishedWaterLevel;
    int currentDelta = (int)filtered_level - (int)lastPublishedWaterLevel;
    
    // Check if current reading is further from last published in same direction
    bool sameDirection = (pendingDelta > 0 && currentDelta > 0) || (pendingDelta < 0 && currentDelta < 0);
    bool continuedFurther = abs(currentDelta) > abs(pendingDelta);
    
    if (sameDirection && continuedFurther) {
        // Continued in same direction - confirm pending and set new pending
        ESP_LOGI(TAG, "Publish: Water level %s from %d%% to %d%% (delta: %+d%%, confirmed by continued trend to %d%%)",
                 pendingDelta > 0 ? "rose" : "fell", lastPublishedWaterLevel, pendingWaterLevel, 
                 pendingDelta, filtered_level);
        // Add to publish history
        if (publishHistoryCount < 3) {
            publishHistory[publishHistoryCount++] = pendingWaterLevel;
        } else {
            publishHistory[0] = publishHistory[1];
            publishHistory[1] = publishHistory[2];
            publishHistory[2] = pendingWaterLevel;
        }
        return true;
    }
    
    // Case 5: Level reversed - CANCEL pending, start new pending
    ESP_LOGD(TAG, "Oscillation detected: %d%% → %d%% → %d%% (pending %d%% cancelled, new pending %d%%)",
             lastPublishedWaterLevel, pendingWaterLevel, filtered_level, pendingWaterLevel, filtered_level);
    pendingWaterLevel = filtered_level;
    return false;
}

/**
 * @brief Updates tracking variables after successful MQTT publish
 * 
 * Stores the median-filtered water level and clears pending state.
 * When publishing a confirmed trend, the pending level becomes the new published level.
 * 
 * @param filtered_level Median-filtered water level (0-100)
 * @param sensorData Published sensor data (for temp/humidity)
 * @param pumpState Published pump state
 * @param timestamp Publish timestamp
 */
static void updateLastPublishedValues(
    uint8_t filtered_level,
    const SensorData* sensorData,
    PeripheralState pumpState,
    uint64_t timestamp)
{
    // CRITICAL FIX: Always update lastPublishedWaterLevel to the filtered_level that was actually
    // published and persisted to NVS. This prevents duplicate publishes when multiple logic paths
    // (trend confirmation, oscillation breakout) evaluate the same value.
    //
    // The filtered_level parameter represents what we're publishing NOW and persisting to NVS.
    // We must track this exact value to prevent re-publishing it later.
    
    lastPublishedWaterLevel = filtered_level;
    
    // Clear pending state after publish - the value has been confirmed and published
    pendingWaterLevel = -1;
    
    ESP_LOGD(TAG, "Published water level: %d%% (pending cleared)", filtered_level);
    
    if (sensorData != NULL) {
        lastPublishedTemperature = sensorData->temperatureCelsius;
        lastPublishedHumidity = sensorData->humidity;
    }
    
    lastPublishedPumpState = pumpState;
    lastPublishTimestamp = timestamp;
    firstPublishDone = true;
}

/**
 * @brief Publishes data to the MQTT broker with specified QoS level.
 *
 * QoS 1 is used for device-to-server publishing to ensure at-least-once delivery
 * of sensor data, events, and responses while allowing for potential duplicates
 * which can be handled by the server.
 *
 * @param data Pointer to a dynamically allocated buffer (e.g., from cJSON_PrintUnformatted). Ownership is transferred to this function.
 *             If publishing fails, this function will free the buffer. If publishing succeeds, the buffer is freed by MQTTManager::publish via freeMqttMessage.
 * @param length Length of the data buffer.
 * @param category MQTT topic category.
 * @param responseTopicCategory MQTT response topic category.
 * @param qos Quality of Service level.
 */
void publishData(
    char *&data,
    size_t length,
    MQTTtopicCategories category,
    MQTTtopicCategories responseTopicCategory,
    QualityOfService qos)
{
    if (data == NULL || length == 0)
    {
        ESP_LOGE(TAG, "Cannot publish NULLish data to the broker !");
        return;
    }

    MQTT5Message *message = (MQTT5Message *)malloc(sizeof(MQTT5Message));
    static const char contentTypeStr[] = "application/json";
    char *contentType = (char *)malloc(sizeof(contentTypeStr));
    strcpy(contentType, contentTypeStr);

    char *deviceID = (char *)malloc(sizeof(char) * DEVICE_ID_BUFFER_LEN);
    getDeviceID(deviceID, DEVICE_ID_BUFFER_LEN);

    char *topic = (char *)malloc(MAX_MQTT_TOPIC_LENGTH);
    char *responseTopic = (char *)malloc(MAX_MQTT_TOPIC_LENGTH);

    snprintf(
        topic,
        MAX_MQTT_TOPIC_LENGTH,
        "/%s/%s/%s",
        message_source_to_string(MESSAGE_SOURCE_DEVICE),
        deviceID,
        topic_category_to_string(category));

    snprintf(
        responseTopic,
        MAX_MQTT_TOPIC_LENGTH,
        "/%s/%s/%s",
        message_source_to_string(MESSAGE_SOURCE_SERVER),
        deviceID,
        topic_category_to_string(responseTopicCategory));

    // Validate topic lengths against MAX_MQTT_TOPIC_LENGTH constraint
    size_t topicLength = strlen(topic);
    size_t responseTopicLength = strlen(responseTopic);

    if (topicLength >= MAX_MQTT_TOPIC_LENGTH)
    {
        ESP_LOGE(TAG, "Generated topic length (%zu) exceeds maximum allowed length (%d): %s",
                 topicLength, MAX_MQTT_TOPIC_LENGTH, topic);
        free(topic);
        free(responseTopic);
        free(contentType);
        free(deviceID);
        free(message);
        if (data != NULL)
        {
            free(data);
            data = NULL;
        }
        return;
    }

    if (responseTopicLength >= MAX_MQTT_TOPIC_LENGTH)
    {
        ESP_LOGE(TAG, "Generated response topic length (%zu) exceeds maximum allowed length (%d): %s",
                 responseTopicLength, MAX_MQTT_TOPIC_LENGTH, responseTopic);
        free(topic);
        free(responseTopic);
        free(contentType);
        free(deviceID);
        free(message);
        if (data != NULL)
        {
            free(data);
            data = NULL;
        }
        return;
    }

    message->contentType = contentType;
    message->deviceID = deviceID;
    message->message = data;
    message->qos = qos;
    message->retain = false;
    message->topic = topic;
    message->responseTopic = responseTopic;

    // NOTE: publish() takes ownership of message and ALL its fields (including data).
    // It ALWAYS calls freeMqttMessage() on both success and failure paths.
    // Therefore, we must NOT free data or any other fields after calling publish().
    // Set data to NULL to prevent caller from accidentally using freed memory.
    bool publishResult = mqttManager.publish(message);
    data = NULL;  // Ownership transferred to publish(), prevent use-after-free
    
    if (!publishResult)
    {
        ESP_LOGE(TAG, "Failed to publish mqtt message");
        // NOTE: Do NOT free data here - publish() already freed it via freeMqttMessage()
    }
    // NOTE: Successful publish logging is handled inside MQTT5Manager::publish()
}

void publishWaterLevelData(SensorData *sensorData, uint64_t timestamp, uint8_t filteredLevel)
{
    if (sensorData == NULL)
    {
        ESP_LOGE(TAG, "Cannot publish null sensor data !");
        return;
    }
    if (mqttManager.isConnected() == false)
        return;

    char *jsonResultStringHolder = NULL;
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to create JSON object for water level data - out of memory");
        return;
    }

    if (timestamp > 0)
    {
        // uint64_t max is 20 digits + null terminator = 21 bytes
        char tsStr[21];
        snprintf(tsStr, sizeof(tsStr), "%" PRIu64, timestamp);
        cJSON_AddStringToObject(root, "ts", tsStr);
    }

    int waterPumpState = peripheralManager.getCurrentMotorState() == ON ? 1 : 0;
    cJSON_AddNumberToObject(root, "level", filteredLevel);
    cJSON_AddNumberToObject(root, "temp", sensorData->temperatureCelsius);
    cJSON_AddNumberToObject(root, "humidity", sensorData->humidity);
    cJSON_AddNumberToObject(root, "pumpStatus", waterPumpState);

    jsonResultStringHolder = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    // Use QoS 1 (At Least Once Delivery) for water level data publishing to ensure:
    // - Sensor data reaches the server with delivery confirmation via PUBACK
    // - Critical monitoring data is not lost due to network interruptions
    // - Allows for potential duplicates which can be handled by server deduplication logic
    // - Provides reliable delivery for time-series data collection and analysis
    publishData(
        jsonResultStringHolder,
        strlen(jsonResultStringHolder),
        TOPIC_CATEGORY_WATER_LEVEL_DATA,
        TOPIC_CATEGORY_INVALID,
        QOS_1);
}

void publishEvent(
    DeviceEventType eventType,
    DeviceEventVerbosity eventVerbosity,
    uint64_t timestamp)
{
    if (mqttManager.isConnected() == false)
        return;

    char *jsonResultStringHolder = NULL;
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to create JSON object for event - out of memory");
        return;
    }

    if (timestamp > 0)
    {
        // uint64_t max is 20 digits + null terminator = 21 bytes
        char tsStr[21];
        snprintf(tsStr, sizeof(tsStr), "%" PRIu64, timestamp);
        cJSON_AddStringToObject(root, "ts", tsStr);
    }

    cJSON_AddNumberToObject(root, "type", (uint8_t)eventType);
    cJSON_AddNumberToObject(root, "verbosity", (uint8_t)eventVerbosity);

    jsonResultStringHolder = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    // Use QoS 1 (At Least Once Delivery) for event publishing to ensure:
    // - Important events reach the server with delivery confirmation via PUBACK
    // - Critical system events like timeouts and alerts are not lost
    // - Allows for potential duplicates which can be handled by server deduplication logic
    // - Provides reliable delivery for monitoring and alerting systems
    publishData(
        jsonResultStringHolder,
        strlen(jsonResultStringHolder),
        TOPIC_CATEGORY_EVENT,
        TOPIC_CATEGORY_INVALID,
        QOS_1);
}

void app_main(void)
{


    // CRITICAL: Reduce console logging to prevent UART0 conflicts with FTDI programmer
    // This solves the NEOPixel/buzzer/display issues when FTDI is connected
    esp_log_level_set("*", ESP_LOG_WARN);  // Only warnings and errors
    esp_log_level_set("Main", ESP_LOG_INFO); // Keep main logs for debugging
    esp_log_level_set("PeripheralManager.cpp", ESP_LOG_INFO); // Enable PeripheralManager logs for debugging
    esp_log_level_set("WiFiManager", ESP_LOG_INFO); // Enable WiFiManager logs for SNTP time sync visibility
    esp_log_level_set("MQTTManager", ESP_LOG_INFO); // Enable MQTTManager logs for publish success visibility
    esp_log_level_set("HeapMonitor", ESP_LOG_INFO); // Enable HeapMonitor logs for memory monitoring
    esp_log_level_set("NVSManager", ESP_LOG_INFO); // Enable NVSManager logs for NVS operations visibility
    esp_log_level_set("WaterLevelKalmanFilter", ESP_LOG_INFO); // Enable Kalman filter INFO logs for filter operation visibility
    esp_log_level_set("WaterLevelMedianFilter", ESP_LOG_INFO); // Enable median filter INFO logs for filter operation visibility
    
    // Log restart reason early for debugging (Requirement 1.1)
    HeapMonitor::logRestartReason();
    
    // Initialize NVS through NVSManager
    if (!nvsManager.init())
    {
        ESP_LOGE(TAG, "Failed to initialize NVS - system cannot continue");
        return;
    }

    // Read last published water level from NVS (for restart comparison)
    // Default to -1 (invalid) if not found (first boot or NVS cleared)
    lastWaterLevelFromNVS = (int16_t)nvsManager.readUint8("last_wl", 255);
    if (lastWaterLevelFromNVS == 255) {
        lastWaterLevelFromNVS = -1;  // Convert 255 (not found) to -1 (invalid sentinel)
        ESP_LOGI(TAG, "No previous water level in NVS (first boot)");
    } else {
        ESP_LOGI(TAG, "Last published water level from NVS: %d%%", lastWaterLevelFromNVS);
    }

    ESP_LOGW(TAG, R"(

███████╗██╗     ██╗   ██╗ ██████╗████████╗ ██████╗
██╔════╝██║     ██║   ██║██╔════╝╚══██╔══╝██╔═══██╗
█████╗  ██║     ██║   ██║██║        ██║   ██║   ██║
██╔══╝  ██║     ██║   ██║██║        ██║   ██║   ██║
██║     ███████╗╚██████╔╝╚██████╗   ██║   ╚██████╔╝
╚═╝     ╚══════╝ ╚═════╝  ╚═════╝   ╚═╝    ╚═════╝

<< Smart Water Pump Switch Controller Firmware v1.0 >>

)");

#ifdef CONFIG_HEAP_TRACING_STANDALONE
    ESP_ERROR_CHECK(heap_trace_init_standalone(trace_records, HEAP_TRACE_NUM_RECORDS));
#endif

    app_event_group = xEventGroupCreate();

    //////////////////////////////////////////////////////////////////////////
    // Setup Heap Monitoring Timer (Requirement 1.2, 1.6)
    //////////////////////////////////////////////////////////////////////////
    
    // Log initial heap status at boot
    HeapMonitor::logHeapStatus();
    
    // Create and start periodic heap monitoring timer (every 60 seconds)
    heapMonitorTimer = xTimerCreate(
        "HeapMonitor",
        pdMS_TO_TICKS(HEAP_MONITOR_INTERVAL_MS),
        pdTRUE,   // Auto-reload timer
        NULL,     // Timer ID (not used)
        heapMonitorTimerCallback);
    
    if (heapMonitorTimer != NULL)
    {
        if (xTimerStart(heapMonitorTimer, pdMS_TO_TICKS(100)) == pdPASS)
        {
            ESP_LOGI(TAG, "Heap monitoring timer started (interval: %d ms)", HEAP_MONITOR_INTERVAL_MS);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to start heap monitoring timer");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to create heap monitoring timer");
    }

    //////////////////////////////////////////////////////////////////////////
    // Setup Pump Controller & Peripheral Manager
    //////////////////////////////////////////////////////////////////////////

    getDeviceID(deviceID, DEVICE_ID_BUFFER_LEN);
    ESP_LOGW(TAG, "Device ID : %s", deviceID);

    ESP_LOGI(TAG, "Initializing Switching Unit Manager");
    switchingUnitManager.init();

    ESP_LOGI(TAG, "Initializing Peripherals");
    peripheralManager.initPeripherals(&switchingUnitManager);
    peripheralManager.displayTwoDigitNumber(88);

    mqttEventPublisher = [&](DeviceEventType eventType, DeviceEventVerbosity verbosity, const cJSON *data)
    {
        if (mqttManager.isConnected())
        {
            char *jsonResultStringHolder = NULL;
            cJSON *root = cJSON_CreateObject();
            if (root == NULL)
            {
                ESP_LOGE(TAG, "mqttEventPublisher: Failed to create JSON object - out of memory");
                return;
            }

            // Ensure timestamp is always present and consistent
            uint64_t eventTimestamp = 0;
            if (data != nullptr && cJSON_HasObjectItem(data, "timestamp"))
            {
                cJSON *timestampItem = cJSON_GetObjectItem(data, "timestamp");
                if (cJSON_IsNumber(timestampItem))
                {
                    eventTimestamp = (uint64_t)timestampItem->valuedouble;
                }
            }

            // If no timestamp provided or timestamp is 0, get current timestamp
            if (eventTimestamp == 0)
            {
                if (!wifiManager.getTimestampMillis(eventTimestamp))
                {
                    ESP_LOGW(TAG, "mqttEventPublisher: Could not get current timestamp for event %d. Using 0.", eventType);
                    eventTimestamp = 0;
                }
            }

            // Always add timestamp field (consistent field name "ts")
            if (eventTimestamp > 0)
            {
                char tsStr[21];
                snprintf(tsStr, sizeof(tsStr), "%" PRIu64, eventTimestamp);
                cJSON_AddStringToObject(root, "ts", tsStr);
            }

            // Add standard event fields with consistent naming
            cJSON_AddNumberToObject(root, "type", (uint8_t)eventType);
            cJSON_AddNumberToObject(root, "verbosity", (uint8_t)verbosity);

            // Handle event-specific data fields for all event types
            if (data != nullptr)
            {
                cJSON *item = data->child;
                while (item)
                {
                    // Skip timestamp as we handle it separately to ensure consistency
                    if (strcmp(item->string, "timestamp") != 0)
                    {
                        // Add event-specific fields, ensuring proper field name consistency
                        cJSON_AddItemToObject(root, item->string, cJSON_Duplicate(item, true));
                    }
                    item = item->next;
                }
            }

            jsonResultStringHolder = cJSON_PrintUnformatted(root);
            cJSON_Delete(root);

            if (jsonResultStringHolder != NULL && strlen(jsonResultStringHolder) > 0)
            {
                MQTT5Message *message = (MQTT5Message *)malloc(sizeof(MQTT5Message));
                if (message == NULL)
                {
                    ESP_LOGE(TAG, "mqttEventPublisher: Failed to allocate MQTT5Message");
                    free(jsonResultStringHolder);
                    return;
                }
                
                char *contentType = (char *)malloc(sizeof("application/json"));
                strcpy(contentType, "application/json");

                char *localDeviceID = (char *)malloc(sizeof(char) * DEVICE_ID_BUFFER_LEN);
                getDeviceID(localDeviceID, DEVICE_ID_BUFFER_LEN);

                char *topic = (char *)malloc(MAX_MQTT_TOPIC_LENGTH);
                snprintf(topic, MAX_MQTT_TOPIC_LENGTH, "/%s/%s/%s",
                         message_source_to_string(MESSAGE_SOURCE_DEVICE),
                         localDeviceID,
                         topic_category_to_string(TOPIC_CATEGORY_EVENT));

                // Validate topic length against MAX_MQTT_TOPIC_LENGTH constraint
                size_t topicLength = strlen(topic);
                if (topicLength >= MAX_MQTT_TOPIC_LENGTH)
                {
                    ESP_LOGE(TAG, "mqttEventPublisher: Generated topic length (%zu) exceeds maximum allowed length (%d): %s",
                             topicLength, MAX_MQTT_TOPIC_LENGTH, topic);
                    free(topic);
                    free(contentType);
                    free(localDeviceID);
                    free(message);
                    if (jsonResultStringHolder)
                        free(jsonResultStringHolder);
                    return;
                }

                // Log event details BEFORE publish() takes ownership and frees the memory
                ESP_LOGI(TAG, "mqttEventPublisher: Publishing event - Type=%d, Topic=%s", 
                         eventType, topic);

                message->contentType = contentType;
                message->deviceID = localDeviceID;
                message->message = jsonResultStringHolder;
                // Use QoS 1 (At Least Once Delivery) for device-to-server event publishing to ensure:
                // - Events reach the server with delivery confirmation via PUBACK
                // - Critical events like pump state changes are not lost due to network issues
                // - Allows for potential duplicates which can be handled by server deduplication logic
                // - Provides good balance between reliability and performance overhead
                // - Suitable for telemetry data where occasional duplicates are acceptable
                message->qos = QOS_1;
                message->retain = false;
                message->topic = topic;
                message->responseTopic = NULL;

                // NOTE: publish() takes ownership of message and ALL its fields.
                // It ALWAYS calls freeMqttMessage() on both success and failure paths.
                // Therefore, we must NOT access topic, jsonResultStringHolder, or any other
                // fields after calling publish() - they will have been freed.
                if (!mqttManager.publish(message))
                {
                    ESP_LOGE(TAG, "mqttEventPublisher: Failed to publish event");
                }
                // NOTE: Successful publish is logged inside MQTT5Manager::publish()
                // Do NOT log topic/data here - they are freed by publish() and would cause use-after-free
            }
            else
            {
                if (jsonResultStringHolder)
                    free(jsonResultStringHolder);
                ESP_LOGE(TAG, "mqttEventPublisher: JSON string for event was null or empty.");
            }
        }
        else
        {
            ESP_LOGW(TAG, "mqttEventPublisher: MQTT not connected, event not published.");
        }
    };

    ESP_LOGI(TAG, "Initializing LED Inidications");
    LedIndication ledIndication;
    ledIndication.init();

    // Initialise I2C for RTC early so time is available before WiFi & MQTT
    if (i2cManager.init() == ESP_OK && i2cManager.isRTCConnected())
    {
        struct tm rtcTime;
        if (i2cManager.readTime(rtcTime))
        {
            time_t rtcEpoch = mktime(&rtcTime);
            if (rtcEpoch > 0)
            {
                struct timeval tv = {.tv_sec = rtcEpoch, .tv_usec = 0};
                settimeofday(&tv, NULL);
                ESP_LOGW(TAG, "System time initialised from DS1307 (%04d-%02d-%02d %02d:%02d:%02d)",
                         rtcTime.tm_year + 1900, rtcTime.tm_mon + 1, rtcTime.tm_mday,
                         rtcTime.tm_hour, rtcTime.tm_min, rtcTime.tm_sec);
                initialTimeSyncFromRTC = true;
            }
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialise I2C for RTC");
        // Print the current esp32 system time in the format of YYYY-MM-DD HH:MM:SS irrespective of the timezone and without considering the current time of writing the code.
        struct timeval tv;
        gettimeofday(&tv, NULL);
        time_t now = tv.tv_sec;
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        ESP_LOGW(TAG, "Current system time: %04d-%02d-%02d %02d:%02d:%02d",
                 timeinfo.tm_year + 1900,
                 timeinfo.tm_mon + 1,
                 timeinfo.tm_mday,
                 timeinfo.tm_hour,
                 timeinfo.tm_min,
                 timeinfo.tm_sec);
    }

    PumpController pumpController(
        peripheralManager,
        switchingUnitManager,
        buzzerControl,
        sensorDataRepo,
        ledIndication,
        mqttEventPublisher,
        app_event_group,
        nvsManager,
        initial_min_level_start_pump,
        initial_max_level_stop_pump);
    ledIndication.startLedTask();

    // Register callbacks for configuration mode button
    peripheralManager.setOnConfigButtonClickedListener(
        [&]() -> void
        {
            ESP_LOGI(TAG, "Config button clicked (simple press)");
            // TODO: implement short press behavior
        });

    peripheralManager.setOnConfigButtonLongPressedListener(
        [&]() -> void
        {
            if (isInConfigMode)
            {
                ESP_LOGW(TAG, "Config button long pressed. Exiting config mode.");
                xTaskCreatePinnedToCore(
                    delayed_restart_task,
                    "restart_task",
                    RESTART_TASK_STACK_DEPTH,  // Use consistent stack size - httpsServer.stop() needs TLS cleanup stack
                    (void *)500,
                    5,
                    NULL,
                    APP_CPU_NUM);
            }
            else
            {
                ESP_LOGW(TAG, "Config button long pressed. Signaling to enter config mode.");
                isInConfigMode = true; // Set flag immediately
                wifiManager.setConfigMode(true);

                if (peripheralManager.getCurrentMotorState() == ON)
                {
                    uint64_t timestamp = 0;
                    wifiManager.getTimestampMillis(timestamp);
                    pumpController.turnWaterPumpOff(timestamp, PUMP_STOP_REASON_CONFIG_MODE);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                xEventGroupSetBits(app_event_group, ENTER_CONFIG_MODE_BIT);
            }
        });

    peripheralManager.setOnMotorToggleButtonPressedListener(
        [&]() -> void
        {
            if (isInConfigMode)
            {
                ESP_LOGW(TAG, "Motor toggle button is disabled in configuration mode.");
                buzzerControl.playFailureTune();
                return;
            }
            if (motorToggleRoutineRunning == true)
                return;
            motorToggleRoutineRunning = true;
            ESP_LOGI(TAG, "Motor toggle button pressed");
            // Logic moved to PumpController::processManualPumpToggle
            // Get current water level and min level for manual control from NVS
            SensorData *currentSensorData = NULL;
            uint8_t currentWaterLevel = 0; // Default to 0 if no data
            int millisWaited = 0;
            while (sensorDataRepo.getSensorData(currentSensorData) != DATA_FETCH_SUCCESS && millisWaited <= 3000)
            {
                vTaskDelay(pdMS_TO_TICKS(100));
                millisWaited += 100;
            }
            if (currentSensorData != NULL)
            {
                currentWaterLevel = currentSensorData->waterLevel;
                free(currentSensorData); // Free data obtained for this check
                currentSensorData = NULL;
            }
            else
            {
                ESP_LOGE(TAG, "Motor toggle: Could not fetch current sensor data.");
                // Decide if to proceed or play failure, for now, assume 0 level if no data.
                // This might need a more robust handling (e.g. play failure, don't operate)
            }

            uint8_t minLevelForManualCtrl = 0; // Default if NVS fails
            // nvsManager.getMinWaterLevelToAllowPumpControl(minLevelForManualCtrl); // Ignoring NVS read failure for now for simplicity

            uint64_t currentTimestamp = 0;
            wifiManager.getTimestampMillis(currentTimestamp);

            pumpController.processManualPumpToggle(currentWaterLevel, minLevelForManualCtrl, currentTimestamp);
            motorToggleRoutineRunning = false;
            // Original logic:
            // if (peripheralManager.getCurrentMotorState() == ON)
            // {
            //     turnWaterPumpOff(); // Now pumpController.turnWaterPumpOff()
            //     motorToggleRoutineRunning = false;
            //     return;
            // }
            // else if (peripheralManager.getCurrentMotorState() == OFF)
            // {
            //     uint8_t wlAllowedPumpCtrl = 0;
            //     if (nvsManager.getMinWaterLevelToAllowPumpControl(wlAllowedPumpCtrl) == false)
            //     {
            //         motorToggleRoutineRunning = false;
            //         return;
            //     }
            // SensorData *sensorData = NULL;
            // int millisWaited = 0;
            // while (sensorDataRepo.getSensorData(sensorData) != DATA_FETCH_SUCCESS && millisWaited <= 3000)
            // {
            //     vTaskDelay(pdMS_TO_TICKS(100));
            //     millisWaited += 100;
            // }
            // if (millisWaited > 3000 && sensorData == NULL)
            // {
            //     ESP_LOGE(TAG, "Could't fetch sensor Data");
            //     motorToggleRoutineRunning = false;
            //     return;
            // }
            // uint8_t currentWaterLevel = sensorData->waterLevel;
            // free(sensorData);

            // // Step 3 : if the current water level is above the wl for allowed pump control then send an error
            // // TODO
            // // if (currentWaterLevel > wlAllowedPumpCtrl)
            // // {
            // //     // Play failure buzzer
            // //     motorToggleRoutineRunning = false;
            // //     return ;
            // // }

            // // Step 4 : If execution flow reches here that mean water pump can be switched on.
            // turnWaterPumpOn(); // Now pumpController.turnWaterPumpOn()
            // manualPumpRestartRequired = false;
            // ESP_LOGI(TAG, "Pump manually started; automatic control re-enabled.");
            // }
            // motorToggleRoutineRunning = false;
        });

    //////////////////////////////////////////////////////////////////////////
    // Setup I2C Manager, UART Manager
    //////////////////////////////////////////////////////////////////////////

    // Setup SensorDataRepo callback using the global instance
    sensorDataRepo.setOnSensorDataUpdatedListener([&](const SensorData &newData)
                                                  {
                                                      // Raw sensor logging moved to main loop (combined with filtered data)
                                                      // This reduces log clutter and shows both raw and filtered values together

                                                      ledIndication.updateDataReceptionStatus(DataReceptionAppStatus::DATA_RECEPTION_OK);

                                                      // Water level publishing is now handled in the main loop to avoid redundant updates.
                                                  });

    uartManager.setDataReceptionTimeoutListener(
        [&]() -> void
        {
            // CRITICAL FIX: Defer heavy operations to main task to prevent IDLE task starvation
            // This callback runs in uartTimeoutTask context - keep it minimal
            // Signal main loop via event group to handle MQTT publish, buzzer, LED updates
            xEventGroupSetBits(app_event_group, UART_TIMEOUT_EVENT_BIT);
        });

    uartManager.initReceiverPort(
        // Uart Data Received Listener Callback
        // The boolean value returned by this callback tells the uart manager
        // whether to reset the dataReceptionTimer.
        [&](char *data, size_t length) -> bool
        {
            if (data == NULL || length == 0)
                return false;

            if (data[0] == TYPE_SENSOR_DATA && length == sizeof(SensorData))
            {
                // Calculate the checksum of the data received.
                // if the size of the sensor data received is N bytes then
                // the last (Nth) byte shall contain the checksum.
                uint8_t checksum = sensorDataRepo.getChecksum(data, length - 1);
                if (checksum == data[length - 1])
                {

                    // Create an aligned SensorData struct and copy the data to it
                    // This prevents LoadStoreAlignment errors when accessing the struct members
                    SensorData sensorData;
                    memcpy(&sensorData, data, length);

                    // ESP_LOGI(TAG, "%" PRIu32 ") T : %" PRIu8 " | H : %" PRIu8 " | WL : %" PRIu8 " | Check : %" PRIx8 " | Device ID : " DEVICE_ID_STR,
                    //          (uint32_t)sensorData.sequenceNumber,
                    //          sensorData.temperatureCelsius,
                    //          sensorData.humidity,
                    //          sensorData.waterLevel,
                    //          sensorData.checksum,
                    //          DEVICE_ID2STR(sensorData.deviceID));
                    sensorDataRepo.updateSensorData(&sensorData);
                }
                else
                {
                    ESP_LOGE(TAG, "Incorrect Checksum | expected : 0x%02x , calculated : 0x%02x !", data[length - 1], checksum);
                }
                return true;
            }
            else if (data[0] == TYPE_REQUEST_DATA)
            {
                ESP_LOGW(TAG, "Received request data of (%d) bytes", length);
                return true;
            }
            else if (data[0] == TYPE_RESPONSE_DATA)
            {
                ESP_LOGW(TAG, "Received response data of (%d) bytes", length);
                return true;
            }
            else
            {
                // Reduce unknown data logging to prevent UART flooding
                static uint32_t unknownDataCounter = 0;
                if (++unknownDataCounter % 25 == 0) {
                    ESP_LOGD(TAG, "Unknown data packets: %" PRIu32, unknownDataCounter);
                }
            }

            return false;
        });

    uartManager.resetRadioModule();

    // uint8_t smallVal = 0;
    // uint32_t bigVal = 0;

    // if (nvsManager.getDataReceptionTimeout(bigVal) != false)
    //     ESP_LOGI(TAG, "Stored DataReceptionTimeout : %" PRIu32, bigVal);
    // if (nvsManager.getMinWaterLevelToAllowPumpControl(smallVal) != false)
    //     ESP_LOGI(TAG, "Stored MinWaterLevelToAllowPumpControl : %" PRIu8, smallVal);
    // if (nvsManager.getMinWaterLevelToStartPump(smallVal) != false)
    //     ESP_LOGI(TAG, "Stored MinWaterLevelToStartPump : %" PRIu8, smallVal);
    // if (nvsManager.getMaxWaterLevelToStopPump(smallVal) != false)
    //     ESP_LOGI(TAG, "Stored MaxWaterLevelToStopPump : %" PRIu8, smallVal);

    uint8_t tempMinLevel, tempMaxLevel, tempMinAllowControl;
    if (nvsManager.getMinWaterLevelToStartPump(tempMinLevel))
    {
        pumpController.updateMinWaterLevelToStartPump(tempMinLevel);
        initial_min_level_start_pump = tempMinLevel;
        ESP_LOGI(TAG, "Loaded MinWaterLevelToStartPump from NVS: %d", tempMinLevel);
    }
    else
    {
        pumpController.updateMinWaterLevelToStartPump(initial_min_level_start_pump);
        nvsManager.setMinWaterLevelToStartPump(initial_min_level_start_pump);
        ESP_LOGI(TAG, "Using default MinWaterLevelToStartPump: %d", initial_min_level_start_pump);
    }

    if (nvsManager.getMaxWaterLevelToStopPump(tempMaxLevel))
    {
        pumpController.updateMaxWaterLevelToStopPump(tempMaxLevel);
        initial_max_level_stop_pump = tempMaxLevel;
        ESP_LOGI(TAG, "Loaded MaxWaterLevelToStopPump from NVS: %d", tempMaxLevel);
    }
    else
    {
        pumpController.updateMaxWaterLevelToStopPump(initial_max_level_stop_pump);
        nvsManager.setMaxWaterLevelToStopPump(initial_max_level_stop_pump);
        ESP_LOGI(TAG, "Using default MaxWaterLevelToStopPump: %d", initial_max_level_stop_pump);
    }

    if (nvsManager.getMinWaterLevelToAllowPumpControl(tempMinAllowControl))
    {
        ESP_LOGI(TAG, "Loaded MinWaterLevelToAllowPumpControl from NVS: %d", tempMinAllowControl);
    }
    else
    {
        nvsManager.setMinWaterLevelToAllowPumpControl(0); // Default value
        ESP_LOGI(TAG, "Using default MinWaterLevelToAllowPumpControl: 0");
    }

    uint32_t manualLockoutDuration = nvsManager.getManualLockoutDurationMs();
    ESP_LOGI(TAG, "Loaded ManualLockoutDurationMs from NVS: %" PRIu32, manualLockoutDuration);

    // SPPR is now managed by Kalman filter state (kf_x_hat), not a separate parameter
    float sppr_value = 0.0f;
    if (nvsManager.getKalmanStateXHat(sppr_value))
    {
        ESP_LOGI(TAG, "Loaded SPPR from Kalman filter state: %.2f s/%%", sppr_value);
    }
    else
    {
        ESP_LOGI(TAG, "No SPPR found in NVS (first boot or cleared)");
    }

    //////////////////////////////////////////////////////////////////////////
    // Sync PumpController with Hardware State
    //////////////////////////////////////////////////////////////////////////

    // After all initialization is complete, sync the software state with hardware
    // This ensures that if the pump was running before ESP32 restart, all monitoring
    // systems will be properly activated
    ESP_LOGI(TAG, "Performing hardware state synchronization...");
    pumpController.syncWithHardwareState();

    //////////////////////////////////////////////////////////////////////////
    // Setup HttpServer Callbacks
    //////////////////////////////////////////////////////////////////////////

    httpsServer.setCredentialsPostCallback(
        [&](string ssid, string password, bool connect, char *&jsonResultStringHolder) -> bool
        {
            // The return type of this callback function represents whether to
            // return an http 200 OK (if true) or https 500 error if false.
            WiFiCredentials credential;
            memcpy(credential.ssid, ssid.c_str(), ssid.length() + 1);
            memcpy(credential.password, password.c_str(), password.length() + 1);
            // Case 1. when credential couldn't be added
            if (nvsManager.addCredential(credential) == false)
                return false;
            cJSON *root = cJSON_CreateObject();
            // case 2. When Credentials have been added but connect is false
            if (connect == false)
            {
                // Prepare the JSON message for this
                cJSON_AddStringToObject(root, "connect", "false");
                cJSON_AddStringToObject(root, "success", "true");
                cJSON_AddStringToObject(root, "message", "Credentials added successfully !");
                jsonResultStringHolder = cJSON_PrintUnformatted(root);
                cJSON_Delete(root);
                return true;
            }
            // Now try connecting to the wifi access point
            // case 3. When the connection to the access point was not a success
            if (wifiManager.testConnection(ssid, password) == false)
            {
                // Prepare a JSON message for this along with a reason code
                cJSON_AddStringToObject(root, "connect", "true");
                cJSON_AddStringToObject(root, "success", "true");
                cJSON_AddStringToObject(root, "connected", "false");
                cJSON_AddStringToObject(root, "message", "Credentials added successfully, but failed to connect to Access Point !");
                char *disconnectionReasonJsonString = NULL;
                wifiManager.getDisconnectionReasonInJson(disconnectionReasonJsonString);
                cJSON *disconnectionReason = cJSON_Parse(disconnectionReasonJsonString);
                cJSON_AddItemToObject(root, "reason", disconnectionReason);
                jsonResultStringHolder = cJSON_PrintUnformatted(root);
                cJSON_Delete(root);
                // Free the intermediate JSON string (Requirement 6.1)
                if (disconnectionReasonJsonString != NULL)
                {
                    free(disconnectionReasonJsonString);
                }
                return true;
            }
            // case 4. When connection was a success
            // Prepare a JSON message for this along with the connection details
            cJSON_AddStringToObject(root, "connect", "true");
            cJSON_AddStringToObject(root, "success", "true");
            cJSON_AddStringToObject(root, "connected", "true");
            cJSON_AddStringToObject(root, "message", "Credentials added successfully, and connect to Access Point !");
            char *connectionDataJsonString = NULL;
            wifiManager.getApConnectionStatusInJson(connectionDataJsonString);
            cJSON *connectionData = cJSON_Parse(connectionDataJsonString);
            cJSON_AddItemToObject(root, "connectionData", connectionData);
            jsonResultStringHolder = cJSON_PrintUnformatted(root);
            ESP_LOGW(TAG, "Connected to AP | connection data : \n%s\n", jsonResultStringHolder);
            cJSON_Delete(root);
            // Free the intermediate JSON string (Requirement 6.1)
            if (connectionDataJsonString != NULL)
            {
                free(connectionDataJsonString);
            }
            return true;
        });
    httpsServer.setCredentialDeleteCallback(
        [&](string ssid) -> bool
        {
            return nvsManager.deleteCredentialWithSSID(ssid);
        });
    httpsServer.setCredentialsGetCallback(
        [&](char *&jsonStringResultHolder) -> bool
        {
            return nvsManager.getSavedCredentialsInJson(jsonStringResultHolder);
        });
    httpsServer.setWiFiAPScanCallback(
        [&](char *&jsonStringResultHolder) -> bool
        {
            return wifiManager.getWiFiScanResultInJson(jsonStringResultHolder);
        });
    httpsServer.setDeviceInfoCallback(
        [&](char *&jsonStringResultHolder) -> bool
        {
            jsonStringResultHolder = NULL;
            
            cJSON *root = cJSON_CreateObject();
            if (root == NULL)
            {
                ESP_LOGE(TAG, "Failed to create JSON object for device info");
                return false;
            }
            
            // Device ID (MAC-based)
            char deviceIDStr[DEVICE_ID_BUFFER_LEN];
            getDeviceID(deviceIDStr, DEVICE_ID_BUFFER_LEN);
            cJSON_AddStringToObject(root, "deviceId", deviceIDStr);
            
            // Chip information
            esp_chip_info_t chipInfo;
            esp_chip_info(&chipInfo);
            cJSON_AddStringToObject(root, "chipModel", chipModelToString(chipInfo.model));
            cJSON_AddNumberToObject(root, "cores", chipInfo.cores);
            cJSON_AddNumberToObject(root, "revision", chipInfo.revision);
            
            // Firmware version
            cJSON *appDesc = NULL;
            if (otaManager.getCurrentAppDescription(appDesc))
            {
                cJSON_AddItemToObject(root, "firmware", appDesc);
            }
            
            // System status
            cJSON_AddNumberToObject(root, "freeHeap", esp_get_free_heap_size());
            cJSON_AddNumberToObject(root, "uptimeMs", esp_timer_get_time() / 1000);
            
            // WiFi status
            cJSON_AddBoolToObject(root, "wifiConnected", wifiManager.isWiFiConnected());
            
            // MQTT status
            cJSON_AddBoolToObject(root, "mqttConnected", mqttManager.isConnected());
            
            jsonStringResultHolder = cJSON_PrintUnformatted(root);
            cJSON_Delete(root);
            
            if (jsonStringResultHolder == NULL)
            {
                ESP_LOGE(TAG, "Failed to print JSON for device info");
                return false;
            }
            
            return true;
        });
    httpsServer.setWiFiStatusCallback(
        [&](char *&jsonStringResultHolder) -> bool
        {
            return wifiManager.getApConnectionStatusInJson(jsonStringResultHolder);
        });
    httpsServer.setWifiApDisconnectCallback(
        [&]() -> bool
        {
            return wifiManager.disconnectFromApSync();
        });
    httpsServer.setExitConfigModeCallback(
        [&]() -> void
        {
            ESP_LOGW(TAG, "Exiting config mode and restarting device...");
            xTaskCreatePinnedToCore(
                delayed_restart_task,
                "restart_task",
                RESTART_TASK_STACK_DEPTH,
                (void *)RESTART_DELAY_MS,
                5,
                NULL,
                APP_CPU_NUM);
        });

    httpsServer.setHttpEventCallback([&]()
                                     { ledIndication.triggerHttpEventIndication(); });

    // System configuration callbacks
    httpsServer.setSystemConfigGetCallback(
        [&](char *&jsonResultStringHolder) -> bool
        {
            cJSON *root = cJSON_CreateObject();
            if (root == NULL)
            {
                ESP_LOGE(TAG, "Failed to create JSON object for system config");
                return false;
            }

            // Get data reception timeout from NVS (stored in milliseconds, return in seconds)
            uint32_t timeoutMs = 0;
            if (nvsManager.getDataReceptionTimeout(timeoutMs))
            {
                uint32_t timeoutSec = timeoutMs / 1000;
                cJSON_AddNumberToObject(root, "data_reception_timeout_sec", timeoutSec);
            }
            else
            {
                // Default timeout if not found in NVS
                cJSON_AddNumberToObject(root, "data_reception_timeout_sec", 30);
            }

            // Get manual lockout duration from NVS (stored in milliseconds, return in minutes)
            uint32_t lockoutMs = nvsManager.getManualLockoutDurationMs();
            uint32_t lockoutMin = lockoutMs / (60 * 1000);
            cJSON_AddNumberToObject(root, "manualLockoutDuration", lockoutMin);

            // Get lockout feature configuration (Requirements 8.1, 8.2, 8.3)
            LockoutConfiguration lockoutConfig = pumpController.getLockoutConfiguration();
            cJSON_AddBoolToObject(root, "lockout_feature_enabled", lockoutConfig.lockoutFeatureEnabled);
            cJSON_AddNumberToObject(root, "lockout_duration_ms", lockoutConfig.lockoutDurationMs);
            cJSON_AddBoolToObject(root, "lockout_currently_active", lockoutConfig.lockoutCurrentlyActive);

            jsonResultStringHolder = cJSON_PrintUnformatted(root);
            cJSON_Delete(root);

            if (jsonResultStringHolder == NULL)
            {
                ESP_LOGE(TAG, "Failed to generate JSON string for system config");
                return false;
            }

            return true;
        });

    httpsServer.setSystemConfigPostCallback(
        [&](const char *jsonRequest, char *&jsonResultStringHolder) -> bool
        {
            if (jsonRequest == NULL)
            {
                ESP_LOGE(TAG, "System config POST: NULL request");
                return false;
            }

            cJSON *requestJson = cJSON_Parse(jsonRequest);
            if (requestJson == NULL)
            {
                ESP_LOGE(TAG, "System config POST: Invalid JSON");
                return false;
            }

            bool success = true;
            cJSON *responseRoot = cJSON_CreateObject();
            cJSON *appliedConfig = cJSON_CreateObject();

            // Process data_reception_timeout_sec
            cJSON *timeoutItem = cJSON_GetObjectItem(requestJson, "data_reception_timeout_sec");
            if (timeoutItem != NULL && cJSON_IsNumber(timeoutItem))
            {
                uint32_t timeoutSec = (uint32_t)timeoutItem->valueint;

                // Validate timeout range (5 seconds to 300 seconds)
                if (timeoutSec >= 5 && timeoutSec <= 300)
                {
                    uint32_t timeoutMs = timeoutSec * 1000;
                    if (nvsManager.setDataReceptionTimeout(timeoutMs))
                    {
                        cJSON_AddNumberToObject(appliedConfig, "data_reception_timeout_sec", timeoutSec);
                        ESP_LOGI(TAG, "Updated data reception timeout to %" PRIu32 " seconds", timeoutSec);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to save data reception timeout to NVS");
                        success = false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid data reception timeout: %" PRIu32 " (must be 5-300 seconds)", timeoutSec);
                    success = false;
                }
            }

            // Process manualLockoutDuration
            cJSON *lockoutItem = cJSON_GetObjectItem(requestJson, "manualLockoutDuration");
            if (lockoutItem != NULL && cJSON_IsNumber(lockoutItem))
            {
                uint32_t lockoutMin = (uint32_t)lockoutItem->valueint;

                // Validate lockout duration range (30 minutes to 300 minutes / 5 hours)
                if (lockoutMin >= 30 && lockoutMin <= 300)
                {
                    uint32_t lockoutMs = lockoutMin * 60 * 1000;
                    esp_err_t err = nvsManager.setManualLockoutDurationMs(lockoutMs);
                    if (err == ESP_OK)
                    {
                        cJSON_AddNumberToObject(appliedConfig, "manualLockoutDuration", lockoutMin);
                        ESP_LOGI(TAG, "Updated manual lockout duration to %" PRIu32 " minutes", lockoutMin);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to save manual lockout duration to NVS");
                        success = false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid manual lockout duration: %" PRIu32 " (must be 30-300 minutes)", lockoutMin);
                    success = false;
                }
            }

            // Generate response
            if (success)
            {
                cJSON_AddStringToObject(responseRoot, "message", "System config updated");
                cJSON_AddItemToObject(responseRoot, "applied", appliedConfig);
            }
            else
            {
                cJSON_AddStringToObject(responseRoot, "message", "Failed to update system config");
                cJSON_Delete(appliedConfig);
            }

            jsonResultStringHolder = cJSON_PrintUnformatted(responseRoot);
            cJSON_Delete(responseRoot);
            cJSON_Delete(requestJson);

            if (jsonResultStringHolder == NULL)
            {
                ESP_LOGE(TAG, "Failed to generate JSON response for system config");
                return false;
            }

            return success;
        });

    // Pump configuration callbacks
    httpsServer.setPumpConfigGetCallback(
        [&](char *&jsonResultStringHolder) -> bool
        {
            cJSON *root = cJSON_CreateObject();
            if (root == NULL)
            {
                ESP_LOGE(TAG, "Failed to create JSON object for pump config");
                return false;
            }

            // Get pump configuration values from NVS
            uint8_t minLevelStart = 0;
            uint8_t maxLevelStop = 0;
            uint8_t minLevelAllowControl = 0;

            if (nvsManager.getMinWaterLevelToStartPump(minLevelStart))
            {
                cJSON_AddNumberToObject(root, "min_water_level_start_pump_cm", minLevelStart);
            }
            else
            {
                // Use default value if not found in NVS
                cJSON_AddNumberToObject(root, "min_water_level_start_pump_cm", initial_min_level_start_pump);
            }

            if (nvsManager.getMaxWaterLevelToStopPump(maxLevelStop))
            {
                cJSON_AddNumberToObject(root, "max_water_level_stop_pump_cm", maxLevelStop);
            }
            else
            {
                // Use default value if not found in NVS
                cJSON_AddNumberToObject(root, "max_water_level_stop_pump_cm", initial_max_level_stop_pump);
            }

            if (nvsManager.getMinWaterLevelToAllowPumpControl(minLevelAllowControl))
            {
                cJSON_AddNumberToObject(root, "min_water_level_allow_control_cm", minLevelAllowControl);
            }
            else
            {
                // Use default value if not found in NVS
                cJSON_AddNumberToObject(root, "min_water_level_allow_control_cm", 0);
            }

            // Get SPPR from Kalman filter state
            float sppr_value = 0.0f;
            if (nvsManager.getKalmanStateXHat(sppr_value))
            {
                cJSON_AddNumberToObject(root, "max_seconds_per_percent_rise", (int)sppr_value);
            }
            else
            {
                // Use default value if not found in NVS
                cJSON_AddNumberToObject(root, "max_seconds_per_percent_rise", 15);
            }

            // Get lockout duration from NVS (in milliseconds)
            uint32_t lockoutDuration = nvsManager.getManualLockoutDurationMs();
            cJSON_AddNumberToObject(root, "lockout_duration_ms", lockoutDuration);

            // Get data reception timeout from NVS (in milliseconds)
            uint32_t dataReceptionTimeout = 0;
            if (nvsManager.getDataReceptionTimeout(dataReceptionTimeout))
            {
                cJSON_AddNumberToObject(root, "data_reception_timeout_ms", dataReceptionTimeout);
            }
            else
            {
                // Use default value if not found in NVS (30 seconds = 30000 ms)
                cJSON_AddNumberToObject(root, "data_reception_timeout_ms", 30000);
            }

            jsonResultStringHolder = cJSON_PrintUnformatted(root);
            cJSON_Delete(root);

            if (jsonResultStringHolder == NULL)
            {
                ESP_LOGE(TAG, "Failed to generate JSON string for pump config");
                return false;
            }

            return true;
        });

    httpsServer.setPumpConfigPostCallback(
        [&](const char *jsonRequest, char *&jsonResultStringHolder) -> bool
        {
            if (jsonRequest == NULL)
            {
                ESP_LOGE(TAG, "Pump config POST: NULL request");
                return false;
            }

            cJSON *requestJson = cJSON_Parse(jsonRequest);
            if (requestJson == NULL)
            {
                ESP_LOGE(TAG, "Pump config POST: Invalid JSON");
                return false;
            }

            bool success = true;
            cJSON *responseRoot = cJSON_CreateObject();
            cJSON *appliedConfig = cJSON_CreateObject();

            // Process min_water_level_start_pump_cm
            cJSON *minStartItem = cJSON_GetObjectItem(requestJson, "min_water_level_start_pump_cm");
            if (minStartItem != NULL && cJSON_IsNumber(minStartItem))
            {
                uint8_t minStart = (uint8_t)minStartItem->valueint;

                // Validate range (0-100 cm)
                if (minStart <= 100)
                {
                    if (nvsManager.setMinWaterLevelToStartPump(minStart))
                    {
                        pumpController.updateMinWaterLevelToStartPump(minStart);
                        cJSON_AddNumberToObject(appliedConfig, "min_water_level_start_pump_cm", minStart);
                        ESP_LOGI(TAG, "Updated min water level to start pump to %d cm", minStart);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to save min water level to start pump to NVS");
                        success = false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid min water level to start pump: %d (must be 0-100 cm)", minStart);
                    success = false;
                }
            }

            // Process max_water_level_stop_pump_cm
            cJSON *maxStopItem = cJSON_GetObjectItem(requestJson, "max_water_level_stop_pump_cm");
            if (maxStopItem != NULL && cJSON_IsNumber(maxStopItem))
            {
                uint8_t maxStop = (uint8_t)maxStopItem->valueint;

                // Validate range (0-100 cm)
                if (maxStop <= 100)
                {
                    if (nvsManager.setMaxWaterLevelToStopPump(maxStop))
                    {
                        pumpController.updateMaxWaterLevelToStopPump(maxStop);
                        cJSON_AddNumberToObject(appliedConfig, "max_water_level_stop_pump_cm", maxStop);
                        ESP_LOGI(TAG, "Updated max water level to stop pump to %d cm", maxStop);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to save max water level to stop pump to NVS");
                        success = false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid max water level to stop pump: %d (must be 0-100 cm)", maxStop);
                    success = false;
                }
            }

            // Process min_water_level_allow_control_cm
            cJSON *minControlItem = cJSON_GetObjectItem(requestJson, "min_water_level_allow_control_cm");
            if (minControlItem != NULL && cJSON_IsNumber(minControlItem))
            {
                uint8_t minControl = (uint8_t)minControlItem->valueint;

                // Validate range (0-100 cm)
                if (minControl <= 100)
                {
                    if (nvsManager.setMinWaterLevelToAllowPumpControl(minControl))
                    {
                        cJSON_AddNumberToObject(appliedConfig, "min_water_level_allow_control_cm", minControl);
                        ESP_LOGI(TAG, "Updated min water level to allow pump control to %d cm", minControl);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to save min water level to allow pump control to NVS");
                        success = false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid min water level to allow pump control: %d (must be 0-100 cm)", minControl);
                    success = false;
                }
            }

            // Process max_seconds_per_percent_rise (SPPR)
            cJSON *maxSecondsItem = cJSON_GetObjectItem(requestJson, "max_seconds_per_percent_rise");
            if (maxSecondsItem != NULL && cJSON_IsNumber(maxSecondsItem))
            {
                uint32_t maxSeconds = (uint32_t)maxSecondsItem->valueint;

                // Validate range (1-100 seconds per percent)
                if (maxSeconds >= 1 && maxSeconds <= 100)
                {
                    // Apply ceiling and save to Kalman filter state
                    float sppr_value = ceilf((float)maxSeconds);
                    if (nvsManager.setKalmanStateXHat(sppr_value))
                    {
                        cJSON_AddNumberToObject(appliedConfig, "max_seconds_per_percent_rise", (int)sppr_value);
                        ESP_LOGI(TAG, "Updated SPPR to %.0f s/%%", sppr_value);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to save SPPR to NVS");
                        success = false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid SPPR: %" PRIu32 " (must be 1-100 s/%%)", maxSeconds);
                    success = false;
                }
            }

            // Process lockout_duration_ms
            cJSON *lockoutDurationItem = cJSON_GetObjectItem(requestJson, "lockout_duration_ms");
            if (lockoutDurationItem != NULL && cJSON_IsNumber(lockoutDurationItem))
            {
                uint32_t lockoutDuration = (uint32_t)lockoutDurationItem->valueint;

                // Validate range (30 min to 5 hours in milliseconds)
                if (lockoutDuration >= MANUAL_LOCKOUT_DURATION_MS_MIN && lockoutDuration <= MANUAL_LOCKOUT_DURATION_MS_MAX)
                {
                    if (nvsManager.setManualLockoutDurationMs(lockoutDuration) == ESP_OK)
                    {
                        pumpController.updateManualLockoutDuration(lockoutDuration);
                        cJSON_AddNumberToObject(appliedConfig, "lockout_duration_ms", lockoutDuration);
                        ESP_LOGI(TAG, "Updated lockout duration to %" PRIu32 " ms", lockoutDuration);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to save lockout duration to NVS");
                        success = false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid lockout duration: %" PRIu32 " (must be %d-%d ms)", 
                             lockoutDuration, MANUAL_LOCKOUT_DURATION_MS_MIN, MANUAL_LOCKOUT_DURATION_MS_MAX);
                    success = false;
                }
            }

            // Process data_reception_timeout_ms
            cJSON *dataTimeoutItem = cJSON_GetObjectItem(requestJson, "data_reception_timeout_ms");
            if (dataTimeoutItem != NULL && cJSON_IsNumber(dataTimeoutItem))
            {
                uint32_t dataTimeout = (uint32_t)dataTimeoutItem->valueint;

                // Validate range (5 seconds to 300 seconds in milliseconds)
                if (dataTimeout >= 5000 && dataTimeout <= 300000)
                {
                    if (nvsManager.setDataReceptionTimeout(dataTimeout))
                    {
                        cJSON_AddNumberToObject(appliedConfig, "data_reception_timeout_ms", dataTimeout);
                        ESP_LOGI(TAG, "Updated data reception timeout to %" PRIu32 " ms", dataTimeout);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to save data reception timeout to NVS");
                        success = false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid data reception timeout: %" PRIu32 " (must be 5000-300000 ms)", dataTimeout);
                    success = false;
                }
            }

            // Additional validation: ensure min start level < max stop level
            if (success && minStartItem != NULL && maxStopItem != NULL)
            {
                uint8_t minStart = (uint8_t)minStartItem->valueint;
                uint8_t maxStop = (uint8_t)maxStopItem->valueint;

                if (minStart >= maxStop)
                {
                    ESP_LOGE(TAG, "Invalid pump configuration: min start level (%d) must be less than max stop level (%d)", minStart, maxStop);
                    success = false;
                }
            }

            // Generate response
            if (success)
            {
                cJSON_AddStringToObject(responseRoot, "message", "Pump config updated");
                cJSON_AddItemToObject(responseRoot, "applied", appliedConfig);
            }
            else
            {
                cJSON_AddStringToObject(responseRoot, "message", "Failed to update pump config");
                cJSON_Delete(appliedConfig);
            }

            jsonResultStringHolder = cJSON_PrintUnformatted(responseRoot);
            cJSON_Delete(responseRoot);
            cJSON_Delete(requestJson);

            if (jsonResultStringHolder == NULL)
            {
                ESP_LOGE(TAG, "Failed to generate JSON response for pump config");
                return false;
            }

            return success;
        });

    // RF Config GET callback
    httpsServer.setRfConfigGetCallback(
        [&](char *&jsonResultStringHolder) -> bool
        {
            // First, check if RF module is responding
            if (!uartManager.isRFModuleResponding())
            {
                ESP_LOGE(TAG, "RF Config GET: RF module not responding");
                
                // Create error response
                cJSON *errorRoot = cJSON_CreateObject();
                cJSON_AddStringToObject(errorRoot, "error", "RF module not responding");
                cJSON_AddStringToObject(errorRoot, "message", "Unable to communicate with RF module. Check connection and ensure module is powered on.");
                
                jsonResultStringHolder = cJSON_PrintUnformatted(errorRoot);
                cJSON_Delete(errorRoot);
                
                return false;  // Return false to trigger 500 Internal Server Error
            }
            
            ESP_LOGI(TAG, "RF Config GET: RF module is responding, querying parameters...");
            
            // Query all RF module parameters
            RFModuleAirDataRate airDataRate = uartManager.getRFModuleAirDataRate();
            RFModuleBaudrate baudrate = uartManager.getRFModuleBaudrate();
            RFModuleCarrierFrequency carrierFreq = uartManager.getRFModuleCarrierFrequency();
            RFModulePowerTransmitLevel powerLevel = uartManager.getRFModulePowerTransmitLevel();
            RFModuleSignalStrengthLimit signalStrength = uartManager.getRFModuleSignalStrengthLimit();
            uint16_t hardwareID = uartManager.getRFModuleHardwareID();
            uint16_t networkID = uartManager.getRFModuleNetworkID();
            uint16_t destinationID = uartManager.getRFModuleDestinationID();
            char *encryptionKey = uartManager.getRFModule128bitEncryptionKey();
            
            // Validate that we got valid responses (not MAX/error values)
            if (airDataRate == DATA_RATE_MAX || baudrate == BAUD_MAX || 
                carrierFreq == FREQ_MAX || powerLevel == POWER_LEVEL_MAX || 
                signalStrength == SIGNAL_STRENGTH_MAX ||
                hardwareID == 0xFFFF || networkID == 0xFFFF || destinationID == 0xFFFF)
            {
                ESP_LOGE(TAG, "RF Config GET: Failed to retrieve valid configuration from RF module");
                
                // Create error response
                cJSON *errorRoot = cJSON_CreateObject();
                cJSON_AddStringToObject(errorRoot, "error", "RF module communication failed");
                cJSON_AddStringToObject(errorRoot, "message", "RF module responded but returned invalid data. Check module configuration and connection.");
                
                jsonResultStringHolder = cJSON_PrintUnformatted(errorRoot);
                cJSON_Delete(errorRoot);
                
                if (encryptionKey != NULL) free(encryptionKey);
                
                return false;  // Return false to trigger 500 Internal Server Error
            }
            
            cJSON *root = cJSON_CreateObject();
            
            // Add all parameters to JSON
            cJSON_AddNumberToObject(root, "air_data_rate", (int)airDataRate);
            cJSON_AddNumberToObject(root, "baudrate", (int)baudrate);
            cJSON_AddNumberToObject(root, "carrier_frequency", (int)carrierFreq);
            cJSON_AddNumberToObject(root, "power_transmit_level", (int)powerLevel);
            cJSON_AddNumberToObject(root, "signal_strength_limit", (int)signalStrength);
            
            char hwIdStr[8];
            snprintf(hwIdStr, sizeof(hwIdStr), "%04X", hardwareID);
            cJSON_AddStringToObject(root, "hardware_id", hwIdStr);
            
            char netIdStr[8];
            snprintf(netIdStr, sizeof(netIdStr), "%04X", networkID);
            cJSON_AddStringToObject(root, "network_id", netIdStr);
            
            char destIdStr[8];
            snprintf(destIdStr, sizeof(destIdStr), "%04X", destinationID);
            cJSON_AddStringToObject(root, "destination_id", destIdStr);
            
            if (encryptionKey != NULL)
            {
                cJSON_AddStringToObject(root, "encryption_key", encryptionKey);
                free(encryptionKey);
            }
            else
            {
                cJSON_AddStringToObject(root, "encryption_key", "");
            }
            
            jsonResultStringHolder = cJSON_PrintUnformatted(root);
            cJSON_Delete(root);
            
            ESP_LOGI(TAG, "RF Config GET: Successfully retrieved configuration");
            return (jsonResultStringHolder != NULL);
        });

    // RF Config POST callback
    httpsServer.setRfConfigPostCallback(
        [&](const char *jsonRequest, char *&jsonResultStringHolder) -> bool
        {
            if (jsonRequest == NULL)
            {
                ESP_LOGE(TAG, "RF config POST: NULL request");
                return false;
            }
            
            // First, check if RF module is responding
            if (!uartManager.isRFModuleResponding())
            {
                ESP_LOGE(TAG, "RF Config POST: RF module not responding");
                
                // Create error response
                cJSON *errorRoot = cJSON_CreateObject();
                cJSON_AddStringToObject(errorRoot, "error", "RF module not responding");
                cJSON_AddStringToObject(errorRoot, "message", "Unable to communicate with RF module. Cannot update configuration.");
                
                jsonResultStringHolder = cJSON_PrintUnformatted(errorRoot);
                cJSON_Delete(errorRoot);
                
                return false;  // Return false to trigger 500 Internal Server Error
            }
            
            ESP_LOGI(TAG, "RF Config POST: RF module is responding, processing configuration update...");
            
            cJSON *requestJson = cJSON_Parse(jsonRequest);
            if (requestJson == NULL)
            {
                ESP_LOGE(TAG, "RF config POST: Invalid JSON");
                return false;
            }
            
            bool success = true;
            cJSON *responseRoot = cJSON_CreateObject();
            cJSON *appliedConfig = cJSON_CreateObject();
            
            // Process air_data_rate
            cJSON *airDataRateItem = cJSON_GetObjectItem(requestJson, "air_data_rate");
            if (airDataRateItem != NULL && cJSON_IsNumber(airDataRateItem))
            {
                int value = airDataRateItem->valueint;
                if (value >= 0 && value < DATA_RATE_MAX)
                {
                    uartManager.setRFModuleAirDataRate((RFModuleAirDataRate)value);
                    cJSON_AddNumberToObject(appliedConfig, "air_data_rate", value);
                }
            }
            
            // Process baudrate
            cJSON *baudrateItem = cJSON_GetObjectItem(requestJson, "baudrate");
            if (baudrateItem != NULL && cJSON_IsNumber(baudrateItem))
            {
                int value = baudrateItem->valueint;
                if (value >= 0 && value < BAUD_MAX)
                {
                    uartManager.setRFModuleBaudrate((RFModuleBaudrate)value);
                    cJSON_AddNumberToObject(appliedConfig, "baudrate", value);
                }
            }
            
            // Process carrier_frequency
            cJSON *carrierFreqItem = cJSON_GetObjectItem(requestJson, "carrier_frequency");
            if (carrierFreqItem != NULL && cJSON_IsNumber(carrierFreqItem))
            {
                int value = carrierFreqItem->valueint;
                if (value >= 0 && value < FREQ_MAX)
                {
                    uartManager.setRFModuleCarrierFrequency((RFModuleCarrierFrequency)value);
                    cJSON_AddNumberToObject(appliedConfig, "carrier_frequency", value);
                }
            }
            
            // Process power_transmit_level
            cJSON *powerLevelItem = cJSON_GetObjectItem(requestJson, "power_transmit_level");
            if (powerLevelItem != NULL && cJSON_IsNumber(powerLevelItem))
            {
                int value = powerLevelItem->valueint;
                if (value >= 0 && value < POWER_LEVEL_MAX)
                {
                    uartManager.setRFModulePowerTransmitLevel((RFModulePowerTransmitLevel)value);
                    cJSON_AddNumberToObject(appliedConfig, "power_transmit_level", value);
                }
            }
            
            // Process signal_strength_limit
            cJSON *signalStrengthItem = cJSON_GetObjectItem(requestJson, "signal_strength_limit");
            if (signalStrengthItem != NULL && cJSON_IsNumber(signalStrengthItem))
            {
                int value = signalStrengthItem->valueint;
                if (value >= 0 && value < SIGNAL_STRENGTH_MAX)
                {
                    uartManager.setRFModuleSignalStrengthLimit((RFModuleSignalStrengthLimit)value);
                    cJSON_AddNumberToObject(appliedConfig, "signal_strength_limit", value);
                }
            }
            
            // Process hardware_id
            cJSON *hardwareIdItem = cJSON_GetObjectItem(requestJson, "hardware_id");
            if (hardwareIdItem != NULL && cJSON_IsString(hardwareIdItem))
            {
                uint16_t value = (uint16_t)strtol(hardwareIdItem->valuestring, NULL, 16);
                uartManager.setRFModuleHardwareID(value);
                cJSON_AddStringToObject(appliedConfig, "hardware_id", hardwareIdItem->valuestring);
            }
            
            // Process network_id
            cJSON *networkIdItem = cJSON_GetObjectItem(requestJson, "network_id");
            if (networkIdItem != NULL && cJSON_IsString(networkIdItem))
            {
                uint16_t value = (uint16_t)strtol(networkIdItem->valuestring, NULL, 16);
                uartManager.setRFModuleNetworkID(value);
                cJSON_AddStringToObject(appliedConfig, "network_id", networkIdItem->valuestring);
            }
            
            // Process destination_id
            cJSON *destinationIdItem = cJSON_GetObjectItem(requestJson, "destination_id");
            if (destinationIdItem != NULL && cJSON_IsString(destinationIdItem))
            {
                uint16_t value = (uint16_t)strtol(destinationIdItem->valuestring, NULL, 16);
                uartManager.setRFModuleDestinationID(value);
                cJSON_AddStringToObject(appliedConfig, "destination_id", destinationIdItem->valuestring);
            }
            
            // Process encryption_key
            cJSON *encryptionKeyItem = cJSON_GetObjectItem(requestJson, "encryption_key");
            if (encryptionKeyItem != NULL && cJSON_IsString(encryptionKeyItem))
            {
                char *keyStr = encryptionKeyItem->valuestring;
                if (strlen(keyStr) == 32)
                {
                    uartManager.setRFModule128bitEncryptionKey(keyStr);
                    cJSON_AddStringToObject(appliedConfig, "encryption_key", keyStr);
                }
            }
            
            // Generate response
            if (success)
            {
                cJSON_AddStringToObject(responseRoot, "message", "RF config updated");
                cJSON_AddItemToObject(responseRoot, "applied", appliedConfig);
            }
            else
            {
                cJSON_AddStringToObject(responseRoot, "message", "Failed to update RF config");
                cJSON_Delete(appliedConfig);
            }
            
            jsonResultStringHolder = cJSON_PrintUnformatted(responseRoot);
            cJSON_Delete(responseRoot);
            cJSON_Delete(requestJson);
            
            return (jsonResultStringHolder != NULL);
        });

    //////////////////////////////////////////////////////////////////////////
    // Setup WiFIManager, MQTTManager, OTAManager
    //////////////////////////////////////////////////////////////////////////

    char *version = NULL;
    if (otaManager.getCurrentFirmwareVersion(version))
    {
        if (version != NULL)
        {
            ESP_LOGW(TAG, "Current version : %s", version);
            uint32_t versionNumber = otaManager.getCurrentFirmwareVersionNumber();
            ESP_LOGW(TAG, "Current Firmware Version Number : %" PRIu32, versionNumber);
            free(version);
        }
        else
        {
            ESP_LOGE(TAG, "version string is null");
        }
    }

    otaManager.setOnNewUpdateAvailableListener(
        [&]() -> bool
        {
            // Check if the water pump is currently active
            if (peripheralManager.getCurrentMotorState() == ON)
            {
                ESP_LOGW(TAG, "OTA update available but pump is currently running. Postponing OTA update.");
                return false; // Prevent OTA update while pump is running
            }

            ESP_LOGI(TAG, "OTA update available and pump is not running. Allowing OTA update to proceed.");
            return true; // Allow OTA update when pump is not running
        });
    otaManager.setOnOTAUpdateStartedListener(
        [&](const char* currentVersion, const char* newVersion) -> void
        {
            ESP_LOGW(TAG, "OTA update started ! Current: %s -> New: %s", currentVersion, newVersion);
            
            // Publish OTA started event via mqttEventPublisher for consistent event structure
            cJSON *data = cJSON_CreateObject();
            if (data == NULL)
            {
                ESP_LOGE(TAG, "Failed to create JSON object for OTA started event - out of memory");
                return;
            }
            cJSON_AddStringToObject(data, "current_version", currentVersion ? currentVersion : "unknown");
            cJSON_AddStringToObject(data, "new_version", newVersion ? newVersion : "unknown");
            
            mqttEventPublisher(EVENT_TYPE_OTA_STARTED, EVENT_LEVEL_INFO, data);
            cJSON_Delete(data);
            ESP_LOGI(TAG, "OTA started event published");
        });
    otaManager.setOnOTAUpdateProgressListener(
        [&](uint8_t progress) -> void
        {
            ESP_LOGW(TAG, "OTA update progress %d%%", progress);
            
            // Publish OTA progress event via mqttEventPublisher for consistent event structure
            cJSON *data = cJSON_CreateObject();
            if (data == NULL)
            {
                ESP_LOGE(TAG, "Failed to create JSON object for OTA progress event - out of memory");
                return;
            }
            cJSON_AddNumberToObject(data, "progress", progress);
            
            mqttEventPublisher(EVENT_TYPE_OTA_PROGRESS, EVENT_LEVEL_VERBOSE, data);
            cJSON_Delete(data);
            ESP_LOGI(TAG, "OTA progress event published: %d%%", progress);
        });
    otaManager.setOnOTAUpdateAbortedListener(
        [&](const char* errorReason) -> void
        {
            ESP_LOGW(TAG, "OTA update Aborted ! Reason: %s", errorReason ? errorReason : "Unknown");
            
            // Publish OTA failed event via mqttEventPublisher for consistent event structure
            cJSON *data = cJSON_CreateObject();
            if (data == NULL)
            {
                ESP_LOGE(TAG, "Failed to create JSON object for OTA failed event - out of memory");
                return;
            }
            cJSON_AddStringToObject(data, "reason", errorReason ? errorReason : "Unknown error");
            
            mqttEventPublisher(EVENT_TYPE_OTA_FAILED, EVENT_LEVEL_ALERT, data);
            cJSON_Delete(data);
            ESP_LOGI(TAG, "OTA failed event published");
        });
    otaManager.setOnOTAUpdateCompletedListener(
        [&]() -> void
        {
            ESP_LOGW(TAG, "OTA update completed ! Restarting...");
            
            // Publish OTA complete event via mqttEventPublisher for consistent event structure
            mqttEventPublisher(EVENT_TYPE_OTA_COMPLETED, EVENT_LEVEL_INFO, nullptr);
            ESP_LOGI(TAG, "OTA complete event published");
            // Small delay to allow MQTT message to be sent before restart
            vTaskDelay(pdMS_TO_TICKS(500));
        });

    // Note: MQTT stop/start callbacks for OTA are no longer used.
    // The configuration changes (increased WiFi RX buffers from 4→10, 
    // disabled peer cert caching, increased OTA stack to 12KB) should
    // prevent heap corruption during concurrent TLS operations.
    // The callbacks are kept in OTAManager for future use if needed.

    // Set device ID for OTA endpoint authentication
    otaManager.setDeviceID(deviceID);

    //

    mqttManager.setOnDeviceCommandReceivedListener(
        // valueType : 0x01 - float | 0x02 - integer
        [&](DeviceCommandType commandType, char *responseTopicString, uint32_t *value, uint8_t valueType) -> bool
        {
            ESP_LOGW(TAG, "Command Received from server with response topic : %s | type : %s | valueType : %s",
                     responseTopicString,
                     deviceCommandTypeToString(commandType),
                     valueType == 0x02 ? "Integer" : (valueType == 0x01 ? "Float" : "Invalid Value Type"));

            // Switch through the command types and execute the corresponding command with the necessary response
            switch (commandType)
            {
            case COMMAND_TYPE_CONTROL_SWITCH:
            {
                if (value == NULL)
                {
                    ESP_LOGE(TAG, "Value and Value type needs to be specified for this command");
                    return false;
                }

                // Step 1 : Check if the values provided are valid
                // In this case the value type should be an integer and the
                // value shall be 0xFF for ON and 0x01 for OFF
                if (valueType != 2 || !(*value == 255 || *value == 1))
                {
                    ESP_LOGE(TAG, "Invalid value type provided for this command");
                    return false;
                }
                motorToggleRoutineRunning = true;
                // Step 2 : If the request if to simply turn the water pump off,
                uint64_t cmdTimestamp = 0;
                wifiManager.getTimestampMillis(cmdTimestamp);

                if (*value == 1) // Turn OFF
                {
                    pumpController.turnWaterPumpOff(cmdTimestamp, PUMP_STOP_REASON_REMOTE_COMMAND);
                    motorToggleRoutineRunning = false;
                    return true;
                }
                else if (*value == 255) // Turn ON
                {
                    // Use the centralized method that includes water level check
                    bool success = pumpController.attemptManualPumpStart(cmdTimestamp, PUMP_START_REASON_REMOTE_COMMAND);
                    if (success)
                    {
                        ESP_LOGI(TAG, "Pump started via MQTT command; automatic control re-enabled.");
                    }
                    else
                    {
                        ESP_LOGW(TAG, "MQTT pump start command rejected due to high water level.");
                    }
                    motorToggleRoutineRunning = false;
                    return success;
                }
            }
            break;
            case COMMAND_TYPE_CHECK_NEW_FIRMWARE:
            {
                otaManager.triggerUpdateCheck();
                return true;
            }
            break;
            case COMMAND_TYPE_UPDATE_DATA_RECEPTION_TIMEOUT:
            {
                if (valueType == 0x02) // Check if the value is an integer
                    return nvsManager.setDataReceptionTimeout(static_cast<uint32_t>(*value));
                else
                {
                    ESP_LOGE(TAG, "Invalid value type provided for this command");
                    return false;
                }
            }
            break;
            case COMMAND_TYPE_UPDATE_MIN_WATER_LEVEL:
            {
                if (valueType == 0x02) // Check if the value is an integer
                {
                    uint8_t level = (uint8_t)*value;
                    if (nvsManager.setMinWaterLevelToStartPump(level))
                    {
                        pumpController.updateMinWaterLevelToStartPump(level);
                        ESP_LOGI(TAG, "Updated min water level to start pump to %d cm", level);
                        return true;
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to save min water level to NVS");
                        return false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid value type provided for this command");
                    return false;
                }
            }
            break;
            case COMMAND_TYPE_UPDATE_MAX_WATER_LEVEL:
            {
                if (valueType == 0x02) // Check if the value is an integer
                {
                    uint8_t level = (uint8_t)*value;
                    if (nvsManager.setMaxWaterLevelToStopPump(level))
                    {
                        pumpController.updateMaxWaterLevelToStopPump(level);
                        ESP_LOGI(TAG, "Updated max water level to stop pump to %d cm", level);
                        return true;
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to save max water level to NVS");
                        return false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid value type provided for this command");
                    return false;
                }
            }
            break;
            case COMMAND_TYPE_UPDATE_ALLOWED_WL_CONTROL_PUMP:
            {
                if (valueType == 0x02) // Check if the value is an integer
                    return nvsManager.setMinWaterLevelToAllowPumpControl((uint8_t)*value);
                else
                {
                    ESP_LOGE(TAG, "Invalid value type provided for this command");
                    return false;
                }
            }
            break;
            case COMMAND_TYPE_UPDATE_MANUAL_LOCKOUT_DURATION:
            {
                if (valueType == 0x02) // Check if the value is an integer
                {
                    uint32_t duration_in_minutes = *value;
                    uint32_t duration_ms = duration_in_minutes * 60 * 1000;
                    
                    esp_err_t err = nvsManager.setManualLockoutDurationMs(duration_ms);
                    if (err != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to set manual lockout duration, validation failed. Error: %s", esp_err_to_name(err));
                        return false;
                    }
                    
                    pumpController.updateManualLockoutDuration(duration_ms);
                    ESP_LOGI(TAG, "Updated lockout duration to %" PRIu32 " ms (%" PRIu32 " minutes)", duration_ms, duration_in_minutes);
                    return true;
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid value type for COMMAND_TYPE_UPDATE_MANUAL_LOCKOUT_DURATION, expected Integer");
                    return false;
                }
            }
            break;
            case COMMAND_TYPE_UPDATE_MAX_SECONDS_PER_PERCENT_RISE:
            {
                if (valueType == 0x02) // Check if the value is an integer
                {
                    // Apply ceiling to the SPPR value before saving
                    float sppr_value = ceilf((float)*value);
                    
                    // Use the correct Kalman filter state function (not the deprecated setMaxSecondsPerPercentRise)
                    if (nvsManager.setKalmanStateXHat(sppr_value))
                    {
                        ESP_LOGI(TAG, "Successfully updated SPPR to %.0f s/%% (ceiled from %" PRIu32 ")", sppr_value, *value);
                        return true;
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to set SPPR, validation failed or NVS error");
                        return false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid value type for COMMAND_TYPE_UPDATE_MAX_SECONDS_PER_PERCENT_RISE, expected Integer");
                    return false;
                }
            }
            break;
            case COMMAND_TYPE_RECALIBRATE:
            {
                if (valueType == 0x02) // Check if the value is an integer
                {
                    bool enable = (*value == 1);
                    ESP_LOGI(TAG, "MQTT Command: RECALIBRATE - %s", enable ? "ENABLE" : "DISABLE");
                    pumpController.setRecalibrationMode(enable);
                    return true;
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid value type for COMMAND_TYPE_RECALIBRATE, expected Integer");
                    return false;
                }
            }
            break;
            case COMMAND_TYPE_ENABLE_LOCKOUT_FEATURE:
            {
                // Validate that value and valueType are provided
                if (value == NULL)
                {
                    ESP_LOGE(TAG, "COMMAND_TYPE_ENABLE_LOCKOUT_FEATURE: value is required");
                    return false;
                }

                // Validate value type is integer (0x02)
                if (valueType != 0x02)
                {
                    ESP_LOGE(TAG, "COMMAND_TYPE_ENABLE_LOCKOUT_FEATURE: Invalid value type (expected Integer, got 0x%02x)", valueType);
                    return false;
                }

                // Extract boolean from integer value (1 = enable, 0 = disable)
                bool enabled = (*value == 1);
                
                ESP_LOGI(TAG, "MQTT Command: ENABLE_LOCKOUT_FEATURE - %s", enabled ? "ENABLE" : "DISABLE");

                // Call PumpController to set lockout feature state
                esp_err_t ret = pumpController.setLockoutFeatureEnabled(enabled);
                
                if (ret == ESP_OK)
                {
                    ESP_LOGI(TAG, "Lockout feature %s successfully", enabled ? "enabled" : "disabled");
                    return true;
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to %s lockout feature: %s", 
                             enabled ? "enable" : "disable", 
                             esp_err_to_name(ret));
                    return false;
                }
            }
            break;
            case COMMAND_TYPE_RESET_RECEIVER_DEVICE:
            {
                ESP_LOGE(TAG, "Resetting the receiver device upon receiving mqtt command");
                xTaskCreate(
                    delayed_restart_task,
                    "restart_task",
                    RESTART_TASK_STACK_DEPTH,
                    (void *)RESTART_DELAY_MS,
                    5,
                    NULL);
            }
            break;
            case COMMAND_TYPE_RESET_TRANSMITTER_DEVICE:
            {
                ResetCommandData *commandData = (ResetCommandData *)malloc(sizeof(ResetCommandData));
                commandData->dataType = TYPE_COMMAND_DATA;
                commandData->commandType = COMMAND_TYPE_RESET_DEVICE;
                uint8_t checksum = sensorDataRepo.getChecksum((char *)commandData, sizeof(ResetCommandData) - 1);
                commandData->checksum = checksum;
                uartManager.sendData((char *)commandData, sizeof(ResetCommandData));
                free(commandData);
                return true;
            }
            break;
            case COMMAND_TYPE_UPDATE_TEMPERATURE_COVARIANCE:
            {
                if (valueType != 0x01)
                {
                    ESP_LOGE(TAG, "Invalid value Type for this command");
                    return false;
                }

                size_t len = sizeof(ParamUpdateCommandData);
                ParamUpdateCommandData *commandData = (ParamUpdateCommandData *)malloc(len);
                commandData->dataType = TYPE_COMMAND_DATA;
                commandData->commandType = COMMAND_TYPE_UPDATE_PARAM;
                commandData->paramType = PARAM_TYPE_TEMPERATURE_COVARIANCE;
                commandData->dataLength = sizeof(float);
                memcpy(&(commandData->data), value, sizeof(uint32_t));

                uint8_t checksum = sensorDataRepo.getChecksum((char *)commandData, len - 1);
                commandData->checksum = checksum;
                uartManager.sendData((char *)commandData, len);
                free(commandData);
                return true;
            }
            break;
            case COMMAND_TYPE_UPDATE_HUMIDITY_COVARIANCE:
            {
                if (valueType != 0x01)
                {
                    ESP_LOGE(TAG, "Invalid value Type for this command");
                    return false;
                }

                size_t len = sizeof(ParamUpdateCommandData);
                ParamUpdateCommandData *commandData = (ParamUpdateCommandData *)malloc(len);
                commandData->dataType = TYPE_COMMAND_DATA;
                commandData->commandType = COMMAND_TYPE_UPDATE_PARAM;
                commandData->paramType = PARAM_TYPE_HUMIDITY_COVARIANCE;
                commandData->dataLength = sizeof(float);
                memcpy(&(commandData->data), value, sizeof(uint32_t));

                uint8_t checksum = sensorDataRepo.getChecksum((char *)commandData, len - 1);
                commandData->checksum = checksum;
                uartManager.sendData((char *)commandData, len);
                free(commandData);
                return true;
            }
            break;
            case COMMAND_TYPE_UPDATE_WATER_LEVEL_COVARIANCE:
            {
                if (valueType != 0x01)
                {
                    ESP_LOGE(TAG, "Invalid value Type for this command");
                    return false;
                }

                size_t len = sizeof(ParamUpdateCommandData);
                ParamUpdateCommandData *commandData = (ParamUpdateCommandData *)malloc(len);
                commandData->dataType = TYPE_COMMAND_DATA;
                commandData->commandType = COMMAND_TYPE_UPDATE_PARAM;
                commandData->paramType = PARAM_TYPE_DISTANCE_COVARIANCE;
                commandData->dataLength = sizeof(float);
                memcpy(&(commandData->data), value, sizeof(uint32_t));

                uint8_t checksum = sensorDataRepo.getChecksum((char *)commandData, len - 1);
                commandData->checksum = checksum;
                uartManager.sendData((char *)commandData, len);
                free(commandData);
                return true;
            }
            break;
            case COMMAND_TYPE_UPDATE_SENSOR_HEIGHT:
            {
                if (valueType != 0x01)
                {
                    ESP_LOGE(TAG, "Invalid value Type for this command");
                    return false;
                }

                size_t len = sizeof(ParamUpdateCommandData);
                ParamUpdateCommandData *commandData = (ParamUpdateCommandData *)malloc(len);
                commandData->dataType = TYPE_COMMAND_DATA;
                commandData->commandType = COMMAND_TYPE_UPDATE_PARAM;
                commandData->paramType = PARAM_TYPE_SENSOR_HEIGHT;
                commandData->dataLength = sizeof(float);
                memcpy(&(commandData->data), value, sizeof(uint32_t));

                uint8_t checksum = sensorDataRepo.getChecksum((char *)commandData, len - 1);
                commandData->checksum = checksum;
                uartManager.sendData((char *)commandData, len);
                free(commandData);
                return true;
            }
            break;
            case COMMAND_TYPE_UPDATE_MIN_HEIGHT:
            {
                if (valueType != 0x01)
                {
                    ESP_LOGE(TAG, "Invalid value Type for this command");
                    return false;
                }

                size_t len = sizeof(ParamUpdateCommandData);
                ParamUpdateCommandData *commandData = (ParamUpdateCommandData *)malloc(len);
                commandData->dataType = TYPE_COMMAND_DATA;
                commandData->commandType = COMMAND_TYPE_UPDATE_PARAM;
                commandData->paramType = PARAM_TYPE_MIN_LEVEL;
                commandData->dataLength = sizeof(float);
                memcpy(&(commandData->data), value, sizeof(uint32_t));

                uint8_t checksum = sensorDataRepo.getChecksum((char *)commandData, len - 1);
                commandData->checksum = checksum;
                uartManager.sendData((char *)commandData, len);
                free(commandData);
                return true;
            }
            break;
            case COMMAND_TYPE_UPDATE_MAX_HEIGHT:
            {
                if (valueType != 0x01)
                {
                    ESP_LOGE(TAG, "Invalid value Type for this command");
                    return false;
                }

                size_t len = sizeof(ParamUpdateCommandData);
                ParamUpdateCommandData *commandData = (ParamUpdateCommandData *)malloc(len);
                commandData->dataType = TYPE_COMMAND_DATA;
                commandData->commandType = COMMAND_TYPE_UPDATE_PARAM;
                commandData->paramType = PARAM_TYPE_MAX_LEVEL;
                commandData->dataLength = sizeof(float);
                memcpy(&(commandData->data), value, sizeof(uint32_t));

                uint8_t checksum = sensorDataRepo.getChecksum((char *)commandData, len - 1);
                commandData->checksum = checksum;
                uartManager.sendData((char *)commandData, len);
                free(commandData);
                return true;
            }
            break;
            case COMMAND_TYPE_UPDATE_WATER_TANK_AREA:
            {
                if (valueType != 0x01)
                {
                    ESP_LOGE(TAG, "Invalid value Type for this command");
                    return false;
                }

                size_t len = sizeof(ParamUpdateCommandData);
                ParamUpdateCommandData *commandData = (ParamUpdateCommandData *)malloc(len);
                commandData->dataType = TYPE_COMMAND_DATA;
                commandData->commandType = COMMAND_TYPE_UPDATE_PARAM;
                commandData->paramType = PARAM_TYPE_WATER_TANK_AREA;
                commandData->dataLength = sizeof(float);
                memcpy(&(commandData->data), value, sizeof(uint32_t));

                uint8_t checksum = sensorDataRepo.getChecksum((char *)commandData, len - 1);
                commandData->checksum = checksum;
                uartManager.sendData((char *)commandData, len);
                free(commandData);
                return true;
            }
            break;
            case COMMAND_TYPE_UPDATE_WATER_TANK_TYPE:
            {
                if (valueType != 0x02)
                {
                    ESP_LOGE(TAG, "Invalid value Type for this command");
                    return false;
                }

                size_t len = sizeof(ParamUpdateCommandData);
                ParamUpdateCommandData *commandData = (ParamUpdateCommandData *)malloc(len);
                commandData->dataType = TYPE_COMMAND_DATA;
                commandData->commandType = COMMAND_TYPE_UPDATE_PARAM;
                commandData->paramType = PARAM_TYPE_WATER_TANK_TYPE;
                commandData->dataLength = sizeof(uint8_t);
                commandData->data = (uint8_t)*value;

                uint8_t checksum = sensorDataRepo.getChecksum((char *)commandData, len - 1);
                commandData->checksum = checksum;
                uartManager.sendData((char *)commandData, len);
                free(commandData);
                return true;
            }
            break;
            case COMMAND_TYPE_INVALID:
            {
                ESP_LOGE(TAG, "Invalid Command Received");
                return false;
            }
            break;
            };
            return false;
        });

    mqttManager.setOnDeviceRequestReceivedListener(
        [&](DeviceRequestType requestType, char *responseTopicString, char *&responseStringHolder) -> bool
        {
            ESP_LOGW(TAG, "Request Received from server with response topic : %s | type : %s ", responseTopicString, deviceRequestTypeToString(requestType));
            switch (requestType)
            {
            case REQUEST_TYPE_RECEIVER_DEVICE_INFO:
            {
                // Create a jsonObject and now add the following fields
                cJSON *root = cJSON_CreateObject();
                cJSON_AddNumberToObject(root, "type", REQUEST_TYPE_RECEIVER_DEVICE_INFO);
                cJSON_AddNumberToObject(root, "status", RESPONSE_TYPE_REQUEST_SUCCEEDED);

                cJSON *response = cJSON_CreateObject();
                // deviceID
                char deviceID[15];
                getDeviceID(deviceID, 15);

                // deviceType - esp32, esp32s2, esp32c3 etc ..
                // number of cores
                esp_chip_info_t chipInfo;
                esp_chip_info(&chipInfo);

                // Heap memory info (Requirements 9.1, 9.2, 9.3)
                // Use HeapMonitor::getHeapInfo() for consistent heap metrics
                size_t freeHeap, minFreeHeap, largestBlock;
                HeapMonitor::getHeapInfo(freeHeap, minFreeHeap, largestBlock);
                ESP_LOGI(TAG, "Device info heap metrics: free=%u, min=%u, largest=%u bytes",
                         (unsigned int)freeHeap, (unsigned int)minFreeHeap, (unsigned int)largestBlock);

                // uptime in milliseconds
                int64_t uptimeMillis = (esp_timer_get_time() / (int64_t)1000);

                cJSON_AddNumberToObject(response, "cores", chipInfo.cores);
                cJSON_AddNumberToObject(response, "uptimeMillis", uptimeMillis);
                cJSON_AddNumberToObject(response, "freeHeap", (int)freeHeap);           // Requirement 9.1
                cJSON_AddNumberToObject(response, "minFreeHeap", (int)minFreeHeap);     // Requirement 9.2
                cJSON_AddNumberToObject(response, "largestBlock", (int)largestBlock);   // Requirement 9.3
                cJSON_AddStringToObject(response, "deviceID", deviceID);
                cJSON_AddStringToObject(response, "chipModel", chipModelToString(chipInfo.model));

                cJSON_AddItemToObject(root, "response", response);

                responseStringHolder = cJSON_PrintUnformatted(root);
                cJSON_Delete(root);
                return true;
            }
            break;
            case REQUEST_TYPE_RECEIVER_APP_INFO:
            {
                cJSON *appDescJson = NULL;
                cJSON *root = cJSON_CreateObject();
                cJSON_AddNumberToObject(root, "type", REQUEST_TYPE_RECEIVER_APP_INFO);
                bool success = otaManager.getCurrentAppDescription(appDescJson) != false;
                cJSON_AddNumberToObject(root, "status", success ? RESPONSE_TYPE_REQUEST_SUCCEEDED : RESPONSE_TYPE_REQUEST_FAILED);
                if (success)
                    cJSON_AddItemToObject(root, "response", appDescJson);

                responseStringHolder = cJSON_PrintUnformatted(root);
                cJSON_Delete(root);
                return success;
            }
            break;
            case REQUEST_TYPE_RECEIVER_CONNECTION_INFO:
            {
                char *resultHolder = NULL;
                cJSON *root = NULL;
                cJSON *resultRoot = NULL;
                root = cJSON_CreateObject();
                cJSON_AddNumberToObject(root, "type", REQUEST_TYPE_RECEIVER_CONNECTION_INFO);

                bool success = wifiManager.getApConnectionStatusInJson(resultHolder);
                cJSON_AddNumberToObject(root, "status", success ? RESPONSE_TYPE_REQUEST_SUCCEEDED : RESPONSE_TYPE_REQUEST_FAILED);

                if (success == true && resultHolder != NULL)
                {
                    resultRoot = cJSON_Parse(resultHolder);
                    if (resultRoot != NULL)
                        cJSON_AddItemToObject(root, "response", resultRoot);
                }

                responseStringHolder = cJSON_PrintUnformatted(root);
                // ESP_LOGW(TAG, "Scan response string : %s", responseStringHolder);

                if (cJSON_HasObjectItem(root, "response"))
                    cJSON_DeleteItemFromObject(root, "response");

                cJSON_Delete(root);
                if (resultHolder != NULL)
                {
                    free(resultHolder);
                }

                return success;
            }
            break;
            case REQUEST_TYPE_RECEIVER_WIFI_SCAN:
            {
                // Optimized: Avoid double JSON serialization to reduce stack usage
                // Previously: getWiFiScanResultInJson() -> cJSON_Parse() -> cJSON_AddItem -> cJSON_Print()
                // Now: getWiFiScanResultInJson() -> string concatenation (no cJSON recursion)
                char *scanResultHolder = NULL;
                bool success = wifiManager.getWiFiScanResultInJson(scanResultHolder);

                if (success && scanResultHolder != NULL)
                {
                    // Build response by string concatenation instead of cJSON re-serialization
                    // This avoids deep cJSON recursion that can cause stack overflow with 25+ networks
                    const char *prefix = "{\"type\":70,\"status\":255,\"response\":";
                    const char *suffix = "}";
                    size_t prefixLen = strlen(prefix);
                    size_t scanLen = strlen(scanResultHolder);
                    size_t suffixLen = strlen(suffix);
                    size_t totalLen = prefixLen + scanLen + suffixLen + 1;

                    responseStringHolder = (char *)malloc(totalLen);
                    if (responseStringHolder != NULL)
                    {
                        memcpy(responseStringHolder, prefix, prefixLen);
                        memcpy(responseStringHolder + prefixLen, scanResultHolder, scanLen);
                        memcpy(responseStringHolder + prefixLen + scanLen, suffix, suffixLen);
                        responseStringHolder[totalLen - 1] = '\0';
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to allocate memory for WiFi scan response");
                        success = false;
                    }
                    free(scanResultHolder);
                }
                else
                {
                    // Failure case - minimal JSON response
                    const char *failResponse = "{\"type\":70,\"status\":0}";
                    size_t len = strlen(failResponse) + 1;
                    responseStringHolder = (char *)malloc(len);
                    if (responseStringHolder != NULL)
                    {
                        memcpy(responseStringHolder, failResponse, len);
                    }
                }

                return success;
            }
            break;
            case REQUEST_TYPE_RECEIVER_WATER_LEVEL_INFO:
            {
                SensorData *sensorData = NULL;
                cJSON *root = cJSON_CreateObject();
                cJSON_AddNumberToObject(root, "type", REQUEST_TYPE_RECEIVER_WATER_LEVEL_INFO);
                bool success = sensorDataRepo.getSensorData(sensorData) == DATA_FETCH_SUCCESS;
                int pumpStatus = peripheralManager.getCurrentMotorState() == ON ? 1 : 0;
                if (success)
                {
                    cJSON_AddNumberToObject(root, "status", RESPONSE_TYPE_REQUEST_SUCCEEDED);
                    // now add another element
                    cJSON *responseRoot = cJSON_CreateObject();
                    cJSON_AddNumberToObject(responseRoot, "level", sensorData->waterLevel);
                    cJSON_AddNumberToObject(responseRoot, "temp", sensorData->temperatureCelsius);
                    cJSON_AddNumberToObject(responseRoot, "humidity", sensorData->humidity);
                    cJSON_AddNumberToObject(responseRoot, "pumpStatus", pumpStatus);
                    
                    // Get SPPR from Kalman filter state (not the deprecated maxSecondsPerPercentRise)
                    float sppr_value = 0.0f;
                    if (nvsManager.getKalmanStateXHat(sppr_value))
                    {
                        // Successfully loaded SPPR from Kalman filter state
                        cJSON_AddNumberToObject(responseRoot, "sppr", (int)sppr_value);
                    }
                    else
                    {
                        // Fallback: use default value if Kalman state not available
                        ESP_LOGW("Main", "Could not load SPPR from Kalman filter state, using default value");
                        cJSON_AddNumberToObject(responseRoot, "sppr", 15);
                    }

                    uint8_t minWaterLevel, maxWaterLevel;
                    if (nvsManager.getMinWaterLevelToStartPump(minWaterLevel))
                    {
                        cJSON_AddNumberToObject(responseRoot, "minWaterLevel", minWaterLevel);
                    }
                    if (nvsManager.getMaxWaterLevelToStopPump(maxWaterLevel))
                    {
                        cJSON_AddNumberToObject(responseRoot, "maxWaterLevel", maxWaterLevel);
                    }

                    // Add lockout information
                    LockoutInfo lockoutInfo = pumpController.getLockoutStatus();
                    cJSON *lockoutRoot = cJSON_CreateObject();
                    cJSON_AddNumberToObject(lockoutRoot, "isActive", lockoutInfo.isActive ? 1 : 0);

                    if (lockoutInfo.isActive)
                    {
                        cJSON_AddNumberToObject(lockoutRoot, "startedAt", lockoutInfo.startedAtMillis);
                        cJSON_AddNumberToObject(lockoutRoot, "elapsed", lockoutInfo.elapsedMillis);
                        cJSON_AddNumberToObject(lockoutRoot, "endsAt", lockoutInfo.endsAtMillis);
                    }

                    cJSON_AddItemToObject(responseRoot, "lockout", lockoutRoot);
                    
                    // Add calibration mode status
                    bool isCalibrationMode = pumpController.isRecalibrationMode();
                    cJSON_AddBoolToObject(responseRoot, "calibrationMode", isCalibrationMode);
                    
                    cJSON_AddItemToObject(root, "response", responseRoot);
                }
                else
                {
                    cJSON_AddNumberToObject(root, "status", RESPONSE_TYPE_REQUEST_FAILED);
                }
                responseStringHolder = cJSON_PrintUnformatted(root);
                cJSON_Delete(root);
                if (sensorData != NULL)
                    free(sensorData);
                return success;
            }
            break;
            case REQUEST_TYPE_LAST_PUMP_RUN_METADATA:
            {
                LastPumpRunMetaData metaData;
                cJSON *root = cJSON_CreateObject();
                bool success = false;
                cJSON_AddNumberToObject(root, "type", REQUEST_TYPE_LAST_PUMP_RUN_METADATA);
                if (nvsManager.getLastPumpRunMetaData(metaData))
                {
                    cJSON_AddNumberToObject(root, "status", RESPONSE_TYPE_REQUEST_SUCCEEDED);
                    cJSON *response = cJSON_CreateObject();
                    cJSON_AddNumberToObject(response, "startedAt", metaData.startedAt);
                    cJSON_AddNumberToObject(response, "runDurationMs", metaData.runDurationMs);
                    cJSON_AddNumberToObject(response, "waterLevelDelta", metaData.waterLevelDelta);
                    cJSON_AddStringToObject(response, "startReason", pumpStartReasonToString(metaData.startReason));
                    cJSON_AddStringToObject(response, "stopReason", pumpStopReasonToString(metaData.stopReason));
                    cJSON_AddItemToObject(root, "response", response);
                    success = true;
                    // Log minimal info to avoid stack overflow - full JSON is sent via MQTT response
                    ESP_LOGI(TAG, "Last pump run metadata retrieved: duration=%ldms, delta=%d",
                             (long)metaData.runDurationMs, metaData.waterLevelDelta);
                }
                else
                {
                    cJSON_AddNumberToObject(root, "status", RESPONSE_TYPE_REQUEST_FAILED);
                }
                responseStringHolder = cJSON_PrintUnformatted(root);
                cJSON_Delete(root);
                return success;
            }
            break;
            case REQUEST_TYPE_LOCKOUT_CONFIGURATION:
            {
                // Log the query request at DEBUG level (Requirement 7.4)
                ESP_LOGD(TAG, "Lockout configuration query request received");
                
                // Get lockout configuration from PumpController
                LockoutConfiguration config = pumpController.getLockoutConfiguration();
                
                // Create JSON response with all required fields (Requirements 3.1-3.6)
                cJSON *root = cJSON_CreateObject();
                if (root == NULL) {
                    ESP_LOGE(TAG, "Failed to create JSON object for lockout configuration response");
                    return false;
                }
                
                cJSON_AddNumberToObject(root, "type", REQUEST_TYPE_LOCKOUT_CONFIGURATION);
                cJSON_AddNumberToObject(root, "status", RESPONSE_TYPE_REQUEST_SUCCEEDED);
                
                cJSON *response = cJSON_CreateObject();
                if (response == NULL) {
                    ESP_LOGE(TAG, "Failed to create response object for lockout configuration");
                    cJSON_Delete(root);
                    return false;
                }
                
                // Add all required fields (Requirements 3.1, 3.2, 3.3, 3.4, 3.5, 3.6)
                cJSON_AddBoolToObject(response, "lockout_feature_enabled", config.lockoutFeatureEnabled);
                cJSON_AddNumberToObject(response, "lockout_duration_ms", config.lockoutDurationMs);
                cJSON_AddBoolToObject(response, "lockout_currently_active", config.lockoutCurrentlyActive);
                cJSON_AddNumberToObject(response, "lockout_trigger_time", config.lockoutTriggerTime);
                cJSON_AddNumberToObject(response, "lockout_clear_time", config.lockoutClearTime);
                cJSON_AddNumberToObject(response, "timestamp", config.timestamp);
                
                cJSON_AddItemToObject(root, "response", response);
                
                // Validate JSON structure before publishing
                responseStringHolder = cJSON_PrintUnformatted(root);
                if (responseStringHolder == NULL) {
                    ESP_LOGE(TAG, "Failed to serialize lockout configuration response");
                    cJSON_Delete(root);
                    return false;
                }
                
                cJSON_Delete(root);
                
                ESP_LOGI(TAG, "Lockout configuration response generated: enabled=%d, duration=%lu ms, active=%d",
                         config.lockoutFeatureEnabled, config.lockoutDurationMs, config.lockoutCurrentlyActive);
                
                return true;
            }
            break;
            case REQUEST_TYPE_TRANSMITTER_APP_INFO:
            {
                // TODO- Do this after deployment
            }
            break;
            case REQUEST_TYPE_TRANSMITTER_DEVICE_INFO:
            {
                // TODO - to be implemented after deployment
            }
            break;
            case REQUEST_TYPE_INVALID:
            {
                // TODO - to be implemented after deployment
            }
            break;
            }
            return false;
        });

    mqttManager.setOnMQTTBrokerConnectedListener(
        [&]() -> void
        {
            // char *deviceID = (char *)malloc(sizeof(char) * DEVICE_ID_BUFFER_LEN);
            char *topic = (char *)malloc(MAX_MQTT_TOPIC_LENGTH);
            getDeviceID(deviceID, DEVICE_ID_BUFFER_LEN);
            snprintf(
                topic,
                MAX_MQTT_TOPIC_LENGTH,
                "/%s/%s/%s",
                message_source_to_string(MESSAGE_SOURCE_SERVER),
                deviceID,
                "#");

            // Validate subscription topic length against MAX_MQTT_TOPIC_LENGTH constraint
            size_t topicLength = strlen(topic);
            if (topicLength >= MAX_MQTT_TOPIC_LENGTH)
            {
                ESP_LOGE(TAG, "Subscription topic length (%zu) exceeds maximum allowed length (%d): %s",
                         topicLength, MAX_MQTT_TOPIC_LENGTH, topic);
                free(topic);
                return;
            }

            // Subscribe with QoS 2 for server-to-device messages to ensure exactly-once delivery
            // Use QoS 2 (Exactly Once Delivery) for server-to-device subscriptions to ensure:
            // - Critical commands are delivered exactly once without duplicates
            // - Commands like pump control, configuration changes are not executed multiple times
            // - Provides the highest level of delivery assurance with 4-way handshake (PUBLISH->PUBREC->PUBREL->PUBCOMP)
            // - Essential for operations that must not be repeated (e.g., pump start/stop commands)
            // - Prevents potential system damage from duplicate command execution
            // - Higher overhead but necessary for critical control operations
            if (mqttManager.subscribe(topic, QOS_2) == false)
                ESP_LOGE(TAG, "Failed to subscribe to the topic %s !", topic);
            else
                ESP_LOGI(TAG, "Subscribed to topic %s successfully !", topic);

            // free(deviceID);
            free(topic);

            // Initialize MQTT Command Processor for three-task decoupled architecture
            // This creates the command queue, response queue, and starts the
            // Command Processor and Publisher tasks for handling MQTT messages
            esp_err_t cmdProcRet = MqttCommandProcessor::init(&mqttManager);
            if (cmdProcRet != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to initialize MqttCommandProcessor: %s", esp_err_to_name(cmdProcRet));
                // Continue operation - the system can still function with direct callback execution
                // but rate limiting and decoupled processing will not be available
            }
            else
            {
                ESP_LOGI(TAG, "MqttCommandProcessor initialized successfully");
                
                // Register OTA status callback to reject MQTT commands during OTA updates
                MqttCommandProcessor::setOtaStatusCallback([]() -> bool {
                    return otaManager.isOTAUpdateRunning();
                });
                ESP_LOGI(TAG, "OTA status callback registered with MqttCommandProcessor");
            }

            ledIndication.updateMqttStatus(MqttAppStatus::MQTT_CONNECTED); // MQTT Connected
        });

    mqttManager.setOnMQTTBrokerDisconnectedListener(
        [&]() -> void
        {
            char *topic = (char *)malloc(MAX_MQTT_TOPIC_LENGTH);
            getDeviceID(deviceID, DEVICE_ID_BUFFER_LEN);
            snprintf(
                topic,
                MAX_MQTT_TOPIC_LENGTH,
                "/%s/%s/%s",
                message_source_to_string(MESSAGE_SOURCE_SERVER),
                deviceID,
                "#");

            if (mqttManager.unsubscribe(topic) == false)
                ESP_LOGE(TAG, "Failed to unsubscribe from the topic %s !", topic);
            else
                ESP_LOGI(TAG, "Unsubscribed from the topic %s successfully !", topic);

            ledIndication.updateMqttStatus(MqttAppStatus::MQTT_DISCONNECTED); // MQTT Disconnected (not due to WiFi)

            free(topic);
        });

    // Make WiFi manager aware of the RTC manager for future sync updates
    wifiManager.setRtcManager(&i2cManager);
    wifiManager.setNvsManager(&nvsManager);

    wifiManager.setOnNewStationConnectedListener(
        [&](string ipAddress, string macID) -> void
        {
            ESP_LOGW(TAG, "New Wi-Fi station connected, IP: %.*s | MAC: %.*s", ipAddress.length(), ipAddress.c_str(), macID.length(), macID.c_str());
            ledIndication.updateHttpServerStatus(HttpServerAppStatus::HTTP_AP_ACTIVE_CLIENT_CONNECTED);
            // The server is already started, this just confirms a client is now connected.
        });

    // Register config mode timeout callback - triggers restart if no station connects for 5 minutes
    wifiManager.setOnConfigModeTimeoutCallback(
        [&]() -> void
        {
            ESP_LOGW(TAG, "Config mode timeout - no station connected for 5 minutes. Restarting device...");
            buzzerControl.playFailureTune();  // Audio feedback before restart
            xTaskCreatePinnedToCore(
                delayed_restart_task,
                "config_timeout_restart",
                2048,
                (void *)1000,  // 1 second delay before restart
                5,
                NULL,
                APP_CPU_NUM);
        });

    wifiManager.setOnAccessPointConnectedListener(
        [&]() -> void
        {
            ESP_LOGI(TAG, "Connected to the Access Point. isInConfigMode = %s", isInConfigMode ? "true" : "false");
            if (isInConfigMode)
            {
                ESP_LOGW(TAG, "In config mode, ignoring AP connection event.");
                return;
            }
#ifdef CONFIG_HEAP_TRACING_STANDALONE
            ESP_LOGI(TAG, "Starting heap trace for leak detection.");
            HEAP_TRACE_START();
#endif

            ledIndication.updateWifiStaStatus(WifiStaAppStatus::WIFI_STA_CONNECTED); // WiFi Connected
            ledIndication.updateMqttStatus(MqttAppStatus::MQTT_CONNECTING);          // MQTT Trying to Connect

            if (mqttManager.isConnected() == false)
            {
                // ESP_LOGW(TAG, "MQTT broker not connected");
                ESP_LOGI(TAG, "Before MQTT connect, free heap: %" PRIu32, esp_get_free_heap_size());
                // HEAP_TRACE_START();
                mqttManager.connect(
                    deviceID,
                    MQTT_BROKER_URL,
                    MQTT_BROKER_CREDENTIAL_USERNAME,
                    MQTT_BROKER_CREDENTIAL_PASSWORD);

                ESP_LOGI(TAG, "After MQTT connect, free heap: %" PRIu32, esp_get_free_heap_size());
            }

            otaManager.startPeriodicUpdatesCheck();
            ESP_LOGW(TAG, "Started periodic updates check");

            ESP_LOGI(TAG, "Starting SNTP synchronization task.");
            wifiManager.startSNTPSync();
        });

    wifiManager.setOnAccessPointDisconnectedListener(
        [&](std::string ssid, wifi_err_reason_t reason) -> void
        {
            ESP_LOGW(TAG, "Disconnected from the access point! Cleaning up MQTT.");

            if (reason == WIFI_REASON_AUTH_FAIL)
            {
                ESP_LOGE(TAG, "Authentication failed for SSID: %s. Deleting credentials.", ssid.c_str());
                nvsManager.deleteCredentialWithSSID(ssid);
            }

            ESP_LOGI(TAG, "Before MQTT disconnect, free heap: %" PRIu32, esp_get_free_heap_size());
            mqttManager.disconnect();
#ifdef CONFIG_HEAP_TRACING_STANDALONE
            ESP_LOGI(TAG, "Stopping heap trace for leak detection.");
            HEAP_TRACE_STOP_DUMP();
#endif
            ESP_LOGI(TAG, "After MQTT disconnect, free heap: %" PRIu32, esp_get_free_heap_size());
            ledIndication.updateWifiStaStatus(WifiStaAppStatus::WIFI_STA_DISCONNECTED); // WiFi Disconnected
            ledIndication.updateMqttStatus(MqttAppStatus::MQTT_WIFI_DISCONNECTED);      // MQTT also down due to WiFi
            otaManager.stopPeriodicUpdatesCheck();
            ESP_LOGW(TAG, "Stopped periodic updates check");

            // The reconnection logic is now handled by the wifiReconnectionTask.
        });

    wifiManager.setOnAccessPointInitialConnectionFailedListener(
        [&]() -> void
        {
            ESP_LOGE(TAG, "Initial connection to Access Point FAILED!");
            ledIndication.updateWifiStaStatus(WifiStaAppStatus::WIFI_STA_DISCONNECTED); // Treat as disconnected
            ledIndication.updateMqttStatus(MqttAppStatus::MQTT_WIFI_DISCONNECTED);      // MQTT also unavailable
        });

    // Check saved credentials and connect to the one with highest signal strength
    size_t credentialsCount = 0;
    ESP_LOGI(TAG, "Checking for saved WiFi credentials...");
    if (nvsManager.getCredentialsCount(credentialsCount) && credentialsCount > 0)
    {
        ESP_LOGI(TAG, "Found %zu saved WiFi credentials. Initializing WiFi.", credentialsCount);
        ledIndication.updateWifiStaStatus(WifiStaAppStatus::WIFI_STA_INITIALIZING);
        wifiManager.startWiFiAsStationSync();
        ESP_LOGI(TAG, "Attempting to connect to WiFi Access Point...");
        ledIndication.updateWifiStaStatus(WifiStaAppStatus::WIFI_STA_CONNECTING); // WiFi Trying to connect

        if (credentialsCount == 1)
        {
            // Only one credential, connect to it directly
            WiFiCredentials *credential = NULL;
            size_t count = 0;
            if (nvsManager.getSavedCredentials(credential, count) && credential != NULL)
            {
                ESP_LOGI(TAG, "Connecting to single saved credential: %s", credential[0].ssid);
                wifiManager.connectToAccessPoint(string(credential[0].ssid), string(credential[0].password));
                free(credential);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to retrieve saved credential");
            }
        }
        else
        {
            // Multiple credentials, connect to the one with highest signal strength
            WiFiCredentials *credentials = NULL;
            size_t count = 0;
            if (nvsManager.getSavedCredentials(credentials, count) && credentials != NULL)
            {
                ESP_LOGI(TAG, "Connecting to nearest saved access point from %zu credentials", count);
                if (!wifiManager.connectToNearestSavedAccessPoint(credentials, count))
                {
                    ESP_LOGW(TAG, "Failed to connect to any saved access point");
                }
                free(credentials);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to retrieve saved credentials");
            }
        }
    }
    else
    {
        ESP_LOGE(TAG, "No WiFi credentials found in NVS. Skipping WiFi initialization.");
        ledIndication.updateWifiStaStatus(WifiStaAppStatus::DEVICE_STATE_NO_WIFI_CREDS); // No WiFi credentials available
    }

    // Commented out hardcoded connection - replaced with saved credentials logic above
    // wifiManager.connectToAccessPoint("FluctoHotspot", "12345678");
    // wifiManager.connectToAccessPoint("Meera_Niwas_Top_Floor", "8621843141");
    // wifiManager.connectToAccessPoint("IN1", "in@india");

    SensorData *sensorData = NULL;
    uint8_t previousWaterLevel = 0;
    while (true)
    {
        // Check for application events without blocking
        EventBits_t bits = xEventGroupWaitBits(app_event_group,
                                               ENTER_CONFIG_MODE_BIT | PUMP_MONITORING_TICK_BIT | UART_TIMEOUT_EVENT_BIT,
                                               pdTRUE,  // Clear bits on exit
                                               pdFALSE, // Don't wait for all bits
                                               0);      // Non-blocking

        // Handle UART timeout event (deferred from callback to prevent IDLE task starvation)
        if (bits & UART_TIMEOUT_EVENT_BIT)
        {
            ESP_LOGW(TAG, "UART timeout event received - handling in main task context");
            
            // Perform heavy operations that were previously in the callback
            if (mqttManager.isConnected())
            {
                uint64_t timestamp = 0;
                wifiManager.getTimestampMillis(timestamp);
                publishEvent(
                    EVENT_TYPE_DATA_RECEPTION_TIMEOUT,
                    EVENT_LEVEL_ALERT,
                    timestamp);
            }
            buzzerControl.playFailureTune();
            ledIndication.updateDataReceptionStatus(DataReceptionAppStatus::DATA_RECEPTION_TIMEOUT);
        }

        if (bits & ENTER_CONFIG_MODE_BIT)
        {
            ESP_LOGW(TAG, "ENTER_CONFIG_MODE_BIT received, processing configuration mode...");

            ESP_LOGI(TAG, "Stopping services for configuration mode...");

            // Step 1: Clear all LED indications to ensure clean transition
            ledIndication.clearIndications();
            vTaskDelay(pdMS_TO_TICKS(100)); // Brief delay to ensure LEDs are cleared

            // Step 2: Stop networking services
            mqttManager.disconnect();
            otaManager.stopPeriodicUpdatesCheck();

            // Step 3: Stop WiFi
            wifiManager.stopWiFi();
            vTaskDelay(pdMS_TO_TICKS(200)); // Give time for services to stop cleanly

            ESP_LOGI(TAG, "Free heap before starting config services: %" PRIu32, (uint32_t)esp_get_free_heap_size());

            // Step 4: Set LED mode and start WiFi in AP+STA mode
            ledIndication.setLed3OverallMode(Led3AppMode::WIFI_AP_HTTP_SERVER_MODE);
            ledIndication.updateHttpServerStatus(HttpServerAppStatus::HTTP_SERVER_INITIATING); // Purple fading starts

            ESP_LOGW(TAG, "Starting WiFi as APSTA...");
            wifiManager.startWiFiAsAccessPointStationSync();
            ESP_LOGI(TAG, "WiFi Started as APSTA");

            // Step 4: Start HTTPS server and update LED status
            httpsServer.start();
            ledIndication.updateHttpServerStatus(HttpServerAppStatus::HTTP_AP_ACTIVE_READY_FOR_CONN); // Solid purple

            ESP_LOGW(TAG, "Configuration mode is active. Halting main task.");
            // Enter an infinite loop to halt the main task. The device will need to be restarted to exit this mode.
            while (isInConfigMode)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

        if (bits & PUMP_MONITORING_TICK_BIT)
        {
            // ESP_LOGI(TAG, "PUMP_MONITORING_TICK_BIT received, processing pump monitoring...");
            // pumpController.processPumpMonitoring(); // This is now handled by a timer
        }

        // SAFETY: Free previous sensor data allocation before fetching new data
        // Add explicit NULL check to prevent double-free heap corruption
        if (sensorData != NULL)
        {
            free(sensorData);
            sensorData = NULL;
        }

        if (sensorDataRepo.getSensorData(sensorData) == DATA_FETCH_SUCCESS)
        {
            if (sensorData == NULL)
            {
                ESP_LOGE(TAG, "Sensor data is null after fetch!");
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            
            // CRITICAL FIX: Rate-limit display updates to prevent IDLE task starvation
            // Display update with delayMicroseconds() blocks for ~100us per call
            // Water level changes slowly (~500ms sensor updates), so update display max once per 500ms
            static uint32_t lastDisplayUpdate = 0;
            uint32_t currentTime = esp_timer_get_time() / 1000; // Convert to milliseconds
            if (currentTime - lastDisplayUpdate >= 500) {
                peripheralManager.displayTwoDigitNumber(sensorData->waterLevel);
                lastDisplayUpdate = currentTime;
            }

            if (previousWaterLevel != sensorData->waterLevel)
            {
                // Only log significant water level changes (>2%) to reduce UART traffic
                if (abs((int)sensorData->waterLevel - (int)previousWaterLevel) >= 3) {
                    ESP_LOGI(TAG, "WL: %" PRIu8 "%% -> %" PRIu8 "%%", previousWaterLevel, sensorData->waterLevel);
                }

                uint64_t currentTimestamp = 0;
                wifiManager.getTimestampMillis(currentTimestamp); // Get timestamp for events

                pumpController.handleWaterLevelChange(sensorData->waterLevel, previousWaterLevel, currentTimestamp);

                previousWaterLevel = sensorData->waterLevel;
            }

            // Run Kalman filter for water level noise filtering
            // This runs ALWAYS (regardless of network status) to maintain filter state
            // and provide consistent velocity estimates for local decision making
            
            // Reject 0% readings (sensor initialization artifact)
            // Water level of exactly 0% is extremely unlikely in normal operation
            // If it occurs, the next valid reading will initialize the filter correctly
            if (sensorData->waterLevel == 0) {
                // Rate-limit this log to once per 5 seconds to prevent UART flooding
                static uint32_t lastZeroWlLogTime = 0;
                uint32_t now = esp_timer_get_time() / 1000;
                if (now - lastZeroWlLogTime >= 5000) {
                    ESP_LOGW(TAG, "Rejecting 0%% water level reading (likely sensor initialization artifact)");
                    lastZeroWlLogTime = now;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            
            // Only process NEW sensor data (check sequence number)
            // The main loop runs every 10ms, but sensor data arrives every ~500ms
            // Without this check, we'd process the same reading multiple times
            static uint64_t lastProcessedSequenceNumber = 0;
            if (sensorData->sequenceNumber == lastProcessedSequenceNumber) {
                // Same data as last iteration, skip processing
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            lastProcessedSequenceNumber = sensorData->sequenceNumber;
            
            // Initialize Kalman filter on first VALID reading (STAGE 1)
            if (!kalmanFilterInitialized) {
                waterLevelKalmanFilter.init(35.0, 1.0, 1.0);  // R=35, Q=1, P0=1 (very aggressive smoothing to suppress 1% oscillations, gain≈0.15)
                kalmanFilterInitialized = true;
                ESP_LOGI(TAG, "Water level Kalman filter initialized (R=35.0, Q=1.0, expected steady-state gain≈0.15)");
            }

            // Apply Kalman filter first (smooths high-frequency noise)
            // Pass pump state for adaptive filtering:
            // - Pump OFF: Heavy smoothing (gain ~0.05) to suppress sensor noise
            // - Pump ON: Responsive tracking (gain ~0.4) to follow rising water
            bool pumpIsOn = pumpController.isPumpOn();
            uint8_t kalman_filtered = waterLevelKalmanFilter.update(sensorData->waterLevel, pumpIsOn);

            // Initialize median filter on first VALID reading (STAGE 2)
            if (!medianFilterInitialized) {
                waterLevelMedianFilter.init();
                medianFilterInitialized = true;
                ESP_LOGI(TAG, "Water level median filter initialized");
                // Don't skip first reading - add it to the filter
            }

            // Update median filter with Kalman-filtered reading
            waterLevelMedianFilter.update(kalman_filtered);

            // Wait until filter has enough samples (at least 4)
            if (!waterLevelMedianFilter.isReady()) {
                ESP_LOGD(TAG, "Median filter warming up (need 4+ samples)");
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            // Get filtered level
            uint8_t filtered_level = waterLevelMedianFilter.getFilteredLevel();

            // Enhanced logging: Show raw → kalman → median → final every 10th reading
            static uint32_t filterLogCounter = 0;
            if (++filterLogCounter % 10 == 0) {
                // Get current median window for debugging
                uint8_t window[7];
                uint8_t window_size = waterLevelMedianFilter.getWindow(window);
                
                // Format window as string
                char window_str[48];  // Increased size for 7 samples
                int offset = 0;
                offset += snprintf(window_str + offset, sizeof(window_str) - offset, "[");
                for (uint8_t i = 0; i < window_size; i++) {
                    if (i > 0) {
                        offset += snprintf(window_str + offset, sizeof(window_str) - offset, ",");
                    }
                    offset += snprintf(window_str + offset, sizeof(window_str) - offset, "%d", window[i]);
                }
                snprintf(window_str + offset, sizeof(window_str) - offset, "]");
                
                ESP_LOGI(TAG, "Filter Pipeline: Raw=%d%% → Kalman=%d%% → Median Window=%s → Final=%d%% | T=%d°C, H=%d%%, Seq=%llu",
                         sensorData->waterLevel, kalman_filtered, window_str, filtered_level,
                         sensorData->temperatureCelsius, sensorData->humidity,
                         sensorData->sequenceNumber);
            }

            // Debug log every reading (can be disabled in production)
            ESP_LOGD(TAG, "Filter: raw=%d%% → kalman=%d%% → median=%d%%",
                     sensorData->waterLevel, kalman_filtered, filtered_level);

            // Check if we should publish water level data (MQTT-dependent)
            // Publishing is separate from filtering to maintain network resilience
            // Use the filtered level directly (stability buffer removed)
            if (mqttManager.isConnected())
            {
                uint64_t currentTimestamp = 0;
                wifiManager.getTimestampMillis(currentTimestamp);
                
                PeripheralState currentPumpState = peripheralManager.getCurrentMotorState();
                
                if (shouldPublishWaterLevelData(filtered_level, sensorData, 
                                                currentPumpState, currentTimestamp))
                {
                    publishWaterLevelData(sensorData, currentTimestamp, filtered_level);
                    updateLastPublishedValues(filtered_level, sensorData, currentPumpState, currentTimestamp);
                    
                    // Persist filtered water level to NVS for restart comparison
                    // Only write on significant changes (when we publish) to minimize flash wear
                    uint8_t filtered_wl_int = filtered_level;  // Already integer
                    esp_err_t ret = nvsManager.writeUint8("last_wl", filtered_wl_int);
                    if (ret != ESP_OK) {
                        ESP_LOGW(TAG, "Failed to persist filtered water level to NVS: %s", esp_err_to_name(ret));
                    } else {
                        ESP_LOGI(TAG, "Persisted filtered water level %u%% to NVS (raw: %d%%, filtered: %d%%)", 
                                 filtered_wl_int, sensorData->waterLevel, filtered_level);
                    }
                }
            }
        }

        // CRITICAL FIX: Increase delay to prevent IDLE task starvation
        // The main loop was running every 10ms with blocking operations (display updates,
        // malloc/free, mutex operations), preventing IDLE task from resetting watchdog.
        // 50ms delay provides sufficient responsiveness while allowing IDLE task to run.
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}