#include "PumpController.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <sys/time.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <climits>  // For ULONG_MAX

// Define TAG for logging, specific to this file
#define TAG "PumpController"

// Worker task configuration - stack sized for heavy logging operations
// This task handles deferred work from timer callbacks (FreeRTOS best practice)
// UPDATED: Increased from 3584 to 4096 bytes (+512 bytes additional safety margin)
// Previous: 3584 bytes (2136 used + 1448 margin = 68% safety)
// New allocation: 4096 bytes (2136 used + 1960 margin = 92% safety)
// Reason: Additional safety margin for adaptive Kalman filter implementation
#define PUMP_WORKER_TASK_STACK_SIZE 4096
#define PUMP_WORKER_TASK_PRIORITY 5

// Static member initialization for lockout feature control
// NVS key max length is 15 chars - using "lockout_en" (10 chars)
const char* PumpController::NVS_KEY_LOCKOUT_FEATURE_ENABLED = "lockout_en";

// Static member initialization for pump start race condition prevention
volatile bool PumpController::_pumpStartInProgress = false;
portMUX_TYPE PumpController::_pumpStartSpinlock = portMUX_INITIALIZER_UNLOCKED;

// Constructor
PumpController::PumpController(PeripheralManager &peripheralManager,
                               SwitchingUnitManager &switchingUnitManager,
                               BuzzerControl &buzzerControl,
                               SensorDataRepo &sensorDataRepo,
                               LedIndication &ledIndication,
                               EventPublisher eventPublisher,
                               EventGroupHandle_t event_group,
                               NVSManager &nvsManager,
                               uint8_t minWaterLevelToStartPump,
                               uint8_t maxWaterLevelToStopPump)
    : periphManager(peripheralManager),
      switchingUnit(switchingUnitManager),
      buzzControl(buzzerControl),
      sensorRepo(sensorDataRepo),
      ledIndication(ledIndication),
      publishEventCallback(eventPublisher),
      app_event_group(event_group),
      nvsManager(nvsManager),
      pumpMonitoringTimer(NULL),
      consecutiveLowIncrementChecks(0),
      manualPumpRestartRequired(false),
      manualLockoutTimer(NULL),
      isLockoutTimerActive(false),
      isRecalibrationModeActive(false),
      manualLockoutDurationMillis(CONFIG_PUMP_DEFAULT_MANUAL_LOCKOUT_DURATION_MS), // Default from Kconfig, will be loaded from NVS
      _lockoutFeatureEnabled(true),  // Default to enabled (fail-safe)
      _stateMutex(NULL),  // Initialize mutex handle to NULL
      minWaterLevelToStart(minWaterLevelToStartPump),
      maxWaterLevelToStop(maxWaterLevelToStopPump),
      kf_x_hat(0.0f),
      kf_P(0.0f),
      kf_Q(0.01f), // Default process noise variance
      kf_R(1.0f),  // Default measurement noise variance
      kf_k(3.0f),  // Default stall detection multiplier
      lastWaterLevelForStallCheck(0),
      lastLevelChangeTimestamp(0),
      monitoringState(IDLE),
      stall_confirmation_count(0),
      stall_check_start_timestamp(0),
      gracePeriodEndTimestamp(0),
      initialWaterLevelForStallCheck(0),
      stallCheckReadingsCount(0),
      ema_water_level(0.0f),
      previous_ema_water_level(0.0f),
      fast_check_start_level(0.0f),
      consecutiveNoWaterRiseChecks(0),
      pumpWorkerTaskHandle(NULL)
{
    // CRITICAL FIX: Set log level to INFO for PumpController TAG
    // Something is setting it to ERROR, so we override it here
    esp_log_level_set(TAG, ESP_LOG_INFO);
    
    // Initialize state mutex for thread-safe access to lockout configuration
    _stateMutex = xSemaphoreCreateMutex();
    if (_stateMutex == NULL) {
        ESP_LOGE(TAG, "FATAL: Failed to create state mutex - thread safety compromised!");
        ESP_ERROR_CHECK(ESP_FAIL);  // Halt system - cannot continue safely without mutex
    }
    ESP_LOGI(TAG, "State mutex created successfully");
    
    // Initialize lockout feature state from NVS
    // readBool() returns the value directly (true/false), not a success indicator
    // If key doesn't exist, it returns the default value (true for fail-safe)
    _lockoutFeatureEnabled = nvsManager.readBool(NVS_KEY_LOCKOUT_FEATURE_ENABLED, true);
    ESP_LOGI(TAG, "Lockout feature state from NVS: %s", 
             _lockoutFeatureEnabled ? "ENABLED" : "DISABLED");
    
    // ESP_LOGE(TAG, "!!! PUMPCONTROLLER CONSTRUCTOR CALLED - ERROR LEVEL TEST !!!");
    // ESP_LOGW(TAG, "!!! PUMPCONTROLLER CONSTRUCTOR CALLED - WARNING LEVEL TEST !!!");
    // ESP_LOGI(TAG, "=== PUMPCONTROLLER INITIALIZED ===");
    // ESP_LOGI(TAG, "  Firmware Build: ENHANCED_LOGGING_v2024.11.05");
    // ESP_LOGI(TAG, "  Features: SPPR Kalman Filter, Runtime Monitoring, Comprehensive Logging");
    // ESP_LOGI(TAG, "  Log Level: Explicitly set to INFO");

    // Initialize values from NVS after it's been initialized
    manualLockoutDurationMillis = nvsManager.getManualLockoutDurationMs();
    ESP_LOGI(TAG, "Loaded manual_lockout_duration from NVS: %lu ms", manualLockoutDurationMillis);

    // Load Kalman filter parameters from NVS
    kf_Q = nvsManager.getKalmanProcessNoiseVariance();
    kf_R = nvsManager.getKalmanMeasurementNoiseVariance();
    kf_k = nvsManager.getKalmanStallDetectionMultiplier();
    ESP_LOGI(TAG, "Loaded Kalman filter parameters from NVS: Q=%.4f, R=%.4f, k=%.4f", kf_Q, kf_R, kf_k);

    // Create worker task for deferred timer callback processing
    // This follows FreeRTOS best practice: timer callbacks should be lightweight
    // and signal a dedicated task to do heavy work (logging, MQTT, sensor reads)
    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        pumpWorkerTask,
        "PumpWorker",
        PUMP_WORKER_TASK_STACK_SIZE,
        this,  // Pass PumpController instance
        PUMP_WORKER_TASK_PRIORITY,
        &pumpWorkerTaskHandle,
        APP_CPU_NUM  // Run on application CPU
    );
    
    if (taskCreated != pdPASS || pumpWorkerTaskHandle == NULL) {
        ESP_LOGE(TAG, "Failed to create PumpWorker task!");
    } else {
        ESP_LOGI(TAG, "PumpWorker task created (stack=%d, priority=%d)", 
                 PUMP_WORKER_TASK_STACK_SIZE, PUMP_WORKER_TASK_PRIORITY);
    }

    // Restore lockout state from NVS if it was active before reboot
    restoreLockoutStateFromNVS();
}

// Destructor
PumpController::~PumpController()
{
    // Delete worker task first to prevent it from processing stale notifications
    if (pumpWorkerTaskHandle != NULL)
    {
        vTaskDelete(pumpWorkerTaskHandle);
        pumpWorkerTaskHandle = NULL;
        ESP_LOGI(TAG, "PumpWorker task deleted.");
    }
    
    if (pumpMonitoringTimer != NULL)
    {
        xTimerDelete(pumpMonitoringTimer, portMAX_DELAY);
        pumpMonitoringTimer = NULL;
    }
    stopManualLockoutTimer(); // Use helper to clean up lockout timer
    
    // Clean up state mutex
    if (_stateMutex != NULL)
    {
        vSemaphoreDelete(_stateMutex);
        _stateMutex = NULL;
        ESP_LOGI(TAG, "State mutex deleted.");
    }
    
    ESP_LOGI(TAG, "PumpController destroyed.");
}

// Static callback for the new pump monitoring timer
// LIGHTWEIGHT: Only sends notification to worker task (FreeRTOS best practice)
void PumpController::pumpMonitoringTimerCallback(TimerHandle_t xTimer)
{
    if (xTimer == NULL) {
        return;  // Silent fail - can't log in lightweight callback
    }
    
    PumpController *controller = static_cast<PumpController *>(pvTimerGetTimerID(xTimer));
    if (!controller || !controller->pumpWorkerTaskHandle)
    {
        return;  // Silent fail - worker task not available
    }
    
    // Signal worker task to run stall detection - non-blocking
    xTaskNotify(controller->pumpWorkerTaskHandle, NOTIFY_STALL_CHECK, eSetBits);
}

// Static callback for the manual lockout timer
// LIGHTWEIGHT: Only sends notification to worker task (FreeRTOS best practice)
void PumpController::manualLockoutTimerCallback(TimerHandle_t xTimer)
{
    if (xTimer == NULL) {
        return;  // Silent fail - can't log in lightweight callback
    }
    
    PumpController *controller = static_cast<PumpController *>(pvTimerGetTimerID(xTimer));
    if (!controller || !controller->pumpWorkerTaskHandle)
    {
        return;  // Silent fail - worker task not available
    }
    
    // Signal worker task to handle lockout expiry - non-blocking
    xTaskNotify(controller->pumpWorkerTaskHandle, NOTIFY_LOCKOUT_EXPIRED, eSetBits);
}

// Worker task that processes deferred work from timer callbacks
// This runs heavy operations (logging, MQTT, sensor reads) in its own stack context
void PumpController::pumpWorkerTask(void *pvParameters)
{
    PumpController *controller = static_cast<PumpController *>(pvParameters);
    uint32_t notificationValue;
    
    ESP_LOGI(TAG, "PumpWorker task started");
    
    for (;;)
    {
        // Wait indefinitely for notifications from timer callbacks
        if (xTaskNotifyWait(0, ULONG_MAX, &notificationValue, portMAX_DELAY) == pdTRUE)
        {
            // Process stall detection if requested
            if (notificationValue & NOTIFY_STALL_CHECK)
            {
                controller->processStallDetection();
            }
            
            // Process lockout expiry if requested
            if (notificationValue & NOTIFY_LOCKOUT_EXPIRED)
            {
                controller->handleLockoutExpired();
            }
        }
    }
}

// Handles lockout timer expiry - called from worker task (not timer callback)
// This contains the heavy work that was previously in manualLockoutTimerCallback
void PumpController::handleLockoutExpired()
{
    ESP_LOGI(TAG, "=== LOCKOUT TIMER EXPIRED ===");
    ESP_LOGI(TAG, "  Lockout period has ended");
    ESP_LOGI(TAG, "  Automatic pump restart is now permitted");
    
    isLockoutTimerActive = false;
    setManualPumpRestartRequired(false);
    
    // Update NVS to reflect that lockout is no longer active
    updateLockoutNVSStatus(false);
    
    // Publish lockout deactivation event to MQTT broker
    if (publishEventCallback)
    {
        uint64_t currentTimestamp = getCurrentTimestampMillis();
        cJSON *eventData = cJSON_CreateObject();
        if (eventData == NULL)
        {
            ESP_LOGE(TAG, "Failed to create JSON object for lockout deactivation event - out of memory");
        }
        else
        {
            cJSON_AddNumberToObject(eventData, "timestamp", (double)currentTimestamp);
            publishEventCallback(EVENT_TYPE_LOCKOUT_DEACTIVATED, EVENT_LEVEL_INFO, eventData);
            cJSON_Delete(eventData);
            ESP_LOGI(TAG, "  Lockout deactivation event published to MQTT");
        }
    }
    
    // Check water level and restart pump if low
    SensorData *currentSensorData = NULL;
    if (sensorRepo.getSensorData(currentSensorData) == DATA_FETCH_SUCCESS && currentSensorData != NULL)
    {
        uint8_t currentWaterLevel = currentSensorData->waterLevel;
        free(currentSensorData);

        ESP_LOGI(TAG, "  Current Water Level: %d%%", currentWaterLevel);
        ESP_LOGI(TAG, "  Auto-start Threshold: %d%%", minWaterLevelToStart);

        if (currentWaterLevel <= minWaterLevelToStart)
        {
            ESP_LOGI(TAG, "  Decision: AUTO-RESTART PUMP");
            ESP_LOGI(TAG, "  Reason: Water level (%d%%) <= threshold (%d%%)",
                     currentWaterLevel, minWaterLevelToStart);
            
            setManualPumpRestartRequired(false);

            uint64_t currentTimestamp = getCurrentTimestampMillis();
            turnWaterPumpOn(currentTimestamp, PUMP_START_REASON_LOCKOUT_EXPIRED);
            ESP_LOGI(TAG, "=== LOCKOUT EXPIRED - PUMP RESTARTED ===");
        }
        else
        {
            ESP_LOGI(TAG, "  Decision: NO AUTO-RESTART");
            ESP_LOGI(TAG, "  Reason: Water level (%d%%) > threshold (%d%%)",
                     currentWaterLevel, minWaterLevelToStart);
            ESP_LOGI(TAG, "  Pump will auto-start when water level drops to %d%% or below",
                     minWaterLevelToStart);
            ESP_LOGI(TAG, "=== LOCKOUT EXPIRED - NO ACTION NEEDED ===");
        }
    }
    else
    {
        ESP_LOGE(TAG, "  ERROR: Failed to get sensor data");
        ESP_LOGE(TAG, "  Cannot determine if auto-restart is needed");
        ESP_LOGE(TAG, "=== LOCKOUT EXPIRED - ERROR ===");
    }
}

// New private method to handle the core logic of pump monitoring
void PumpController::processStallDetection()
{
    // CRITICAL: Stall detection ALWAYS runs regardless of lockout feature state
    // The lockout feature only controls whether a restart delay is enforced,
    // NOT whether stall detection monitors pump operation (safety-critical)
    
    if (isRecalibrationModeActive)
    {
        return;
    }

    SensorData *currentSensorData = NULL;
    if (sensorRepo.getSensorData(currentSensorData) == DATA_FETCH_SUCCESS && currentSensorData != NULL)
    {
        uint8_t currentWaterLevel = currentSensorData->waterLevel;
        uint64_t currentSequenceNumber = currentSensorData->sequenceNumber;
        free(currentSensorData);

        // Update EMA
        ema_water_level = (EMA_ALPHA * currentWaterLevel) + ((1 - EMA_ALPHA) * ema_water_level);

        switch (monitoringState)
        {
        case GRACE_PERIOD:
        {
            // Early exit: EMA of water level has risen ≥2% above start level.
            if (ema_water_level >= pumpStartWaterLevel + 2.0f)
            {
                ESP_LOGI(TAG, "Stall Detection: Water rising during GRACE_PERIOD!");
                ESP_LOGI(TAG, "  EMA Level: %.1f%% >= %d%% + 2%% threshold", ema_water_level, pumpStartWaterLevel);
                ESP_LOGI(TAG, "  Transitioning to MONITORING (early exit)");

                monitoringState = MONITORING;
                lastLevelChangeTimestamp = esp_timer_get_time();
                lastWaterLevelForStallCheck = currentWaterLevel;
            }
            else if (esp_timer_get_time() >= gracePeriodEndTimestamp)
            {
                ESP_LOGW(TAG, "Stall Detection: Protected window expired without water level rise!");
                ESP_LOGW(TAG, "  Start Level: %d%%, EMA Level: %.1f%%", pumpStartWaterLevel, ema_water_level);
                ESP_LOGW(TAG, "  Transitioning to STALL_SUSPECTED");

                monitoringState = STALL_SUSPECTED;
                stall_check_start_timestamp = esp_timer_get_time();
                stall_confirmation_count = 0;
                stallCheckReadingsCount = 0;
                initialWaterLevelForStallCheck = currentWaterLevel;
                initialStallCheckSeqNum = currentSequenceNumber;
            }
            else
            {
                int64_t remainingSeconds = (gracePeriodEndTimestamp - esp_timer_get_time()) / 1000000;
                ESP_LOGD(TAG, "Stall Detection: In GRACE_PERIOD, %lld seconds remaining, EMA=%.1f%%", remainingSeconds, ema_water_level);
            }
            break;
        }
        case MONITORING:
        {
            // Dynamic timeout calculation based on Kalman filter's estimated SPPR
            int64_t timeElapsedMicros = esp_timer_get_time() - lastLevelChangeTimestamp;
            int64_t timeElapsedSeconds = timeElapsedMicros / 1000000;

            // Calculate dynamic timeout: kf_x_hat * 2.5
            int64_t dynamicTimeout = (int64_t)(kf_x_hat * 2.5f);

            // Ensure timeout is reasonable (at least 30 seconds, max 300 seconds)
            if (dynamicTimeout < 30)
                dynamicTimeout = 30;
            if (dynamicTimeout > 300)
                dynamicTimeout = 300;

            // Calculate dynamic stall threshold
            float stall_threshold = kf_x_hat + (kf_k * sqrtf(kf_P));

            // Log every 10 seconds to avoid log spam
            static int64_t lastMonitoringLogTime = 0;
            int64_t currentTime = esp_timer_get_time() / 1000000; // Convert to seconds
            
            if (currentTime - lastMonitoringLogTime >= 10)
            {
                ESP_LOGI(TAG, "=== PUMP MONITORING STATUS ===");
                ESP_LOGI(TAG, "  Water Level: %d%%", currentWaterLevel);
                ESP_LOGI(TAG, "  EMA Water Level: %.2f%%", ema_water_level);
                ESP_LOGI(TAG, "  Time Since Last Change: %lld seconds", timeElapsedSeconds);
                ESP_LOGI(TAG, "  Kalman Filter State:");
                ESP_LOGI(TAG, "    SPPR Estimate (x_hat): %.2f s/%%", kf_x_hat);
                ESP_LOGI(TAG, "    Uncertainty (P): %.4f", kf_P);
                ESP_LOGI(TAG, "    Process Noise (Q): %.4f", kf_Q);
                ESP_LOGI(TAG, "    Measurement Noise (R): %.2f", kf_R);
                ESP_LOGI(TAG, "    Stall Multiplier (k): %.2f", kf_k);
                ESP_LOGI(TAG, "  Stall Detection:");
                ESP_LOGI(TAG, "    Dynamic Timeout: %lld seconds", dynamicTimeout);
                ESP_LOGI(TAG, "    Stall Threshold: %.2f s/%% (x_hat + k*sqrt(P))", stall_threshold);
                ESP_LOGI(TAG, "    Time Until Timeout: %lld seconds", dynamicTimeout - timeElapsedSeconds);
                lastMonitoringLogTime = currentTime;
            }

            // Check if water level has increased (indicating pump is working)
            if (currentWaterLevel > lastWaterLevelForStallCheck)
            {
                int8_t levelIncrease = currentWaterLevel - lastWaterLevelForStallCheck;
                ESP_LOGI(TAG, "Stall Detection: Water level INCREASED by %d%% (%d%% -> %d%%)",
                         levelIncrease, lastWaterLevelForStallCheck, currentWaterLevel);
                ESP_LOGI(TAG, "  Resetting timeout timer - pump is working normally");
                lastWaterLevelForStallCheck = currentWaterLevel;
                lastLevelChangeTimestamp = esp_timer_get_time();
            }
            
            if (timeElapsedSeconds >= dynamicTimeout)
            {
                // Timeout exceeded, transition to STALL_SUSPECTED state
                ESP_LOGW(TAG, "=== STALL SUSPECTED ===");
                ESP_LOGW(TAG, "  Dynamic timeout (%lld seconds) exceeded", dynamicTimeout);
                ESP_LOGW(TAG, "  Current Water Level: %d%%", currentWaterLevel);
                ESP_LOGW(TAG, "  Last Water Level: %d%%", lastWaterLevelForStallCheck);
                ESP_LOGW(TAG, "  No water level increase detected for %lld seconds", timeElapsedSeconds);
                ESP_LOGW(TAG, "  Transitioning to STALL_SUSPECTED state for confirmation");
                monitoringState = STALL_SUSPECTED;
                stall_check_start_timestamp = esp_timer_get_time();
                stall_confirmation_count = 0;
                stallCheckReadingsCount = 0;
                // Store initial water level and sequence number for comparison
                initialWaterLevelForStallCheck = currentWaterLevel;
                initialStallCheckSeqNum = currentSequenceNumber;
            }
            break;
        }
        case STALL_SUSPECTED:
        {
            // Take water level readings over a short period (15 seconds)
            int64_t timeInStallSuspectedState = (esp_timer_get_time() - stall_check_start_timestamp) / 1000000;

            // Take a reading every 5 seconds, up to 3 readings
            if (timeInStallSuspectedState >= (stallCheckReadingsCount + 1) * 5 && stallCheckReadingsCount < 3)
            {
                // Store the reading and sequence number
                stallCheckReadings[stallCheckReadingsCount] = currentWaterLevel;
                stallCheckSequenceNumbers[stallCheckReadingsCount] = currentSequenceNumber;
                stallCheckReadingsCount++;

                ESP_LOGI(TAG, "Stall Detection: Confirmation check %d/3 - WaterLevel=%d%%, SeqNum=%llu, InitialLevel=%d%%, Delta=%d%%",
                         stallCheckReadingsCount, currentWaterLevel, (unsigned long long)currentSequenceNumber, initialWaterLevelForStallCheck,
                         (int)currentWaterLevel - (int)initialWaterLevelForStallCheck);
            }

            // After 15 seconds (3 readings), make a decision
            if (timeInStallSuspectedState >= 15 && stallCheckReadingsCount >= 3)
            {
                // Count how many readings showed an increase compared to the initial level, excluding stale packets
                uint8_t validReadings = 0;
                uint8_t increaseCount = 0;
                for (int i = 0; i < 3; i++)
                {
                    uint64_t prevSeqNum = (i == 0) ? initialStallCheckSeqNum : stallCheckSequenceNumbers[i-1];
                    bool isFresh = (stallCheckSequenceNumbers[i] != prevSeqNum);

                    if (isFresh)
                    {
                        validReadings++;
                        if (stallCheckReadings[i] > initialWaterLevelForStallCheck)
                        {
                            increaseCount++;
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG, "  Reading %d: STALE (sequence number %llu unchanged)", i, (unsigned long long)stallCheckSequenceNumbers[i]);
                    }
                }

                if (validReadings < 2)
                {
                    ESP_LOGW(TAG, "Stall Confirmation: Only %d/3 fresh readings (RF dropout). Restarting confirmation check.", validReadings);
                    stall_check_start_timestamp = esp_timer_get_time();
                    stallCheckReadingsCount = 0;
                }
                else if (increaseCount * 2 >= validReadings)
                {
                    ESP_LOGI(TAG, "Stall Detection: STALL NOT CONFIRMED - %d/%d valid readings showed increase. Returning to MONITORING.",
                             increaseCount, validReadings);
                    monitoringState = MONITORING;
                    // Reset the slow check timer
                    lastLevelChangeTimestamp = esp_timer_get_time();
                }
                else
                {
                    // Otherwise, confirm stall and turn off pump
                    ESP_LOGE(TAG, "Stall Detection: STALL CONFIRMED - Only %d/%d valid readings showed increase. InitialLevel=%d%%. TURNING OFF PUMP!",
                             increaseCount, validReadings, initialWaterLevelForStallCheck);
                    setManualPumpRestartRequired(true);
                    turnWaterPumpOff(getCurrentTimestampMillis(), PUMP_STOP_REASON_STALL_DETECTED);
                }
            }
            break;
        }
        case IDLE:
        default:
            // Do nothing
            break;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Stall check: Failed to get current sensor data.");
    }
}

void PumpController::turnWaterPumpOn(uint64_t currentTimestamp, PumpStartReason reason, bool lockoutWasCleared)
{
    ESP_LOGI(TAG, "=== TURN WATER PUMP ON CALLED ===");
    ESP_LOGI(TAG, "  Start Reason: %s", pumpStartReasonToString(reason));
    ESP_LOGI(TAG, "  Current Pump State: %s", periphManager.getCurrentMotorState() == OFF ? "OFF" : "ON");
    ESP_LOGI(TAG, "  Recalibration Mode: %s", isRecalibrationModeActive ? "ACTIVE" : "INACTIVE");
    ESP_LOGI(TAG, "  Lockout Cleared: %s", lockoutWasCleared ? "YES" : "NO");
    
    // Check pump state first (outside critical section to avoid calling methods in critical section)
    bool pumpIsOff = (periphManager.getCurrentMotorState() == OFF);
    
    // Atomically check and set the "start in progress" flag to prevent race condition
    // This prevents multiple tasks from starting the pump simultaneously (double buzzer issue)
    portENTER_CRITICAL(&_pumpStartSpinlock);
    bool startAlreadyInProgress = _pumpStartInProgress;
    if (pumpIsOff && !startAlreadyInProgress) {
        _pumpStartInProgress = true;  // Claim exclusive access to start operation
    }
    portEXIT_CRITICAL(&_pumpStartSpinlock);
    
    // Check if we can proceed with pump start
    if (!pumpIsOff) {
        ESP_LOGW(TAG, "=== WATER PUMP ON SKIPPED ===");
        ESP_LOGW(TAG, "  Pump is already ON");
        ESP_LOGW(TAG, "  No action taken");
        return;
    }
    
    if (startAlreadyInProgress) {
        ESP_LOGW(TAG, "=== WATER PUMP ON SKIPPED ===");
        ESP_LOGW(TAG, "  Pump start already in progress by another task");
        ESP_LOGW(TAG, "  No action taken (prevents double buzzer)");
        return;
    }
    
    // At this point, we have exclusive access to start the pump
    ESP_LOGI(TAG, "Pump is OFF - Proceeding with startup sequence");
    
    // CORRECT SEQUENCE: Play buzzer tune FIRST, then activate relay
    // This ensures user hears the tune before the relay clicks
    buzzControl.playMotorOnTune();
    
    // Now activate the relay via Switching Unit and update state
    uint16_t cooldown_sec = 0;
    esp_err_t err = switchingUnit.sendPumpOn(&cooldown_sec);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_NOT_ALLOWED)
        {
            // SwitchingUnit is in LOCKED state — pump cannot be started or stopped
            ESP_LOGW(TAG, "Pump start rejected: SwitchingUnit is in LOCKED state.");
            ESP_LOGW(TAG, "  Use CMD_PUMP_UNLOCK to exit lockout before controlling the pump.");
            
            if (publishEventCallback)
            {
                cJSON *eventData = cJSON_CreateObject();
                if (eventData != NULL)
                {
                    cJSON_AddNumberToObject(eventData, "timestamp", (double)currentTimestamp);
                    cJSON_AddStringToObject(eventData, "reason", "switching_unit_locked");
                    publishEventCallback(EVENT_TYPE_WATER_PUMP_ON, EVENT_LEVEL_ALERT, eventData);
                    cJSON_Delete(eventData);
                }
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to turn on water pump: 0x%x", err);
            if (cooldown_sec > 0)
            {
                ESP_LOGW(TAG, "Pump is in cooldown. Remaining: %u seconds.", cooldown_sec);
                setManualPumpRestartRequired(true);
                
                // Only start lockout timer if lockout feature is enabled
                if (isLockoutFeatureEnabled())
                {
                    ESP_LOGI(TAG, "Starting lockout timer for cooldown duration: %u seconds", cooldown_sec);
                    uint32_t old_duration = manualLockoutDurationMillis;
                    manualLockoutDurationMillis = cooldown_sec * 1000;
                    startManualLockoutTimer(currentTimestamp);
                    manualLockoutDurationMillis = old_duration;
                }
            }
        }
        
        // Clear the "start in progress" flag
        portENTER_CRITICAL(&_pumpStartSpinlock);
        _pumpStartInProgress = false;
        portEXIT_CRITICAL(&_pumpStartSpinlock);
        return;
    }
    
    periphManager.setMotorToggleButtonLedState(ON);

    pumpStartTimeMs = esp_timer_get_time() / 1000;
    SensorData *data = nullptr;
    if (sensorRepo.getSensorData(data) == DATA_FETCH_SUCCESS && data != nullptr)
    {
        pumpStartWaterLevel = data->waterLevel;
        free(data);
        ESP_LOGI(TAG, "Pump Start: Water Level = %d%%", pumpStartWaterLevel);
    }
    else
    {
        pumpStartWaterLevel = 0; // Default if data is unavailable
        ESP_LOGW(TAG, "Pump Start: Could not get water level, using default 0%%");
    }

        consecutiveLowIncrementChecks = 0;
        consecutiveNoWaterRiseChecks = 0;
        fast_check_start_level = 0.0f;

        // DECOUPLED ARCHITECTURE: Initialize Kalman filter ALWAYS (independent of stall detection)
        // This provides SPPR context for both normal and calibration modes
        ESP_LOGI(TAG, "=== INITIALIZING KALMAN FILTER ===");
        
        // Initialize Kalman filter state with safe defaults
        kf_x_hat = 15.0f; // A safe, default starting guess for SPPR
        kf_P = 100.0f;    // High initial uncertainty

        // Load Kalman filter state from NVS if available
        float saved_kf_x_hat, saved_kf_P;
        if (nvsManager.getKalmanStateXHat(saved_kf_x_hat) && nvsManager.getKalmanStateP(saved_kf_P))
        {
            // Use saved state if it's reasonable (positive values)
            if (saved_kf_x_hat > 0 && saved_kf_P > 0)
            {
                kf_x_hat = saved_kf_x_hat;
                kf_P = saved_kf_P;
                ESP_LOGI(TAG, "  Kalman Filter: Loaded from NVS");
                ESP_LOGI(TAG, "    SPPR Estimate: %.2fs/%%", kf_x_hat);
                ESP_LOGI(TAG, "    Uncertainty: %.2f", kf_P);
            }
            else
            {
                ESP_LOGW(TAG, "  Kalman Filter: Invalid state in NVS (x_hat=%.2f, P=%.2f)", 
                         saved_kf_x_hat, saved_kf_P);
                ESP_LOGI(TAG, "  Using defaults: SPPR=%.2fs/%%, Uncertainty=%.2f", kf_x_hat, kf_P);
            }
        }
        else
        {
            ESP_LOGI(TAG, "  Kalman Filter: No saved state in NVS");
            ESP_LOGI(TAG, "  Using defaults: SPPR=%.2fs/%%, Uncertainty=%.2f", kf_x_hat, kf_P);
        }

        // STALL DETECTION: Only initialize in normal mode (decoupled from Kalman filter)
        if (isRecalibrationModeActive)
        {
            ESP_LOGI(TAG, "=== CALIBRATION MODE ACTIVE ===");
            ESP_LOGI(TAG, "  Stall detection: DISABLED");
            ESP_LOGI(TAG, "  Monitoring state: IDLE (no monitoring)");
            ESP_LOGI(TAG, "  SPPR calculation: ENABLED (adaptive Kalman filter)");
            ESP_LOGI(TAG, "  Assumption: Sufficient water supply available");
            ESP_LOGI(TAG, "  Purpose: Measure SPPR without stall detection interference");
            ESP_LOGI(TAG, "  Pump will run until water level reaches stop threshold");
            
            monitoringState = IDLE; // No monitoring in calibration mode
            pumpMonitoringTimer = NULL; // No timer needed
        }
        else
        {
            ESP_LOGI(TAG, "=== NORMAL MODE ACTIVE ===");
            ESP_LOGI(TAG, "  Stall detection: ENABLED");
            ESP_LOGI(TAG, "  SPPR calculation: ENABLED (adaptive Kalman filter)");
            
            // Initialize monitoring state based on water level
            if (pumpStartWaterLevel == 0)
            {
                monitoringState = GRACE_PERIOD;
                gracePeriodEndTimestamp = esp_timer_get_time() + (CONFIG_PUMP_STALL_GRACE_PERIOD_SEC * 1000000LL);
                ema_water_level = 0.0f;
                previous_ema_water_level = 0.0f;
                ESP_LOGI(TAG, "  Monitoring: GRACE_PERIOD (%d seconds)", CONFIG_PUMP_STALL_GRACE_PERIOD_SEC);
            }
            else
            {
                monitoringState = MONITORING;
                ema_water_level = pumpStartWaterLevel;
                previous_ema_water_level = pumpStartWaterLevel;
                ESP_LOGI(TAG, "  Monitoring: ACTIVE");
            }
            
            // Log the initial stall detection threshold
            float initial_stall_threshold = kf_x_hat + (kf_k * sqrtf(kf_P));
            ESP_LOGI(TAG, "  Stall Detection Threshold: %.2fs/%% (SPPR + %.1f*sqrt(P))",
                     initial_stall_threshold, kf_k);

            // Initialize stall detection variables
            SensorData *currentSensorData = NULL;
            if (sensorRepo.getSensorData(currentSensorData) == DATA_FETCH_SUCCESS && currentSensorData != NULL)
            {
                lastWaterLevelForStallCheck = currentSensorData->waterLevel;
                free(currentSensorData);
            }
            else
            {
                lastWaterLevelForStallCheck = 0; // Default if no data available
                ESP_LOGW(TAG, "  Could not get initial water level for stall detection");
            }
            lastLevelChangeTimestamp = esp_timer_get_time();
            ESP_LOGI(TAG, "  Stall Detection: Initialized (level=%d, timestamp=%lld)", 
                     lastWaterLevelForStallCheck, lastLevelChangeTimestamp);

            // Create and start monitoring timer
            if (pumpMonitoringTimer != NULL)
            { // Delete existing timer if any
                xTimerDelete(pumpMonitoringTimer, portMAX_DELAY);
            }

            pumpMonitoringTimer = xTimerCreate(
                "pumpStallTimer",
                pdMS_TO_TICKS(CONFIG_PUMP_IMMEDIATE_STALL_CHECK_INTERVAL_MS),
                pdTRUE, // Auto-reload timer
                this,   // Pass PumpController instance as Timer ID
                pumpMonitoringTimerCallback);

            if (pumpMonitoringTimer != NULL)
            {
                if (xTimerStart(pumpMonitoringTimer, portMAX_DELAY) == pdPASS)
                {
                    ESP_LOGI(TAG, "  Pump Monitoring Timer: Created and Started");
                }
                else
                {
                    ESP_LOGE(TAG, "  Pump Monitoring Timer: Failed to start");
                    xTimerDelete(pumpMonitoringTimer, portMAX_DELAY);
                    pumpMonitoringTimer = NULL;
                }
            }
            else
            {
                ESP_LOGE(TAG, "  Pump Monitoring Timer: Failed to create");
            }
        }

        if (publishEventCallback)
        {
            lastStartReason = reason;
            cJSON *eventData = cJSON_CreateObject();
            if (eventData == NULL)
            {
                ESP_LOGE(TAG, "Failed to create JSON object for pump start event - out of memory");
            }
            else
            {
                cJSON_AddNumberToObject(eventData, "timestamp", currentTimestamp);
                cJSON_AddStringToObject(eventData, "startReason", pumpStartReasonToString(lastStartReason));
                cJSON_AddNumberToObject(eventData, "waterLevelPercent", pumpStartWaterLevel);
                
                // Add lockout deactivation info if lockout was cleared
                if (lockoutWasCleared)
                {
                    cJSON_AddBoolToObject(eventData, "lockoutDeactivated", true);
                    ESP_LOGI(TAG, "Lockout was cleared - included in PUMP_ON event");
                }
                
                publishEventCallback(EVENT_TYPE_WATER_PUMP_ON, EVENT_LEVEL_INFO, eventData);
                cJSON_Delete(eventData);
            }
        }
        
        ESP_LOGI(TAG, "=== WATER PUMP ON COMPLETE ===");
        ESP_LOGI(TAG, "  Pump successfully started");
        ESP_LOGI(TAG, "  Final State: ON");
        ESP_LOGI(TAG, "  Monitoring: ACTIVE");
        
        // Clear the "start in progress" flag now that startup is complete
        portENTER_CRITICAL(&_pumpStartSpinlock);
        _pumpStartInProgress = false;
        portEXIT_CRITICAL(&_pumpStartSpinlock);
}

void PumpController::turnWaterPumpOff(uint64_t currentTimestamp, PumpStopReason reason)
{
    ESP_LOGI(TAG, "=== TURN WATER PUMP OFF CALLED ===");
    ESP_LOGI(TAG, "  Stop Reason: %s", pumpStopReasonToString(reason));
    ESP_LOGI(TAG, "  Current Pump State: %s", periphManager.getCurrentMotorState() == ON ? "ON" : "OFF");
    ESP_LOGI(TAG, "  Recalibration Mode: %s", isRecalibrationModeActive ? "ACTIVE" : "INACTIVE");
    
    if (periphManager.getCurrentMotorState() == ON)
    {
        ESP_LOGI(TAG, "Pump is ON - Proceeding with shutdown sequence");

        monitoringState = IDLE;

        // Save recalibration mode state before potentially disabling it
        // This ensures SPPR calculation uses the correct logic path
        bool wasInRecalibrationMode = isRecalibrationModeActive;

        // CRITICAL FIX: Check if calibration mode should be automatically disabled
        // Calibration mode exits when pump stops automatically with ≥10% water level change
        if (isRecalibrationModeActive && reason == PUMP_STOP_REASON_WATER_LEVEL_REACHED)
        {
            // Calculate water level change during this pump cycle
            SensorData *currentData = nullptr;
            uint8_t currentWaterLevel = 0;
            if (sensorRepo.getSensorData(currentData) == DATA_FETCH_SUCCESS && currentData != nullptr)
            {
                currentWaterLevel = currentData->waterLevel;
                free(currentData);
            }
            
            int8_t waterLevelDelta = currentWaterLevel - pumpStartWaterLevel;
            
            ESP_LOGI(TAG, "=== CALIBRATION MODE AUTO-EXIT CHECK ===");
            ESP_LOGI(TAG, "  Start Level: %d%%, Stop Level: %d%%, Delta: %d%%",
                     pumpStartWaterLevel, currentWaterLevel, waterLevelDelta);
            ESP_LOGI(TAG, "  Required Delta: >= 10%%");
            
            if (waterLevelDelta >= 10)
            {
                ESP_LOGI(TAG, "  Decision: EXIT CALIBRATION MODE");
                ESP_LOGI(TAG, "  Reason: Water level change (≥10%%) achieved");
                ESP_LOGI(TAG, "  SPPR will be calculated and stored to NVS");
                setRecalibrationMode(false);
            }
            else
            {
                ESP_LOGI(TAG, "  Decision: KEEP CALIBRATION MODE ACTIVE");
                ESP_LOGI(TAG, "  Reason: Water level change (%d%%) < 10%%", waterLevelDelta);
                ESP_LOGI(TAG, "  Calibration mode remains active for next pump cycle");
            }
        }

        buzzControl.playMotorOFFTune();
        esp_err_t offErr = switchingUnit.sendPumpOff();
        if (offErr == ESP_ERR_NOT_ALLOWED)
        {
            ESP_LOGW(TAG, "sendPumpOff rejected: SwitchingUnit is in LOCKED state.");
            ESP_LOGW(TAG, "  Pump hardware may not have been deactivated.");
        }
        else if (offErr != ESP_OK)
        {
            ESP_LOGE(TAG, "sendPumpOff failed: 0x%x", offErr);
        }
        periphManager.setMotorToggleButtonLedState(OFF);

        if (pumpMonitoringTimer != NULL)
        {
            xTimerStop(pumpMonitoringTimer, portMAX_DELAY);
            xTimerDelete(pumpMonitoringTimer, portMAX_DELAY);
            pumpMonitoringTimer = NULL;
            ESP_LOGI(TAG, "pumpMonitoringTimer stopped and deleted.");
        }
        consecutiveLowIncrementChecks = 0; // Reset counter

        // Check if lockout timer should be started for manual operations
        if (reason == PUMP_STOP_REASON_MANUAL || reason == PUMP_STOP_REASON_REMOTE_COMMAND)
        {
            // Get current water level to determine if lockout is needed
            SensorData *currentData = nullptr;
            uint8_t currentWaterLevel = 0;
            if (sensorRepo.getSensorData(currentData) == DATA_FETCH_SUCCESS && currentData != nullptr)
            {
                currentWaterLevel = currentData->waterLevel;
                free(currentData);
            }
            else
            {
                ESP_LOGW(TAG, "Could not get current water level for lockout decision. Assuming lockout needed.");
                currentWaterLevel = 0; // Assume low level if data unavailable
            }

            // Calculate lockout threshold: minWaterLevelToStart + 10
            uint8_t lockoutThreshold = minWaterLevelToStart + TOLERANCE_LEVEL;
            if (lockoutThreshold > 100)
                lockoutThreshold = 100; // Cap at 100%

            if (currentWaterLevel <= lockoutThreshold)
            {
                ESP_LOGI(TAG, "Water level (%d%%) is <= lockout threshold (%d%%).",
                         currentWaterLevel, lockoutThreshold);
                setManualPumpRestartRequired(true);
                
                // Only start lockout timer if lockout feature is enabled
                if (isLockoutFeatureEnabled())
                {
                    ESP_LOGI(TAG, "Lockout feature ENABLED - Starting lockout timer");
                    startManualLockoutTimer(currentTimestamp);
                }
                else
                {
                    ESP_LOGI(TAG, "Lockout feature DISABLED - No lockout timer, immediate restart allowed");
                }
            }
            else
            {
                ESP_LOGI(TAG, "Water level (%d%%) is > lockout threshold (%d%%). No lockout timer needed.",
                         currentWaterLevel, lockoutThreshold);
            }
        }
        // If this turn-off was due to a required manual restart (stall detection), start the lockout timer.
        else if (manualPumpRestartRequired)
        {
            // Only start lockout timer if lockout feature is enabled
            if (isLockoutFeatureEnabled())
            {
                ESP_LOGI(TAG, "Stall detected - Lockout feature ENABLED - Starting lockout timer");
                startManualLockoutTimer(currentTimestamp);
            }
            else
            {
                ESP_LOGI(TAG, "Stall detected - Lockout feature DISABLED - No lockout timer, immediate restart allowed");
            }
        }

        // NOTE: Kalman filter state will be saved to NVS AFTER SPPR calculation
        // This ensures the updated SPPR value is saved, not the old value

        if (publishEventCallback)
        {
            lastStopReason = reason;
            int64_t runDurationMs = (esp_timer_get_time() / 1000) - pumpStartTimeMs;

            SensorData *currentData = nullptr;
            uint8_t currentWaterLevel = 0;
            if (sensorRepo.getSensorData(currentData) == DATA_FETCH_SUCCESS && currentData != nullptr)
            {
                currentWaterLevel = currentData->waterLevel;
                free(currentData);
            }

            int8_t waterLevelDelta = currentWaterLevel - pumpStartWaterLevel;

            // Log pump cycle summary for SPPR calculation
            ESP_LOGI(TAG, "=== PUMP CYCLE SUMMARY ===");
            ESP_LOGI(TAG, "  Start Level: %d%%, Stop Level: %d%%, Delta: %d%%",
                     pumpStartWaterLevel, currentWaterLevel, waterLevelDelta);
            ESP_LOGI(TAG, "  Run Duration: %.1fs (%.1f minutes)",
                     runDurationMs / 1000.0f, runDurationMs / 60000.0f);
            ESP_LOGI(TAG, "  Stop Reason: %s", pumpStopReasonToString(reason));
            ESP_LOGI(TAG, "  Recalibration Mode: %s", wasInRecalibrationMode ? "ACTIVE" : "INACTIVE");
            ESP_LOGI(TAG, "  Current SPPR Estimate: %.2fs/%% (Uncertainty: %.2f)", kf_x_hat, kf_P);

            // Calculate SPPR (Seconds Per Percent Rise) from this pump run
            // Only calculate if water level increased by at least 5%
            const int8_t MIN_WATER_LEVEL_DELTA_FOR_SPPR = 5;
            float measuredSPPR = 0.0f;
            
            if (waterLevelDelta >= MIN_WATER_LEVEL_DELTA_FOR_SPPR)
            {
                measuredSPPR = (float)(runDurationMs / 1000.0f) / (float)waterLevelDelta;
                ESP_LOGI(TAG, "SPPR Calculation: RunDuration=%.1fs, WaterLevelDelta=%d%%, MeasuredSPPR=%.2fs/%%",
                         runDurationMs / 1000.0f, waterLevelDelta, measuredSPPR);

                // Check for unusually high SPPR values
                if (measuredSPPR > 100.0f)
                {
                    ESP_LOGW(TAG, "SPPR WARNING: Measured SPPR (%.2fs/%%) is very high! This indicates slow water supply or sensor issues.",
                             measuredSPPR);
                }
                else if (measuredSPPR > 50.0f)
                {
                    ESP_LOGW(TAG, "SPPR NOTICE: Measured SPPR (%.2fs/%%) is higher than normal (expected 10-20s/%%).",
                             measuredSPPR);
                }

                // ADAPTIVE KALMAN FILTER: Use for both normal and calibration modes
                // Only skip if stall was detected (unreliable data)
                if (reason != PUMP_STOP_REASON_STALL_DETECTED)
                {
                    // Determine process noise based on mode
                    // Calibration mode: High Q (100.0) for fast convergence (Kalman gain ≈ 0.99)
                    // Normal mode: Low Q (0.01) for gradual refinement (Kalman gain starts at 0.91)
                    float process_noise_Q = wasInRecalibrationMode ? 100.0f : kf_Q;
                    
                    if (wasInRecalibrationMode)
                    {
                        ESP_LOGI(TAG, "SPPR Update Path: CALIBRATION MODE (Adaptive Kalman Filter)");
                        ESP_LOGI(TAG, "  Process Noise (Q): %.2f (high for fast convergence)", process_noise_Q);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "SPPR Update Path: NORMAL MODE (Adaptive Kalman Filter)");
                        ESP_LOGI(TAG, "  Process Noise (Q): %.4f (low for gradual refinement)", process_noise_Q);
                    }
                    
                    // Store previous values for logging
                    float prev_kf_x_hat = kf_x_hat;
                    float prev_kf_P = kf_P;

                    // Kalman Filter Prediction Step
                    // P_minus = P + Q
                    float kf_P_minus = kf_P + process_noise_Q;
                    
                    // Safety check: Ensure P_minus is positive
                    if (kf_P_minus <= 0.0f)
                    {
                        ESP_LOGE(TAG, "  SAFETY ERROR: P_minus (%.4f) is not positive! Resetting to 100.0", kf_P_minus);
                        kf_P_minus = 100.0f;
                    }

                    ESP_LOGI(TAG, "  Kalman Prediction Step:");
                    ESP_LOGI(TAG, "    Previous SPPR Estimate: %.2fs/%%", prev_kf_x_hat);
                    ESP_LOGI(TAG, "    Previous Uncertainty: %.2f", prev_kf_P);
                    ESP_LOGI(TAG, "    Predicted Uncertainty: %.2f", kf_P_minus);

                    // Kalman Filter Update Step
                    // Innovation: y = measurement - x_hat
                    float innovation = measuredSPPR - kf_x_hat;

                    // Kalman Gain: K = P_minus / (P_minus + R)
                    float denominator = kf_P_minus + kf_R;
                    
                    // Safety check: Ensure denominator is not zero
                    if (denominator <= 0.0f)
                    {
                        ESP_LOGE(TAG, "  SAFETY ERROR: Denominator (%.4f) is not positive! Skipping update", denominator);
                        ESP_LOGE(TAG, "  P_minus=%.4f, R=%.4f", kf_P_minus, kf_R);
                    }
                    else
                    {
                        float kalman_gain = kf_P_minus / denominator;
                        
                        // Safety check: Ensure Kalman gain is in valid range [0, 1]
                        if (kalman_gain < 0.0f || kalman_gain > 1.0f)
                        {
                            ESP_LOGW(TAG, "  SAFETY WARNING: Kalman gain (%.4f) out of range [0,1]! Clamping", kalman_gain);
                            kalman_gain = (kalman_gain < 0.0f) ? 0.0f : 1.0f;
                        }

                        // Update estimate: x_hat = x_hat + K * innovation
                        kf_x_hat = kf_x_hat + (kalman_gain * innovation);
                        
                        // Safety check: Ensure SPPR estimate is positive and reasonable
                        if (kf_x_hat <= 0.0f)
                        {
                            ESP_LOGE(TAG, "  SAFETY ERROR: Updated SPPR (%.2f) is not positive! Using measurement", kf_x_hat);
                            kf_x_hat = measuredSPPR;
                        }
                        else if (kf_x_hat > 200.0f)
                        {
                            ESP_LOGW(TAG, "  SAFETY WARNING: Updated SPPR (%.2f) is very high! Capping at 200.0", kf_x_hat);
                            kf_x_hat = 200.0f;
                        }

                        // Update uncertainty: P = (1 - K) * P_minus
                        kf_P = (1.0f - kalman_gain) * kf_P_minus;
                        
                        // Safety check: Ensure uncertainty is positive and bounded
                        if (kf_P <= 0.0f)
                        {
                            ESP_LOGW(TAG, "  SAFETY WARNING: Updated P (%.4f) is not positive! Resetting to 10.0", kf_P);
                            kf_P = 10.0f;
                        }
                        else if (kf_P > 1000.0f)
                        {
                            ESP_LOGW(TAG, "  SAFETY WARNING: Updated P (%.4f) is very high! Capping at 1000.0", kf_P);
                            kf_P = 1000.0f;
                        }

                        ESP_LOGI(TAG, "  Kalman Update Step:");
                        ESP_LOGI(TAG, "    Measurement (z): %.2fs/%%", measuredSPPR);
                        ESP_LOGI(TAG, "    Innovation (z - x_hat): %.2fs/%%", innovation);
                        ESP_LOGI(TAG, "    Measurement Noise (R): %.2f", kf_R);
                        ESP_LOGI(TAG, "    Kalman Gain (K): %.3f", kalman_gain);
                        ESP_LOGI(TAG, "    Updated SPPR Estimate: %.2f -> %.2f s/%%", prev_kf_x_hat, kf_x_hat);
                        ESP_LOGI(TAG, "    Updated Uncertainty: %.2f -> %.2f", prev_kf_P, kf_P);
                        
                        // Calculate and log the dynamic stall detection threshold
                        float stall_threshold = kf_x_hat + (kf_k * sqrtf(kf_P));
                        ESP_LOGI(TAG, "    New Stall Detection Threshold: %.2fs/%% (SPPR + %.1f*sqrt(P))",
                                 stall_threshold, kf_k);
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "SPPR Update Path: SKIPPED (Stall detected, SPPR not updated)");
                }
            }
            else if (waterLevelDelta > 0 && waterLevelDelta < MIN_WATER_LEVEL_DELTA_FOR_SPPR)
            {
                ESP_LOGW(TAG, "SPPR Calculation: SKIPPED - Water level change too small");
                ESP_LOGW(TAG, "  Start Level: %d%%, Stop Level: %d%%, Delta: %d%%",
                         pumpStartWaterLevel, currentWaterLevel, waterLevelDelta);
                ESP_LOGW(TAG, "  Minimum required delta: %d%%", MIN_WATER_LEVEL_DELTA_FOR_SPPR);
                ESP_LOGW(TAG, "  Reason: Small changes can lead to inaccurate SPPR measurements");
                ESP_LOGW(TAG, "  Recommendation: Let pump run longer for more accurate calibration");
                ESP_LOGW(TAG, "  Current SPPR estimate unchanged: %.2fs/%%", kf_x_hat);
            }
            else if (waterLevelDelta == 0)
            {
                ESP_LOGW(TAG, "SPPR Calculation: SKIPPED - No water level change detected");
                ESP_LOGW(TAG, "  Start Level: %d%%, Stop Level: %d%%, Delta: 0%%",
                         pumpStartWaterLevel, currentWaterLevel);
                ESP_LOGW(TAG, "  This may indicate sensor issues or pump not actually pumping water");
                ESP_LOGW(TAG, "  Current SPPR estimate unchanged: %.2fs/%%", kf_x_hat);
            }
            else
            {
                ESP_LOGW(TAG, "SPPR Calculation: SKIPPED - Negative water level delta detected");
                ESP_LOGW(TAG, "  Start Level: %d%%, Stop Level: %d%%, Delta: %d%%",
                         pumpStartWaterLevel, currentWaterLevel, waterLevelDelta);
                ESP_LOGW(TAG, "  This indicates water level decreased while pump was running!");
                ESP_LOGW(TAG, "  Possible causes: sensor error, water consumption, or leak");
                ESP_LOGW(TAG, "  Current SPPR estimate unchanged: %.2fs/%%", kf_x_hat);
            }
            
            ESP_LOGI(TAG, "=== END PUMP CYCLE SUMMARY ===");

            // Save Kalman filter state to NVS AFTER SPPR calculation (not before!)
            // This ensures we save the UPDATED value, not the old value
            if (reason != PUMP_STOP_REASON_STALL_DETECTED)
            {
                ESP_LOGI(TAG, "=== SAVING KALMAN FILTER STATE TO NVS ===");
                ESP_LOGI(TAG, "  SPPR (x_hat): %.2fs/%%", kf_x_hat);
                ESP_LOGI(TAG, "  Uncertainty (P): %.2f", kf_P);
                
                // Apply ceiling to SPPR before saving for faster integer calculations
                float ceiled_sppr = ceilf(kf_x_hat);
                ESP_LOGI(TAG, "  SPPR ceiled: %.2f -> %.0f s/%%", kf_x_hat, ceiled_sppr);
                
                // Save the full Kalman filter state (ceiled kf_x_hat and kf_P) to NVS
                if (nvsManager.setKalmanStateXHat(ceiled_sppr) && nvsManager.setKalmanStateP(kf_P))
                {
                    ESP_LOGI(TAG, "  Save Result: SUCCESS");
                    ESP_LOGI(TAG, "  Updated SPPR value (ceiled) stored to NVS");
                }
                else
                {
                    ESP_LOGE(TAG, "  Save Result: FAILED");
                    ESP_LOGE(TAG, "  Could not store Kalman filter state to NVS");
                    ESP_LOGE(TAG, "  This may be due to validation rejection or NVS error");
                }
                ESP_LOGI(TAG, "=== END NVS SAVE ===");
            }
            else
            {
                ESP_LOGW(TAG, "Kalman filter state NOT saved to NVS (stall detected)");
            }

            // Create and populate the metadata struct
            LastPumpRunMetaData metaData;
            metaData.startedAt = currentTimestamp - runDurationMs; // Approximate start time
            metaData.runDurationMs = runDurationMs;
            metaData.waterLevelDelta = waterLevelDelta;
            metaData.startReason = lastStartReason;
            metaData.stopReason = reason;

            // Save to NVS
            if (nvsManager.setLastPumpRunMetaData(metaData))
            {
                ESP_LOGI(TAG, "Pump Metadata: Saved to NVS - Duration=%.1fs, Delta=%d%%, StartReason=%s, StopReason=%s",
                         runDurationMs / 1000.0f, waterLevelDelta,
                         pumpStartReasonToString(lastStartReason), pumpStopReasonToString(reason));
            }
            else
            {
                ESP_LOGE(TAG, "Failed to save last pump run metadata to NVS");
            }

            cJSON *eventData = cJSON_CreateObject();
            if (eventData == NULL)
            {
                ESP_LOGE(TAG, "Failed to create JSON object for pump stop event - out of memory");
            }
            else
            {
                cJSON_AddNumberToObject(eventData, "timestamp", currentTimestamp);
                cJSON_AddNumberToObject(eventData, "runDurationMs", runDurationMs);
                cJSON_AddNumberToObject(eventData, "waterLevelDelta", waterLevelDelta);
                cJSON_AddStringToObject(eventData, "stopReason", pumpStopReasonToString(lastStopReason));

                // Add lockout information if lockout is being activated
                if (isLockoutTimerActive)
                {
                    cJSON_AddBoolToObject(eventData, "lockoutActive", true);
                    cJSON_AddNumberToObject(eventData, "lockoutDuration", (double)manualLockoutDurationMillis);
                    uint64_t lockoutEndsAt = currentTimestamp + manualLockoutDurationMillis;
                    cJSON_AddNumberToObject(eventData, "lockoutEndsAt", (double)lockoutEndsAt);
                    ESP_LOGI(TAG, "Lockout activated - Duration: %lu ms, Ends at: %llu", 
                             manualLockoutDurationMillis, lockoutEndsAt);
                }

                publishEventCallback(EVENT_TYPE_WATER_PUMP_OFF, EVENT_LEVEL_ALERT, eventData);
                cJSON_Delete(eventData);
            }
        }
        
        ESP_LOGI(TAG, "=== WATER PUMP OFF COMPLETE ===");
        ESP_LOGI(TAG, "  Pump successfully stopped");
        ESP_LOGI(TAG, "  Final State: OFF");
    }
    else
    {
        ESP_LOGW(TAG, "=== WATER PUMP OFF SKIPPED ===");
        ESP_LOGW(TAG, "  Pump is already OFF");
        ESP_LOGW(TAG, "  No action taken");
    }
}

void PumpController::handleWaterLevelChange(uint8_t newWaterLevel, uint8_t previousWaterLevel, uint64_t currentTimestamp)
{
    if (newWaterLevel <= minWaterLevelToStart)
    {
        if (periphManager.getCurrentMotorState() == OFF)
        {
            if (manualPumpRestartRequired)
            {
                // Only enforce lockout if the lockout feature is enabled
                if (isLockoutFeatureEnabled())
                {
                    if (!isLockoutTimerActive)
                    {
                        ESP_LOGI(TAG, "Water Level: LOW (%" PRIu8 "%% <= %" PRIu8 "%%) - Lockout expired, auto-restart allowed",
                                 newWaterLevel, minWaterLevelToStart);
                        setManualPumpRestartRequired(false); // Clear the flag to allow this one start
                        turnWaterPumpOn(currentTimestamp, PUMP_START_REASON_WATER_LEVEL_REACHED);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Water Level: LOW (%" PRIu8 "%% <= %" PRIu8 "%%) - Lockout ACTIVE, pump will not auto-start",
                                 newWaterLevel, minWaterLevelToStart);
                    }
                }
                else
                {
                    // Lockout feature is disabled, allow pump to start even if lockout flag is set
                    ESP_LOGI(TAG, "Water Level: LOW (%" PRIu8 "%% <= %" PRIu8 "%%) - Lockout feature DISABLED, auto-restart allowed",
                             newWaterLevel, minWaterLevelToStart);
                    setManualPumpRestartRequired(false);
                    turnWaterPumpOn(currentTimestamp, PUMP_START_REASON_WATER_LEVEL_REACHED);
                }
            }
            else
            {
                ESP_LOGI(TAG, "Water Level: LOW (%" PRIu8 "%% <= %" PRIu8 "%%) - Turning pump ON",
                         newWaterLevel, minWaterLevelToStart);
                turnWaterPumpOn(currentTimestamp, PUMP_START_REASON_WATER_LEVEL_REACHED);
            }
        }
    }
    else if (newWaterLevel >= maxWaterLevelToStop)
    {
        if (periphManager.getCurrentMotorState() == ON)
        {
            ESP_LOGI(TAG, "Water Level: HIGH (%" PRIu8 "%% >= %" PRIu8 "%%) - Turning pump OFF",
                     newWaterLevel, maxWaterLevelToStop);
            turnWaterPumpOff(currentTimestamp, PUMP_STOP_REASON_WATER_LEVEL_REACHED);
        }
    }
}

bool PumpController::attemptManualPumpStart(uint64_t currentTimestamp, PumpStartReason reason)
{
    // Get current water level from sensor data
    SensorData *currentSensorData = nullptr;
    uint8_t currentWaterLevel = 0;

    if (sensorRepo.getSensorData(currentSensorData) == DATA_FETCH_SUCCESS && currentSensorData != nullptr)
    {
        currentWaterLevel = currentSensorData->waterLevel;
        free(currentSensorData);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to get current water level for manual pump start. Rejecting request.");
        return false;
    }

    // Check if pump is already on
    if (periphManager.getCurrentMotorState() == ON)
    {
        ESP_LOGI(TAG, "Pump is already ON. Manual start request ignored.");
        return true; // Not an error, pump is already running
    }

    // Check if water level allows manual start
    uint8_t maxAllowedLevel = (maxWaterLevelToStop > TOLERANCE_LEVEL) ? (maxWaterLevelToStop - TOLERANCE_LEVEL) : 0;

    if (currentWaterLevel < maxAllowedLevel)
    {
        // Manual start is allowed - override any lockout
        bool lockoutWasActive = stopManualLockoutTimer();
        setManualPumpRestartRequired(false);
        ESP_LOGI(TAG, "Manual pump start allowed (level: %d%% < %d%%). Lockout cleared: %s",
                 currentWaterLevel, maxAllowedLevel, lockoutWasActive ? "YES" : "NO");
        turnWaterPumpOn(currentTimestamp, reason, lockoutWasActive);
        return true;
    }
    else
    {
        ESP_LOGW(TAG, "Manual pump start rejected: water level (%d%%) >= max allowed level (%d%%). Stop threshold: %d%%",
                 currentWaterLevel, maxAllowedLevel, maxWaterLevelToStop);

        // Play error tone to indicate rejection
        buzzControl.playErrorTone();

        return false;
    }
}

void PumpController::processManualPumpToggle(uint8_t currentWaterLevel, uint8_t minWaterLevelForManualControl, uint64_t currentTimestamp)
{
    ESP_LOGI(TAG, "Processing manual pump toggle. Current Level: %d, Min for Manual: %d", currentWaterLevel, minWaterLevelForManualControl);
    if (periphManager.getCurrentMotorState() == ON)
    {
        // turnWaterPumpOff will handle lockout logic based on water level
        turnWaterPumpOff(currentTimestamp, PUMP_STOP_REASON_MANUAL);
        ESP_LOGI(TAG, "Pump manually turned OFF.");
    }
    else
    { // Pump is OFF - use the new centralized method
        attemptManualPumpStart(currentTimestamp, PUMP_START_REASON_MANUAL);
    }
}

bool PumpController::isPumpOn() const
{
    return periphManager.getCurrentMotorState() == ON;
}

void PumpController::setManualPumpRestartRequired(bool required)
{
    if (manualPumpRestartRequired == required)
    {
        return; // No change, do nothing
    }

    manualPumpRestartRequired = required;
    
    if (required)
    {
        // CRITICAL FIX: Don't change LED if calibration mode is active
        // Calibration mode LED indication takes precedence over lockout indication
        if (isRecalibrationModeActive)
        {
            ESP_LOGI(TAG, "=== LOCKOUT ENABLED (CALIBRATION MODE ACTIVE) ===");
            ESP_LOGI(TAG, "  Manual pump restart is now REQUIRED");
            ESP_LOGI(TAG, "  LED indication: Preserving CALIBRATION MODE (blue)");
            return; // Don't update LED, keep calibration mode indication
        }
        
        // Check if lockout feature is enabled to determine LED indication and messaging
        if (isLockoutFeatureEnabled())
        {
            // Lockout feature ENABLED - enforce lockout with timer and LED indication
            ESP_LOGI(TAG, "=== LOCKOUT ENABLED ===");
            ESP_LOGI(TAG, "  Manual pump restart is now REQUIRED");
            ESP_LOGI(TAG, "  Automatic pump restart is DISABLED");
            ESP_LOGI(TAG, "  User must manually restart pump or wait for lockout timer to expire");
            ledIndication.updatePumpStatus(PumpAppStatus::PUMP_MANUAL_RESTART_NEEDED);
        }
        else
        {
            // Lockout feature DISABLED - stall detected but no lockout enforcement
            ESP_LOGI(TAG, "=== STALL DETECTED - LOCKOUT FEATURE DISABLED ===");
            ESP_LOGI(TAG, "  Pump stopped due to stall detection (safety feature)");
            ESP_LOGI(TAG, "  Lockout feature is DISABLED - no restart delay enforced");
            ESP_LOGI(TAG, "  Pump can auto-restart immediately when water level is low");
            ESP_LOGI(TAG, "  LED indication: Normal operation (no lockout warning)");
            // Don't show lockout LED when feature is disabled - show normal state
            ledIndication.updatePumpStatus(PumpAppStatus::PUMP_AUTO_MODE_OK);
        }
    }
    else
    {
        // CRITICAL FIX: Don't change LED if calibration mode is active
        // Calibration mode LED indication takes precedence
        if (isRecalibrationModeActive)
        {
            ESP_LOGI(TAG, "=== LOCKOUT DISABLED (CALIBRATION MODE ACTIVE) ===");
            ESP_LOGI(TAG, "  Manual pump restart is no longer required");
            ESP_LOGI(TAG, "  LED indication: Preserving CALIBRATION MODE (blue)");
            return; // Don't update LED, keep calibration mode indication
        }
        
        ESP_LOGI(TAG, "=== LOCKOUT DISABLED ===");
        ESP_LOGI(TAG, "  Manual pump restart is no longer required");
        ESP_LOGI(TAG, "  Automatic pump restart is now ENABLED");
        ESP_LOGI(TAG, "  Pump can auto-start when water level is low");
        ledIndication.updatePumpStatus(PumpAppStatus::PUMP_AUTO_MODE_OK);
    }
}

void PumpController::updateMinWaterLevelToStartPump(uint8_t level)
{
    minWaterLevelToStart = level;
    ESP_LOGI(TAG, "Updated minWaterLevelToStart to %d", level);
    
    // SAFETY FIX: Check if sensor data is fresh before immediate evaluation
    // This prevents pump activation with stale/uninitialized data during system startup
    if (!sensorRepo.isDataFresh(5000)) {
        uint64_t dataAge = sensorRepo.getDataAgeMillis();
        ESP_LOGW(TAG, "  Sensor data is stale (age: %llu ms) - skipping immediate evaluation", dataAge);
        ESP_LOGW(TAG, "  Pump will be evaluated when fresh sensor data arrives");
        return;
    }
    
    // Immediately evaluate if pump should start with new threshold
    SensorData *currentData = nullptr;
    if (sensorRepo.getSensorData(currentData) == DATA_FETCH_SUCCESS && currentData != nullptr)
    {
        uint8_t currentWaterLevel = currentData->waterLevel;
        free(currentData);
        
        // Additional validation: reject obviously invalid readings
        if (currentWaterLevel > 100) {
            ESP_LOGW(TAG, "  Invalid water level reading: %d%% - skipping evaluation", currentWaterLevel);
            return;
        }
        
        ESP_LOGI(TAG, "  Current water level: %d%%, evaluating pump start condition", currentWaterLevel);
        uint64_t currentTimestamp = getCurrentTimestampMillis();
        handleWaterLevelChange(currentWaterLevel, currentWaterLevel, currentTimestamp);
    }
    else
    {
        ESP_LOGW(TAG, "  Could not get current sensor data for immediate evaluation");
    }
}

void PumpController::updateMaxWaterLevelToStopPump(uint8_t level)
{
    maxWaterLevelToStop = level;
    ESP_LOGI(TAG, "Updated maxWaterLevelToStop to %d", level);
    
    // SAFETY FIX: Check if sensor data is fresh before immediate evaluation
    // This prevents pump deactivation with stale/uninitialized data during system startup
    if (!sensorRepo.isDataFresh(5000)) {
        uint64_t dataAge = sensorRepo.getDataAgeMillis();
        ESP_LOGW(TAG, "  Sensor data is stale (age: %llu ms) - skipping immediate evaluation", dataAge);
        ESP_LOGW(TAG, "  Pump will be evaluated when fresh sensor data arrives");
        return;
    }
    
    // Immediately evaluate if pump should stop with new threshold
    SensorData *currentData = nullptr;
    if (sensorRepo.getSensorData(currentData) == DATA_FETCH_SUCCESS && currentData != nullptr)
    {
        uint8_t currentWaterLevel = currentData->waterLevel;
        free(currentData);
        
        // Additional validation: reject obviously invalid readings
        if (currentWaterLevel > 100) {
            ESP_LOGW(TAG, "  Invalid water level reading: %d%% - skipping evaluation", currentWaterLevel);
            return;
        }
        
        ESP_LOGI(TAG, "  Current water level: %d%%, evaluating pump stop condition", currentWaterLevel);
        uint64_t currentTimestamp = getCurrentTimestampMillis();
        handleWaterLevelChange(currentWaterLevel, currentWaterLevel, currentTimestamp);
    }
    else
    {
        ESP_LOGW(TAG, "  Could not get current sensor data for immediate evaluation");
    }
}

void PumpController::updateManualLockoutDuration(uint32_t duration_ms)
{
    manualLockoutDurationMillis = duration_ms;
    ESP_LOGI(TAG, "Updated manual lockout duration to %lu ms", duration_ms);
}

// Helper function implementations
void PumpController::startManualLockoutTimer(uint64_t currentTimestamp)
{
    ESP_LOGI(TAG, "=== STARTING LOCKOUT TIMER ===");
    
    // Get current water level for context
    SensorData *currentData = nullptr;
    uint8_t currentWaterLevel = 0;
    if (sensorRepo.getSensorData(currentData) == DATA_FETCH_SUCCESS && currentData != nullptr)
    {
        currentWaterLevel = currentData->waterLevel;
        free(currentData);
    }
    
    ESP_LOGI(TAG, "  Current Water Level: %d%%", currentWaterLevel);
    ESP_LOGI(TAG, "  Lockout Duration: %lu ms (%.1f minutes)", 
             manualLockoutDurationMillis, manualLockoutDurationMillis / 60000.0f);
    
    stopManualLockoutTimer(); // Ensure no old timer is running

    manualLockoutTimer = xTimerCreate(
        "manualLockoutTimer",
        pdMS_TO_TICKS(manualLockoutDurationMillis),
        pdFALSE, // One-shot timer
        this,
        manualLockoutTimerCallback);

    if (manualLockoutTimer != NULL)
    {
        if (xTimerStart(manualLockoutTimer, portMAX_DELAY) == pdPASS)
        {
            isLockoutTimerActive = true;

            // Get current timestamp and update NVS
            updateLockoutNVSStatus(true, currentTimestamp);

            // Format end time for logging
            time_t endTime = (currentTimestamp + manualLockoutDurationMillis) / 1000;
            struct tm endTimeInfo;
            localtime_r(&endTime, &endTimeInfo);
            char endTimeStr[32];
            strftime(endTimeStr, sizeof(endTimeStr), "%Y-%m-%d %H:%M:%S", &endTimeInfo);

            ESP_LOGI(TAG, "  Timer Started: SUCCESS");
            ESP_LOGI(TAG, "  Expires At: %s (timestamp: %llu)", 
                     endTimeStr, currentTimestamp + manualLockoutDurationMillis);
            ESP_LOGI(TAG, "  During lockout: Pump will NOT auto-start");
            ESP_LOGI(TAG, "  After lockout: Pump can auto-start if water level is low");
            ESP_LOGI(TAG, "=== LOCKOUT TIMER ACTIVE ===");
        }
        else
        {
            ESP_LOGE(TAG, "  Timer Started: FAILED");
            ESP_LOGE(TAG, "  Could not start lockout timer");
            ESP_LOGE(TAG, "=== LOCKOUT TIMER FAILED ===");
            xTimerDelete(manualLockoutTimer, portMAX_DELAY);
            manualLockoutTimer = NULL;
        }
    }
    else
    {
        ESP_LOGE(TAG, "  Timer Creation: FAILED");
        ESP_LOGE(TAG, "  Could not create lockout timer");
        ESP_LOGE(TAG, "=== LOCKOUT TIMER FAILED ===");
    }
}

bool PumpController::stopManualLockoutTimer()
{
    bool wasActive = isLockoutTimerActive;
    
    if (manualLockoutTimer != NULL)
    {
        if (xTimerIsTimerActive(manualLockoutTimer))
        {
            xTimerStop(manualLockoutTimer, portMAX_DELAY);
            
            if (wasActive)
            {
                ESP_LOGI(TAG, "=== STOPPING LOCKOUT TIMER ===");
                ESP_LOGI(TAG, "  Timer was active and has been stopped");
                ESP_LOGI(TAG, "  Reason: Manual cancellation or override");
                
                // Get current water level for context
                SensorData *currentData = nullptr;
                uint8_t currentWaterLevel = 0;
                if (sensorRepo.getSensorData(currentData) == DATA_FETCH_SUCCESS && currentData != nullptr)
                {
                    currentWaterLevel = currentData->waterLevel;
                    free(currentData);
                }
                ESP_LOGI(TAG, "  Current Water Level: %d%%", currentWaterLevel);
                ESP_LOGI(TAG, "=== LOCKOUT TIMER STOPPED ===");
            }
        }
        xTimerDelete(manualLockoutTimer, portMAX_DELAY);
        manualLockoutTimer = NULL;
    }
    
    if (isLockoutTimerActive)
    {
        isLockoutTimerActive = false;
        // Update NVS to reflect that lockout is no longer active
        updateLockoutNVSStatus(false);
    }
    
    return wasActive;
}

void PumpController::syncWithHardwareState()
{
    ESP_LOGI(TAG, "Syncing software state with hardware pump relay state...");

    // Check the actual hardware state of the pump relay
    PeripheralState hardwarePumpState = periphManager.getCurrentMotorState();

    if (hardwarePumpState == ON)
    {
        ESP_LOGW(TAG, "Hardware pump relay is ON after restart. Syncing software state...");

        // Get current timestamp for sync operation
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t currentTimestamp = (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));

        // Sync the pump ON state without controlling hardware
        syncPumpOnState(currentTimestamp, PUMP_START_REASON_WATER_LEVEL_REACHED);

        ESP_LOGI(TAG, "Pump state synchronized successfully. Monitoring systems activated.");
    }
    else
    {
        ESP_LOGI(TAG, "Hardware pump relay is OFF. No sync needed.");
    }
}

void PumpController::syncPumpOnState(uint64_t currentTimestamp, PumpStartReason reason)
{
    ESP_LOGI(TAG, "Syncing pump ON state without hardware control...");

    // Play motor on tune to indicate sync (similar to normal pump start)
    buzzControl.playMotorOnTune();

    // Set LED state to match pump state
    periphManager.setMotorToggleButtonLedState(ON);

    // Record pump start time (use current time as best estimate)
    pumpStartTimeMs = esp_timer_get_time() / 1000;

    // Get current water level for monitoring
    SensorData *data = nullptr;
    if (sensorRepo.getSensorData(data) == DATA_FETCH_SUCCESS && data != nullptr)
    {
        pumpStartWaterLevel = data->waterLevel;
        lastWaterLevelForStallCheck = data->waterLevel;
        free(data);
        ESP_LOGI(TAG, "Sync: Current water level is %d%%", pumpStartWaterLevel);
    }
    else
    {
        pumpStartWaterLevel = 0; // Default if data is unavailable
        lastWaterLevelForStallCheck = 0;
        ESP_LOGW(TAG, "Sync: Could not get current water level, using default values");
    }

    // Initialize stall detection variables - Added two-tiered stall detection state machine
    consecutiveLowIncrementChecks = 0;
    consecutiveNoWaterRiseChecks = 0;
    fast_check_start_level = 0.0f;

    if (pumpStartWaterLevel == 0)
    {
        monitoringState = GRACE_PERIOD;
        gracePeriodEndTimestamp = esp_timer_get_time() + (CONFIG_PUMP_STALL_GRACE_PERIOD_SEC * 1000000LL);
        ema_water_level = 0.0f;
        previous_ema_water_level = 0.0f;
        ESP_LOGI(TAG, "Sync: Pump starting in GRACE_PERIOD for %d seconds.", CONFIG_PUMP_STALL_GRACE_PERIOD_SEC);
    }
    else
    {
        monitoringState = MONITORING;
        ema_water_level = pumpStartWaterLevel;
        previous_ema_water_level = pumpStartWaterLevel;
        ESP_LOGI(TAG, "Sync: Pump starting in normal MONITORING mode.");
    }

    // Initialize Kalman filter state
    kf_x_hat = 15.0f; // A safe, default starting guess for SPPR
    kf_P = 100.0f;    // High initial uncertainty

    // Load Kalman filter state from NVS if available
    float saved_kf_x_hat, saved_kf_P;
    if (nvsManager.getKalmanStateXHat(saved_kf_x_hat) && nvsManager.getKalmanStateP(saved_kf_P))
    {
        // Use saved state if it's reasonable (positive values)
        if (saved_kf_x_hat > 0 && saved_kf_P > 0)
        {
            kf_x_hat = saved_kf_x_hat;
            kf_P = saved_kf_P;
            ESP_LOGI(TAG, "Loaded Kalman filter state from NVS: x_hat=%.2f, P=%.2f", kf_x_hat, kf_P);
        }
        else
        {
            ESP_LOGW(TAG, "Invalid Kalman filter state in NVS, using default initialization");
        }
    }
    else
    {
        ESP_LOGI(TAG, "No Kalman filter state found in NVS, using default initialization");
    }

    lastLevelChangeTimestamp = esp_timer_get_time();

    ESP_LOGI(TAG, "Sync: Initialized stall detection - level=%d, timestamp=%lld",
             lastWaterLevelForStallCheck, lastLevelChangeTimestamp);

    // Clean up any existing timer
    if (pumpMonitoringTimer != NULL)
    {
        xTimerDelete(pumpMonitoringTimer, portMAX_DELAY);
    }

    // Create and start pump monitoring timer for stall detection with correct interval
    pumpMonitoringTimer = xTimerCreate(
        "pumpStallTimer",
        pdMS_TO_TICKS(CONFIG_PUMP_IMMEDIATE_STALL_CHECK_INTERVAL_MS),
        pdTRUE, // Auto-reload timer
        this,   // Pass PumpController instance as Timer ID
        pumpMonitoringTimerCallback);

    if (pumpMonitoringTimer != NULL)
    {
        if (xTimerStart(pumpMonitoringTimer, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI(TAG, "Sync: Pump monitoring timer created and started successfully");
        }
        else
        {
            ESP_LOGE(TAG, "Sync: Failed to start pump monitoring timer");
            xTimerDelete(pumpMonitoringTimer, portMAX_DELAY);
            pumpMonitoringTimer = NULL;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Sync: Failed to create pump monitoring timer");
    }

    // Record the start reason for this sync operation
    lastStartReason = reason;

    ESP_LOGI(TAG, "Pump ON state sync completed. All monitoring systems are now active.");
}

void PumpController::updateLockoutNVSStatus(bool active, uint64_t startTime)
{
    if (!nvsManager.setLockoutActive(active))
    {
        ESP_LOGE(TAG, "Failed to update lockout active status in NVS");
    }

    if (active && startTime > 0)
    {
        if (!nvsManager.setLockoutStartedAtMillis(startTime))
        {
            ESP_LOGE(TAG, "Failed to update lockout start time in NVS");
        }
    }
}

LockoutInfo PumpController::getLockoutStatus()
{
    LockoutInfo info = {false, 0, 0, 0};

    // First check NVS for persistent state
    bool nvsActive = false;
    uint64_t nvsStartTime = 0;

    bool nvsActiveValid = nvsManager.getLockoutActive(nvsActive);
    bool nvsStartTimeValid = nvsManager.getLockoutStartedAtMillis(nvsStartTime);

    // Sync runtime state with NVS if there's a discrepancy
    if (nvsActiveValid && (nvsActive != isLockoutTimerActive))
    {
        ESP_LOGW(TAG, "Lockout state mismatch: NVS=%s, Runtime=%s",
                 nvsActive ? "active" : "inactive",
                 isLockoutTimerActive ? "active" : "inactive");
    }

    // Use runtime state as the authoritative source
    info.isActive = isLockoutTimerActive;

    if (info.isActive && nvsStartTimeValid && nvsStartTime > 0)
    {
        info.startedAtMillis = nvsStartTime;

        // Get current timestamp
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t currentTimestamp = (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
        ESP_LOGI(TAG, "nvsStartTime: %llu, currentTimestamp: %llu", nvsStartTime, currentTimestamp);

        // Calculate elapsed time
        if (currentTimestamp >= info.startedAtMillis)
        {
            info.elapsedMillis = currentTimestamp - info.startedAtMillis;
        }
        else
        {
            info.elapsedMillis = 0; // Handle clock adjustment edge case
        }

        // Calculate end time
        info.endsAtMillis = info.startedAtMillis + manualLockoutDurationMillis;
        ESP_LOGI(TAG, "elapsedMillis: %llu, endsAtMillis: %llu", info.elapsedMillis, info.endsAtMillis);
    }

    return info;
}

uint64_t PumpController::getCurrentTimestampMillis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

void PumpController::restoreManualLockoutTimer(uint32_t remainingDurationMs)
{
    // Don't create timer if remaining time is too small (less than 30 seconds)
    if (remainingDurationMs < 30000)
    {
        ESP_LOGW(TAG, "Remaining lockout duration too small (%lu ms), clearing lockout state", remainingDurationMs);
        updateLockoutNVSStatus(false);
        return;
    }

    stopManualLockoutTimer(); // Ensure no old timer is running

    manualLockoutTimer = xTimerCreate(
        "manualLockoutTimer",
        pdMS_TO_TICKS(remainingDurationMs),
        pdFALSE, // One-shot timer
        this,
        manualLockoutTimerCallback);

    if (manualLockoutTimer != NULL)
    {
        if (xTimerStart(manualLockoutTimer, portMAX_DELAY) == pdPASS)
        {
            isLockoutTimerActive = true;
            setManualPumpRestartRequired(true);
            ESP_LOGI(TAG, "Restored manual lockout timer with %lu ms remaining", remainingDurationMs);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to start restored manualLockoutTimer");
            xTimerDelete(manualLockoutTimer, portMAX_DELAY);
            manualLockoutTimer = NULL;
            updateLockoutNVSStatus(false); // Clear NVS if timer creation failed
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to create restored manualLockoutTimer");
        updateLockoutNVSStatus(false); // Clear NVS if timer creation failed
    }
}

void PumpController::restoreLockoutStateFromNVS()
{
    ESP_LOGI(TAG, "Checking for lockout state to restore from NVS...");

    bool wasActive = false;
    uint64_t startTime = 0;

    // Step 1: Check if lockout was active before reboot
    if (!nvsManager.getLockoutActive(wasActive))
    {
        ESP_LOGD(TAG, "Failed to read lockout active status from NVS or no lockout was active");
        return;
    }

    if (!wasActive)
    {
        ESP_LOGD(TAG, "No active lockout found in NVS");
        return;
    }

    // Step 2: Get the lockout start timestamp
    if (!nvsManager.getLockoutStartedAtMillis(startTime) || startTime == 0)
    {
        ESP_LOGW(TAG, "Invalid lockout start time in NVS, clearing lockout state");
        updateLockoutNVSStatus(false);
        return;
    }

    // Step 3: Calculate elapsed time since lockout started
    uint64_t currentTime = getCurrentTimestampMillis();

    // Handle clock adjustment edge case
    if (currentTime < startTime)
    {
        ESP_LOGW(TAG, "Current time is before lockout start time (clock adjustment?), clearing lockout state");
        updateLockoutNVSStatus(false);
        return;
    }

    uint64_t elapsedMs = currentTime - startTime;

    // Step 4: Check if lockout should still be active
    if (elapsedMs >= manualLockoutDurationMillis)
    {
        ESP_LOGI(TAG, "Lockout has expired during reboot (elapsed: %llu ms, duration: %lu ms), clearing state",
                 elapsedMs, manualLockoutDurationMillis);
        updateLockoutNVSStatus(false);
        return;
    }

    // Step 5: Restore the lockout timer with remaining duration
    uint32_t remainingMs = (uint32_t)(manualLockoutDurationMillis - elapsedMs);
    ESP_LOGI(TAG, "Restoring lockout state: elapsed=%llu ms, remaining=%lu ms", elapsedMs, remainingMs);

    restoreManualLockoutTimer(remainingMs);
}

void PumpController::setRecalibrationMode(bool enable)
{
    isRecalibrationModeActive = enable;
    
    // Get current water level and SPPR for logging
    SensorData *currentData = nullptr;
    uint8_t currentWaterLevel = 0;
    if (sensorRepo.getSensorData(currentData) == DATA_FETCH_SUCCESS && currentData != nullptr)
    {
        currentWaterLevel = currentData->waterLevel;
        free(currentData);
    }
    
    if (enable)
    {
        ESP_LOGI(TAG, "=== CALIBRATION MODE ENABLED ===");
        ESP_LOGI(TAG, "  Current Water Level: %d%%", currentWaterLevel);
        ESP_LOGI(TAG, "  Current SPPR Estimate: %.2fs/%%", kf_x_hat);
        ESP_LOGI(TAG, "  Current Uncertainty: %.2f", kf_P);
        ESP_LOGI(TAG, "  Next pump cycle will recalibrate SPPR directly from measurement");
        ESP_LOGI(TAG, "  Kalman filter will be reset with new baseline");
        
        // When enabling recalibration, stop any active manual lockout.
        bool lockoutWasActive = stopManualLockoutTimer();
        
        // If lockout was active, publish LOCKOUT_DEACTIVATED event
        if (lockoutWasActive && publishEventCallback)
        {
            uint64_t currentTimestamp = getCurrentTimestampMillis();
            cJSON *eventData = cJSON_CreateObject();
            if (eventData == NULL)
            {
                ESP_LOGE(TAG, "Failed to create JSON object for calibration mode event - out of memory");
            }
            else
            {
                cJSON_AddNumberToObject(eventData, "timestamp", (double)currentTimestamp);
                cJSON_AddStringToObject(eventData, "reason", "calibration_mode_enabled");
                publishEventCallback(EVENT_TYPE_LOCKOUT_DEACTIVATED, EVENT_LEVEL_INFO, eventData);
                cJSON_Delete(eventData);
                ESP_LOGI(TAG, "  Lockout was active - LOCKOUT_DEACTIVATED event published");
            }
        }
        
        // Set LED to indicate recalibration mode (e.g., solid blue).
        ledIndication.updatePumpStatus(PumpAppStatus::PUMP_RECALIBRATION_MODE);
    }
    else
    {
        ESP_LOGI(TAG, "=== CALIBRATION MODE DISABLED ===");
        ESP_LOGI(TAG, "  Final Water Level: %d%%", currentWaterLevel);
        ESP_LOGI(TAG, "  Final SPPR Estimate: %.2fs/%%", kf_x_hat);
        ESP_LOGI(TAG, "  Final Uncertainty: %.2f", kf_P);
        ESP_LOGI(TAG, "  Returning to normal Kalman filter operation");
        
        // CRITICAL FIX: Removed automatic pump stop logic to prevent reentrancy bug
        // When turnWaterPumpOff() calls setRecalibrationMode(false), the pump is still
        // in the process of being stopped. Calling turnWaterPumpOff() again here causes:
        // - Double buzzer tune playback
        // - Duplicate pump cycle summaries
        // - Incorrect SPPR calculations
        // The caller (turnWaterPumpOff, manual commands, etc.) is responsible for stopping the pump.
        
        // When disabling, restore default LED status.
        ledIndication.updatePumpStatus(PumpAppStatus::PUMP_AUTO_MODE_OK);
    }
}

bool PumpController::isRecalibrationMode() const
{
    return isRecalibrationModeActive;
}

// Method to enable or disable the lockout feature
esp_err_t PumpController::setLockoutFeatureEnabled(bool enabled)
{
    ESP_LOGI(TAG, "=== SET LOCKOUT FEATURE %s ===", enabled ? "ENABLED" : "DISABLED");
    
    // Validation: Check if pump is currently running
    if (periphManager.getCurrentMotorState() == ON)
    {
        ESP_LOGW(TAG, "Cannot change lockout feature state while pump is running");
        ESP_LOGW(TAG, "Rejection reason: Pump state = ON");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Validation: Check water level
    SensorData *currentData = nullptr;
    uint8_t currentWaterLevel = 0;
    if (sensorRepo.getSensorData(currentData) == DATA_FETCH_SUCCESS && currentData != nullptr)
    {
        currentWaterLevel = currentData->waterLevel;
        free(currentData);
        
        // Only check water level when disabling lockout feature
        if (!enabled && currentWaterLevel < minWaterLevelToStart)
        {
            ESP_LOGW(TAG, "Cannot disable lockout feature with low water level");
            ESP_LOGW(TAG, "Current water level: %d%%, Minimum safe level: %d%%", 
                     currentWaterLevel, minWaterLevelToStart);
            return ESP_ERR_INVALID_STATE;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to get current water level for validation");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Take mutex to write shared state atomically
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Check if state is already at the desired value (inside mutex)
        if (_lockoutFeatureEnabled == enabled)
        {
            xSemaphoreGive(_stateMutex);
            ESP_LOGI(TAG, "Lockout feature already %s, no change needed", 
                     enabled ? "ENABLED" : "DISABLED");
            return ESP_OK;
        }
        
        // Update in-memory state first
        _lockoutFeatureEnabled = enabled;
        
        xSemaphoreGive(_stateMutex);
        
        // Persist to NVS outside mutex (NVS has its own synchronization)
        esp_err_t ret = nvsManager.writeBool(NVS_KEY_LOCKOUT_FEATURE_ENABLED, enabled);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to persist lockout feature state to NVS: %s", 
                     esp_err_to_name(ret));
            
            // Rollback in-memory state on NVS failure
            if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                _lockoutFeatureEnabled = !enabled;
                xSemaphoreGive(_stateMutex);
            }
            
            return ret;
        }
        
        ESP_LOGI(TAG, "Lockout feature %s successfully", enabled ? "ENABLED" : "DISABLED");
        ESP_LOGI(TAG, "State persisted to NVS");
        ESP_LOGI(TAG, "Current water level: %d%%", currentWaterLevel);
        
        // Note: Active lockout is NOT cleared when disabling the feature
        if (!enabled && manualPumpRestartRequired)
        {
            ESP_LOGI(TAG, "Active lockout preserved (not cleared by feature disable)");
        }
        
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Mutex timeout in setLockoutFeatureEnabled");
        return ESP_ERR_TIMEOUT;
    }
}

// Method to check if lockout feature is enabled
bool PumpController::isLockoutFeatureEnabled() const
{
    return _lockoutFeatureEnabled;
}

// Method to get complete lockout configuration
LockoutConfiguration PumpController::getLockoutConfiguration() const
{
    LockoutConfiguration config;
    
    // Take mutex to read shared state atomically
    // Use 100ms timeout to prevent deadlocks
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Copy shared state while holding mutex
        config.lockoutFeatureEnabled = _lockoutFeatureEnabled;
        config.lockoutDurationMs = manualLockoutDurationMillis;
        config.lockoutCurrentlyActive = isLockoutTimerActive;
        
        xSemaphoreGive(_stateMutex);
    } else {
        ESP_LOGW(TAG, "Mutex timeout in getLockoutConfiguration - returning safe defaults");
        // Return fail-safe defaults on timeout
        config.lockoutFeatureEnabled = true;  // Fail-safe: lockout enabled
        config.lockoutDurationMs = 1800000;   // 30 minutes default
        config.lockoutCurrentlyActive = false;
    }
    
    // NVS and time operations outside mutex (they have their own synchronization)
    uint64_t startTime = 0;
    if (nvsManager.getLockoutStartedAtMillis(startTime) && startTime > 0)
    {
        config.lockoutTriggerTime = (uint32_t)(startTime / 1000);  // Convert ms to seconds
    }
    else
    {
        config.lockoutTriggerTime = 0;
    }
    
    // Lockout clear time is not currently tracked, set to 0
    config.lockoutClearTime = 0;
    
    // Get current timestamp
    struct timeval tv;
    gettimeofday(&tv, NULL);
    config.timestamp = (uint32_t)tv.tv_sec;
    
    return config;
}
