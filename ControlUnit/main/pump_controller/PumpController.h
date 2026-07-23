#ifndef PUMPCONTROLLER_H_
#define PUMPCONTROLLER_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "../app_events.h"
#include "freertos/event_groups.h"
#include "peripherals/PeripheralManager.h"
#include "peripherals/BuzzerControl.h"
#include "switching_unit/SwitchingUnitManager.h"
#include "peripherals/SensorDataRepo.h" // Added for SensorDataRepo&
#include "ledIndication/LedIndication.h"
#include "data/dataTypes.h" // For DeviceEventType
#include <functional>
#include "storage/nvsManager.h"
#include "cJSON.h"
#include "sdkconfig.h"

#ifndef CONFIG_PUMP_PRIMING_SAFETY_MULTIPLIER
#define CONFIG_PUMP_PRIMING_SAFETY_MULTIPLIER 15
#endif

#ifndef CONFIG_PUMP_MAX_PROTECTED_WINDOW_SEC
#define CONFIG_PUMP_MAX_PROTECTED_WINDOW_SEC 240
#endif

// Forward declaration if MQTTManager is only used for a callback type
// class MQTT5Manager; // Or include MQTTManager.h if methods are called

#define PUMP_MONITORING_TICK_PERIOD_MILLIS CONFIG_PUMP_MONITORING_TICK_PERIOD_MS
#define TOLERANCE_LEVEL 10

// Struct to hold lockout status information
struct LockoutInfo
{
    bool isActive;
    uint64_t startedAtMillis;
    uint64_t elapsedMillis;
    uint64_t endsAtMillis;
};

// Struct to hold complete lockout configuration
struct LockoutConfiguration
{
    bool lockoutFeatureEnabled;      // Is lockout feature enabled?
    uint32_t lockoutDurationMs;      // Configured lockout duration in milliseconds
    bool lockoutCurrentlyActive;     // Is pump currently in lockout state?
    uint32_t lockoutTriggerTime;     // Unix timestamp when lockout was triggered (0 if not active)
    uint32_t lockoutClearTime;       // Unix timestamp when lockout was cleared (0 if not cleared)
    uint32_t timestamp;              // Current device time (Unix timestamp)
};

class PumpController
{
public:
    // Define a type for the event publishing callback
    using EventPublisher = std::function<void(DeviceEventType eventType, DeviceEventVerbosity eventVerbosity, const cJSON *data)>;

    PumpController(PeripheralManager &peripheralManager,
                   SwitchingUnitManager &switchingUnitManager,
                   BuzzerControl &buzzerControl,
                   SensorDataRepo &sensorDataRepo,
                   LedIndication &ledIndication,
                   EventPublisher eventPublisher,
                   EventGroupHandle_t event_group,
                   NVSManager &nvsManager,
                   uint8_t minWaterLevelToStartPump,
                   uint8_t maxWaterLevelToStopPump);
    ~PumpController();

    void turnWaterPumpOn(uint64_t currentTimestamp, PumpStartReason reason, bool lockoutWasCleared = false);
    void turnWaterPumpOff(uint64_t currentTimestamp, PumpStopReason reason);
    void handleWaterLevelChange(uint8_t newWaterLevel, uint8_t previousWaterLevel, uint64_t currentTimestamp);
    void processManualPumpToggle(uint8_t currentWaterLevel, uint8_t minWaterLevelForManualControl, uint64_t currentTimestamp); // New method for button logic
    bool attemptManualPumpStart(uint64_t currentTimestamp, PumpStartReason reason);                                            // New method for manual starts with water level check
    bool isPumpOn() const;
    void setManualPumpRestartRequired(bool required);
    void syncWithHardwareState(); // Sync software state with hardware state on startup

    // NVS dependent values that might change
    void updateMinWaterLevelToStartPump(uint8_t level);
    void updateMaxWaterLevelToStopPump(uint8_t level);
    void updateManualLockoutDuration(uint32_t duration_ms);

    // Lockout status methods
    LockoutInfo getLockoutStatus();
    
    /**
     * @brief Enable or disable the lockout feature (stall detection protection)
     * 
     * This method controls whether the stall detection system is active. When disabled,
     * the pump can run without triggering lockouts even if water level doesn't increase.
     * 
     * SAFETY VALIDATIONS:
     * - Command REJECTED if pump is currently running
     * - Command REJECTED if water level is below MIN_SAFE_LEVEL (when disabling)
     * - Disabling does NOT clear existing active lockouts (safety preservation)
     * 
     * PERSISTENCE:
     * - State is persisted to NVS (key: "lockout_en", max 15 chars per NVS spec)
     * - Default value: true (fail-safe: lockout enabled)
     * - State survives reboots and power cycles
     * 
     * FEEDBACK:
     * - When disabled: Amber pulsing LED + warning buzzer tone
     * - When enabled: Normal LED indication + confirmation buzzer tone
     * - All state changes logged with timestamps and trigger source
     * 
     * @param enabled true to enable lockout feature, false to disable
     * @return ESP_OK on success
     *         ESP_ERR_INVALID_STATE if validation fails (pump running or low water)
     *         ESP_ERR_* on NVS write failure
     * 
     * @note This method is called from MQTT command handler
     * @note All other safety interlocks remain enforced regardless of this setting
     * @warning Disabling lockout removes stall detection protection - use with caution
     */
    esp_err_t setLockoutFeatureEnabled(bool enabled);
    
    /**
     * @brief Check if lockout feature is currently enabled
     * 
     * @return true if lockout feature is enabled (stall detection active)
     *         false if lockout feature is disabled (stall detection bypassed)
     * 
     * @note This is checked in canStartPump() and checkForStallCondition()
     */
    bool isLockoutFeatureEnabled() const;
    
    /**
     * @brief Get complete lockout configuration and status
     * 
     * Returns a comprehensive snapshot of the lockout system state including:
     * - Whether lockout feature is enabled (stall detection active)
     * - Configured lockout duration in milliseconds
     * - Whether pump is currently in lockout state
     * - Timestamps for lockout trigger and clear events
     * - Current device time
     * 
     * @return LockoutConfiguration struct with all lockout parameters
     * 
     * @note This method is called from MQTT configuration query handler
     * @note Timestamps are Unix timestamps (seconds since epoch)
     * @note lockoutTriggerTime is 0 if no lockout is active
     */
    LockoutConfiguration getLockoutConfiguration() const;

    void setRecalibrationMode(bool enable);
    bool isRecalibrationMode() const;

private:
    enum PumpMonitoringState
    {
        IDLE,
        GRACE_PERIOD,
        MONITORING,
        STALL_SUSPECTED
    };

    PeripheralManager &periphManager;
    SwitchingUnitManager &switchingUnit;
    BuzzerControl &buzzControl;
    SensorDataRepo &sensorRepo; // Added SensorDataRepo reference
    LedIndication &ledIndication;
    EventPublisher publishEventCallback;
    EventGroupHandle_t app_event_group;
    NVSManager &nvsManager;

    TimerHandle_t pumpMonitoringTimer;
    uint8_t consecutiveLowIncrementChecks;

    bool manualPumpRestartRequired;
    TimerHandle_t manualLockoutTimer; // For timed auto-reset
    bool isLockoutTimerActive;        // To quickly check timer status
    bool isRecalibrationModeActive;
    uint32_t manualLockoutDurationMillis; // Configurable lockout time
    
    // Lockout feature control
    bool _lockoutFeatureEnabled;  // In-memory state
    static const char* NVS_KEY_LOCKOUT_FEATURE_ENABLED;  // NVS key: "lockout_en" (max 15 chars)
    
    /**
     * @brief Mutex to protect lockout configuration state
     * 
     * Protects concurrent access to:
     * - _lockoutFeatureEnabled
     * - manualLockoutDurationMillis
     * - isLockoutTimerActive
     * 
     * Thread-safe access required because these variables are accessed from:
     * - MQTT request handler task
     * - Button press handler task
     * - Pump monitoring task
     * - Timer callbacks
     */
    SemaphoreHandle_t _stateMutex;

    uint8_t minWaterLevelToStart; // MIN_WATER_LEVEL_TO_START_PUMP
    uint8_t maxWaterLevelToStop;  // MAX_WATER_LEVEL_TO_STOP_PUMP
    // Kalman Filter State Variables
    float kf_x_hat; // Estimated state (SPPR)
    float kf_P;     // Estimated uncertainty (variance)

    // Kalman Filter Parameters (load from NVS)
    float kf_Q; // Process noise variance
    float kf_R; // Measurement noise variance
    float kf_k; // Stall detection multiplier

    uint8_t lastWaterLevelForStallCheck;
    int64_t lastLevelChangeTimestamp;
    int64_t pumpStartTimeMs;
    uint8_t pumpStartWaterLevel;
    PumpStopReason lastStopReason;
    PumpStartReason lastStartReason;

    // For the new stall detection logic
    PumpMonitoringState monitoringState;
    uint8_t stall_confirmation_count;
    int64_t stall_check_start_timestamp;
    int64_t gracePeriodEndTimestamp;
    uint8_t initialWaterLevelForStallCheck;
    uint8_t stallCheckReadings[3];
    uint64_t stallCheckSequenceNumbers[3];
    uint64_t initialStallCheckSeqNum;
    uint8_t stallCheckReadingsCount;
    int64_t firstRiseTimestamp;
    bool primingTimeRecorded;

    // EMA smoothing variables for water level
    float ema_water_level;
    float previous_ema_water_level;
    float fast_check_start_level;
    uint8_t consecutiveNoWaterRiseChecks;

    // EMA smoothing factor (alpha)
    static constexpr float EMA_ALPHA = 0.3f;

    // Static callback declarations
    static void pumpMonitoringTimerCallback(TimerHandle_t xTimer);
    static void manualLockoutTimerCallback(TimerHandle_t xTimer); // Callback for the lockout timer

    // Worker task for deferred timer callback processing (FreeRTOS best practice)
    static void pumpWorkerTask(void *pvParameters);
    TaskHandle_t pumpWorkerTaskHandle;
    
    // Task notification bits for worker task
    static constexpr uint32_t NOTIFY_STALL_CHECK = (1 << 0);
    static constexpr uint32_t NOTIFY_LOCKOUT_EXPIRED = (1 << 1);
    
    // Race condition prevention for pump start operations
    // Prevents double buzzer tune when multiple tasks try to start pump simultaneously
    static volatile bool _pumpStartInProgress;
    static portMUX_TYPE _pumpStartSpinlock;

    // New private method declarations
    void processStallDetection();
    void handleLockoutExpired();  // Deferred lockout expiry handling
    void startManualLockoutTimer(uint64_t currentTimestamp);                 // Helper to start the lockout
    bool stopManualLockoutTimer();                                           // Helper to stop the lockout - returns true if lockout was active
    void syncPumpOnState(uint64_t currentTimestamp, PumpStartReason reason); // Sync pump ON state without hardware control
    void updateLockoutNVSStatus(bool active, uint64_t startTime = 0);        // Helper to update NVS lockout status
    void restoreLockoutStateFromNVS();                                       // Restore lockout state after reboot
    void restoreManualLockoutTimer(uint32_t remainingDurationMs);            // Restore timer with remaining duration
    uint64_t getCurrentTimestampMillis();                                    // Helper to get current timestamp consistently
};

#endif // PUMPCONTROLLER_H_
