#ifndef OTAMANAGER_H_
#define OTAMANAGER_H_

#include "../https/HttpsClient.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include "../secrets.h"

#define DEFAULT_UPDATE_CHECK_INTERVAL_MILLIS (CONFIG_OTA_UPDATE_CHECK_INTERVAL_HOURS * 3600000) // Convert hours to milliseconds
#define MINIMUM_UPDATE_CHECK_INTERVAL_MILLIS 20000                                              // 20 seconds - static definition
#define MAXIMUM_UPDATE_CHECK_INTERVAL_MILLIS 86400000                                           // 24 hours - static definition
#define CONFIG_OTA_RECV_TIMEOUT CONFIG_OTA_RECV_TIMEOUT_MS
#define HASH_LEN 32

// HTTP client buffer and keepalive configuration
#define OTA_HTTP_BUFFER_SIZE CONFIG_OTA_HTTP_BUFFER_SIZE
#define OTA_KEEPALIVE_IDLE CONFIG_OTA_KEEPALIVE_IDLE_SEC
#define OTA_KEEPALIVE_INTERVAL CONFIG_OTA_KEEPALIVE_INTERVAL_SEC
#define OTA_KEEPALIVE_COUNT CONFIG_OTA_KEEPALIVE_COUNT

#define VERSION_NAME_REGEX "^([1-9]{1,2}\\.[0-9]{1,2}\\.[0-9]{1,2})$"

// Increased to 8192 words (~32 KB) to avoid stack overflow during TLS / HTTP OTA operations
#define OTA_UPDATE_CHECK_TASK_STACK_DEPTH CONFIG_OTA_TASK_STACK_DEPTH
#define OTA_UPDATE_CHECK_TASK_PRIORITY CONFIG_OTA_TASK_PRIORITY
#define OTA_WRITE_DATA_BUFFER_LENGTH CONFIG_OTA_WRITE_DATA_BUFFER_LENGTH
// #define VERSION_CHECK_API_URL "https://34.105.37.184:3001/versionCheck"
// #define VERSION_CHECK_API_URL "http://192.168.0.171:5001/water-level-analyzer/us-central1/ota/firmwareData/latest"
// #define FIRMWARE_OTA_UPDATE_URL "http://192.168.0.171:5001/water-level-analyzer/us-central1/ota/firmware.bin"

#if USE_DEVELOPMENT_SERVER == 1
#define VERSION_CHECK_API_URL CONFIG_OTA_VERSION_CHECK_URL_DEV
#define FIRMWARE_OTA_UPDATE_URL CONFIG_OTA_FIRMWARE_DOWNLOAD_URL_DEV
#else
#define VERSION_CHECK_API_URL CONFIG_OTA_VERSION_CHECK_URL
#define FIRMWARE_OTA_UPDATE_URL CONFIG_OTA_FIRMWARE_DOWNLOAD_URL
#endif

extern const char server_pem_start[] asm("_binary_firebase_function_pem_start");
extern const char server_pem_end[] asm("_binary_firebase_function_pem_end");

using namespace std;

class OTAManager
{
private:
    uint64_t updateCheckIntervalMillis;
    TimerHandle_t timerHandle;
    TaskHandle_t otaTaskHandle;
    EventGroupHandle_t otaEventGroup;
    HttpsClient *httpsClient;
    bool isUpdateCheckRunning = false;
    bool otaUpdateRunning = false;
    char *otaWriteDataBuffer = NULL;
    char *deviceID = NULL;  // Device ID for OTA endpoint authentication

    function<bool()> onNewUpdateAvailableListener = NULL;
    function<void(const char* currentVersion, const char* newVersion)> onOTAUpdateStartedListener = NULL;
    function<void(const char* errorReason)> onOTAUpdateAbortedListener = NULL;
    function<void(uint8_t)> onOTAUpdateProgressListener = NULL;
    function<void()> onOTAUpdateCompletedListener = NULL;
    
    // Callbacks for stopping/starting MQTT during OTA to avoid concurrent TLS issues
    function<void()> onBeforeHttpsRequestListener = NULL;  // Called before HTTPS request (stop MQTT)
    function<void()> onAfterHttpsRequestListener = NULL;   // Called after HTTPS request (restart MQTT)

    // Store new version name for event publishing
    char* newVersionName = NULL;

    uint32_t getVersionNumberFromString(char *versionStringPtr);
    friend void otaTimerCallback(TimerHandle_t xTimer);
    friend void otaTask(void *params);
    void abortOTA(const char* errorReason = NULL);

public:
    bool getCurrentFirmwareVersion(char *&versionStringHolder);
    bool getCurrentAppDescription(cJSON *&jsonHolder);
    uint32_t getCurrentFirmwareVersionNumber();

    void startPeriodicUpdatesCheck();
    void stopPeriodicUpdatesCheck();
    bool isOTAUpdateRunning();
    void triggerUpdateCheck();
    
    // Set device ID for OTA endpoint authentication
    void setDeviceID(const char *deviceID);

    void setOnNewUpdateAvailableListener(function<bool()> onNewUpdateAvailableListener);
    void setOnOTAUpdateStartedListener(function<void(const char* currentVersion, const char* newVersion)> onOTAUpdateStartedListener);
    void setOnOTAUpdateAbortedListener(function<void(const char* errorReason)> onOTAUpdateAbortedListener);
    void setOnOTAUpdateProgressListener(function<void(uint8_t)> onOTAUpdateProgressListener);
    void setOnOTAUpdateCompletedListener(function<void()> onOTAUpdateCompletedListener);
    
    // Set callbacks for stopping/starting MQTT during OTA to avoid concurrent TLS heap corruption
    void setOnBeforeHttpsRequestListener(function<void()> listener);
    void setOnAfterHttpsRequestListener(function<void()> listener);

    OTAManager(HttpsClient *httpsClient);
    OTAManager(HttpsClient *httpsClient, uint64_t updateCheckIntervalMillis);
    ~OTAManager();
};

#endif