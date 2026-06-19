#ifndef WIFIMANAGER_H_
#define WIFIMANAGER_H_

#include <functional>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "esp_idf_version.h"

// Compatibility macros for ESP-IDF v6.0 bandwidth names
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#ifndef WIFI_BW_HT20
#define WIFI_BW_HT20 WIFI_BW20
#endif
#ifndef WIFI_BW_HT40
#define WIFI_BW_HT40 WIFI_BW40
#endif
#endif
#include "../secrets.h"
#include "../storage/nvsManager.h"

// Wifi Application Settings
#define WIFI_AP_SSID WIFI_AP_SSID_SECRET                       // Access Point name visible to other devices.
#define WIFI_AP_PASSWORD WIFI_AP_PASSWORD_SECRET               // Access Point password
#define WIFI_AP_CHANNEL CONFIG_WIFI_AP_CHANNEL                 // Access Point Channel
#define WIFI_AP_SSID_HIDDEN CONFIG_WIFI_AP_SSID_HIDDEN         // Whether to make the AP visible to nearby devices 0 - visible 1- invisible
#define WIFI_AP_MAX_CONNECTIONS CONFIG_WIFI_AP_MAX_CONNECTIONS // Max number of devices that can connect to this Access Point
// Increase the following value from 100 to max 1000 to allow wifi scanning without disconnecting other connected stations
#define WIFI_AP_BEACON_INTERVAL CONFIG_WIFI_AP_BEACON_INTERVAL               // Access Point beacon interval in milliseconds : 100 recommended
#define WIFI_AP_IP CONFIG_WIFI_AP_IP                                         // AP Default IP Address
#define WIFI_AP_GATEWAY CONFIG_WIFI_AP_GATEWAY                               // Default Gateway (should be the same as the IP Address)
#define WIFI_AP_NETMASK CONFIG_WIFI_AP_NETMASK                               // AP NETMASK
#define WIFI_AP_BANDWIDTH WIFI_BW_HT40                                       // AP BandWidth 20 Mhz (40 MHz is the other option)
#define WIFI_STA_POWER_SAVE WIFI_PS_NONE                                     // Power save not used
#define WIFI_AP_CONNECT_RETRY_INTERVAL CONFIG_WIFI_AP_CONNECT_RETRY_INTERVAL // Retry interval in milliseconds

#define MAX_SSID_LENGTH CONFIG_MAX_SSID_LENGTH         // IEEE Standard maximum
#define MAX_PASSWORD_LENGTH CONFIG_MAX_PASSWORD_LENGTH // IEEE Standard maximum

#define WIFI_MAX_CONNECTION_RETRIES CONFIG_WIFI_MAX_CONNECTION_RETRIES // Retry number on disconnect

#define SNTP_TIME_UPDATE_INTERVAL_SEC CONFIG_SNTP_TIME_UPDATE_INTERVAL_SEC
#define SNTP_TIME_ZONE_INDIA CONFIG_SNTP_TIME_ZONE
#define SNTP_SERVER_NAME_0 CONFIG_SNTP_SERVER_NAME_0
#define SNTP_SERVER_NAME_1 CONFIG_SNTP_SERVER_NAME_1
// #define SNTP_SERVER_NAME_2 "pool.ntp.org"
// #define SNTP_SERVER_NAME_3 "pool.ntp.org"
#define SNTP_TIME_SYNC_TASK_STACK_DEPTH CONFIG_SNTP_TIME_SYNC_TASK_STACK_DEPTH
#define SNTP_TIME_SYNC_TASK_PRIORITY CONFIG_SNTP_TIME_SYNC_TASK_PRIORITY
#define SNTP_TIME_SYNC_TASK_CORE PRO_CPU_NUM // Core on which the SNTP task will run

// Config mode idle timeout - auto-restart if no station connects
#define CONFIG_MODE_IDLE_TIMEOUT (5 * 60000) // 5 minutes in milliseconds

// ----------------------------------------------------------

#define WIFI_ACESS_POINT_CONNECTED_BIT BIT0
#define WIFI_ACESS_POINT_DISCONNECTED_BIT BIT1
#define WIFI_STATION_CONNECTED_BIT BIT2
#define WIFI_STATION_DISCONNECTED_BIT BIT3

#define WIFI_ACESS_POINT_CONNECT_FAILED_BIT BIT6
#define WIFI_ACESS_POINT_DISCONNECT_FAILED_BIT BIT7
#define WIFI_STATION_CONNECT_FAILED_BIT BIT8
#define WIFI_STATION_DISCONNECT_FAILED_BIT BIT9

#define WIFI_RECONNECT_BIT BIT14

// ----------------------------------------------------------

#define WIFI_AS_STATION_STARTED_BIT BIT4
#define WIFI_AS_ACCESS_POINT_STARTED_BIT BIT5
#define WIFI_AS_STATION_CLOSED_BIT BIT4
#define WIFI_AS_ACCESS_POINT_CLOSED_BIT BIT5

#define WIFI_AS_STATION_START_FAILED_BIT BIT10
#define WIFI_AS_ACCESS_POINT_START_FAILED_BIT BIT11
#define WIFI_AS_STATION_CLOSE_FAILED_BIT BIT12
#define WIFI_AS_ACCESS_POINT_CLOSE_FAILED_BIT BIT13

// ----------------------------------------------------------

#define SNTP_TASK_STARTED_BIT BIT17

// ----------------------------------------------------------

#define MESSAGE_TABLE_IT(reason) \
    {                            \
        reason, #reason}

using namespace std;

typedef struct
{
    wifi_err_reason_t code;
    const char *message;
} ReasonMessage;

typedef enum wifi_application_task_message
{
    WIFI_AS_STA_INIT_MSG = 0,
    WIFI_AS_AP_INIT_MSG,
    WIFI_AS_APSTA_INIT_MSG,
    WIFI_AS_STA_DEINIT_MSG,
    WIFI_AS_AP_DEINIT_MSG,
    WIFI_AS_APSTA_DEINIT_MSG,
    WIFI_CONNECT_TO_AP_MSG,
    WIFI_DISCONNECT_FROM_AP_MSG,
    WIFI_DISCONNECT_FROM_STA_MSG,
} WiFiApplicationTaskMessage;

typedef struct wifi_scan_result
{
    wifi_ap_record_t *apList;
    uint16_t count;
} WiFiScanResult;

// reasonMessageTable has been moved to WiFiManager.cpp to resolve compiler warnings and errors.


class I2CManager; // Forward declaration to avoid circular dependency

class WiFiManager
{
private:
    QueueHandle_t wifiApplicationTaskMessageQueue;
    esp_netif_t *stationModeNetworkInterface = NULL;
    esp_netif_t *accessPointModeNetworkInterface = NULL;

    bool wifiInitialized;
    bool connectedToAp = false;
    wifi_mode_t currentWiFiMode;
    bool isTestingCredentials = false;
    TimerHandle_t reconnectTimer = NULL;
    TaskHandle_t sntpTaskHandle = NULL;
    TaskHandle_t wifiReconnectionTaskHandle = NULL;
    string lastConnectedSSID;
    string lastConnectedPassword;
    bool isConfigModeActive = false;
    TimerHandle_t configModeInactivityTimer = NULL;
    function<void()> onConfigModeTimeoutCallback;

    string currentIPAddressAsSTA;
    string currentConnectedApSSID;
    string currentConnectedApGateWay;
    string currentConnectedApSubnet;
    string currentConnectedApMacID;
    wifi_err_reason_t disconnectionReason = WIFI_REASON_UNSPECIFIED;

    string currentConnectedStaMacID;
    string currentConnectedStaIPAddress;

    function<void(bool)> onWiFiAsStationStartedListener;
    function<void(bool)> onWiFiAsAccessPointStartedListener;
    function<void()> onAccessPointConnectedListener;
    function<void(std::string, wifi_err_reason_t)> onAccessPointDisconnectedListener;
    function<void()> onAccessPointInitialConnectionFailedCallback;

    function<void(string, string)> onNewStationConnectedListener;
    function<void()> onNewStationDisconnectedListener;

    I2CManager *rtcManager = nullptr;
    NVSManager *nvsManager = nullptr;

    void clearConnectionInfo();
    void wifiReconnectionTask();
    static void wifi_reconnection_task_wrapper(void *pvParameters);

    void initialize_wifi();
    esp_err_t start_wifi_as_ap();
    esp_err_t start_wifi_as_sta();
    esp_err_t start_wifi_as_apsta();
    esp_err_t connect_to_ap(string ssid, string password);
    esp_err_t disconnect_from_ap();
    const char *reasonToName(wifi_err_reason_t code);
    friend void timeSyncTask(void *pvParams);

    // function<void(ip4_addr_t assignedIP, )> onWifiAccessPointConnectionListener;
public:
    EventGroupHandle_t wifiEventGroupHandle, sntpTaskEventGroup, wifiTaskEventGroup;

    void startWiFiAsAccessPoint(function<void(bool)> onWiFiAsAccessPointStartedListener);
    void startWiFiAsStation(function<void(bool)> onWiFiAsStationStartedListener);
    void startWiFiAsAccessPointStation(
        function<void(bool)> onWiFiAsAccessPointStartedListener,
        function<void(bool)> onWiFiAsStationStartedListener);
    void connectToAccessPoint(string ssid, string password);
    void startReconnectTimer();
    void stopReconnectTimer();
    void startWiFiAsAccessPointSync();
    void startWiFiAsStationSync();
    void startWiFiAsAccessPointStationSync();
    bool connectToAccessPointSync(string ssid, string password);
    bool disconnectFromApSync();
    bool testConnection(const std::string &ssid, const std::string &password);
    void stopWiFi();
    void setConfigMode(bool isActive);
    void setOnConfigModeTimeoutCallback(function<void()> callback);

    // New function for connecting to nearest saved access point
    bool connectToNearestSavedAccessPoint(WiFiCredentials *credentials, size_t count);

    void setOnNewStationConnectedListener(function<void(string, string)> onNewStationConnectedListener);
    void setOnNewStationDisconnectedListener(function<void()> onNewStationdisconnectedListener);
    void setOnAccessPointConnectedListener(function<void()> onAccessPointConnectedListener);
    void setOnAccessPointDisconnectedListener(function<void(string, wifi_err_reason_t)> onAccessPointDisconnectedListener);
    void setOnAccessPointInitialConnectionFailedListener(function<void()> callback);

    bool getWifiScanResult(WiFiScanResult &scanResult);
    bool getWiFiScanResultInJson(char *&resultJsonString);
    bool getApConnectionStatusInJson(char *&resultJsonString);
    bool getDisconnectionReasonInJson(char *&resultJsonString);

    bool startSNTPSync();
    void stopSNTPSync();
    bool getTimestampMillis(uint64_t &timestamp);

    friend void wifiEventHandler(void *arg, esp_event_base_t eventBase, int32_t eventID, void *eventData);
    friend void ipEventHandler(void *arg, esp_event_base_t eventBase, int32_t eventID, void *eventData);
    friend void reconnectTimerCallback(TimerHandle_t xTimer);
    friend void configModeInactivityTimerCallback(TimerHandle_t xTimer);

    void setRtcManager(I2CManager *manager) { this->rtcManager = manager; }
    void setNvsManager(NVSManager *manager) { this->nvsManager = manager; }

    // Constructor
    WiFiManager();

    // Destructor to clean up timer resources (Requirement 6.4)
    ~WiFiManager();

    bool isWiFiConnected();
};

#endif
