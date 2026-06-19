#include "WiFiManager.h"
#include "esp_idf_version.h"
#include "DynamicCredentialGenerator.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/apps/sntp.h"
#include "lwip/ip4_addr.h"
#include "time.h"
#include "stdlib.h"
#include "sdkconfig.h"
#include "cJSON.h"
#include "string.h"
#include "peripherals/I2CManager.h"
#define TAG "WiFiManager"

using namespace std;

static bool sntpOpModeSet = false;

// Use Kconfig values for SNTP configuration
#define SNTP_SYNC_RETRY_COUNT CONFIG_SNTP_SYNC_RETRY_COUNT

static_assert(CONFIG_SNTP_TIME_SYNC_INTERVAL - (CONFIG_SNTP_TIME_SYNC_RETRY_INTERVAL * SNTP_SYNC_RETRY_COUNT) > 10000, "Invalid SNTP sync interval vs retry configuration");

// Reconnection timer callback
void reconnectTimerCallback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Reconnect timer callback");
    WiFiManager *wifiManager = (WiFiManager *)pvTimerGetTimerID(xTimer);
    if (wifiManager != nullptr && !wifiManager->connectedToAp)
    {
        if (wifiManager->isConfigModeActive)
        {
            ESP_LOGI(TAG, "Reconnect timer callback ignored: Config mode is active.");
            return;
        }
        ESP_LOGI(TAG, "Setting WIFI_RECONNECT_BIT to trigger reconnection task.");
        xEventGroupSetBits(wifiManager->wifiTaskEventGroup, WIFI_RECONNECT_BIT);
    }
    else
    {
        ESP_LOGE(TAG, "Reconnect timer callback: wifiManager is nullptr or connectedToAp is true");
    }
}

// Config mode inactivity timer callback - triggers restart when no station connected for 5 minutes
void configModeInactivityTimerCallback(TimerHandle_t xTimer)
{
    ESP_LOGW(TAG, "Config mode inactivity timer callback triggered");
    WiFiManager *wifiManager = (WiFiManager *)pvTimerGetTimerID(xTimer);
    if (wifiManager != nullptr && wifiManager->isConfigModeActive)
    {
        // Double-check no stations are connected before triggering timeout
        wifi_sta_list_t staList;
        memset(&staList, 0, sizeof(staList));
        esp_wifi_ap_get_sta_list(&staList);
        
        if (staList.num == 0)
        {
            ESP_LOGW(TAG, "Config mode inactivity timeout! No station connected for 5 minutes. Triggering restart...");
            if (wifiManager->onConfigModeTimeoutCallback != nullptr)
            {
                wifiManager->onConfigModeTimeoutCallback();
            }
            else
            {
                ESP_LOGE(TAG, "Config mode timeout callback not set!");
            }
        }
        else
        {
            ESP_LOGI(TAG, "Config mode inactivity timer fired but %d station(s) connected, ignoring.", staList.num);
        }
    }
    else
    {
        ESP_LOGW(TAG, "Config mode inactivity timer callback: wifiManager is nullptr or not in config mode");
    }
}

const char *modeToString(wifi_mode_t mode)
{
    switch (mode)
    {
    case WIFI_MODE_AP:
        return "WIFI_MODE_AP";
    case WIFI_MODE_APSTA:
        return "WIFI_MODE_APSTA";
    case WIFI_MODE_STA:
        return "WIFI_MODE_STA";
    case WIFI_MODE_NULL:
    case WIFI_MODE_MAX:
    default:
        return "WIFI_MODE_UNKNOWN";
    }
    return "";
}

static ReasonMessage reasonMessageTable[] = {
    MESSAGE_TABLE_IT(WIFI_REASON_UNSPECIFIED),
    MESSAGE_TABLE_IT(WIFI_REASON_AUTH_EXPIRE),
    MESSAGE_TABLE_IT(WIFI_REASON_AUTH_LEAVE),
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    MESSAGE_TABLE_IT(WIFI_REASON_ASSOC_EXPIRE),
#endif
    MESSAGE_TABLE_IT(WIFI_REASON_ASSOC_TOOMANY),
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    MESSAGE_TABLE_IT(WIFI_REASON_NOT_AUTHED),
    MESSAGE_TABLE_IT(WIFI_REASON_NOT_ASSOCED),
#endif
    MESSAGE_TABLE_IT(WIFI_REASON_ASSOC_LEAVE),
    MESSAGE_TABLE_IT(WIFI_REASON_ASSOC_NOT_AUTHED),
    MESSAGE_TABLE_IT(WIFI_REASON_DISASSOC_PWRCAP_BAD),
    MESSAGE_TABLE_IT(WIFI_REASON_DISASSOC_SUPCHAN_BAD),
    MESSAGE_TABLE_IT(WIFI_REASON_BSS_TRANSITION_DISASSOC),
    MESSAGE_TABLE_IT(WIFI_REASON_IE_INVALID),
    MESSAGE_TABLE_IT(WIFI_REASON_MIC_FAILURE),
    MESSAGE_TABLE_IT(WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT),
    MESSAGE_TABLE_IT(WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT),
    MESSAGE_TABLE_IT(WIFI_REASON_IE_IN_4WAY_DIFFERS),
    MESSAGE_TABLE_IT(WIFI_REASON_GROUP_CIPHER_INVALID),
    MESSAGE_TABLE_IT(WIFI_REASON_PAIRWISE_CIPHER_INVALID),
    MESSAGE_TABLE_IT(WIFI_REASON_AKMP_INVALID),
    MESSAGE_TABLE_IT(WIFI_REASON_UNSUPP_RSN_IE_VERSION),
    MESSAGE_TABLE_IT(WIFI_REASON_INVALID_RSN_IE_CAP),
    MESSAGE_TABLE_IT(WIFI_REASON_802_1X_AUTH_FAILED),
    MESSAGE_TABLE_IT(WIFI_REASON_CIPHER_SUITE_REJECTED),
    MESSAGE_TABLE_IT(WIFI_REASON_INVALID_PMKID),
    MESSAGE_TABLE_IT(WIFI_REASON_BEACON_TIMEOUT),
    MESSAGE_TABLE_IT(WIFI_REASON_NO_AP_FOUND),
    MESSAGE_TABLE_IT(WIFI_REASON_AUTH_FAIL),
    MESSAGE_TABLE_IT(WIFI_REASON_ASSOC_FAIL),
    MESSAGE_TABLE_IT(WIFI_REASON_HANDSHAKE_TIMEOUT),
    MESSAGE_TABLE_IT(WIFI_REASON_CONNECTION_FAIL),
    MESSAGE_TABLE_IT(WIFI_REASON_AP_TSF_RESET),
    MESSAGE_TABLE_IT(WIFI_REASON_ROAMING),
};

const char *WiFiManager::reasonToName(wifi_err_reason_t code)
{
    size_t i;
    for (i = 0; i < sizeof(reasonMessageTable) / sizeof(reasonMessageTable[0]); ++i)
    {
        if (reasonMessageTable[i].code == code)
        {
            return reasonMessageTable[i].message;
        }
    }
    return "WIFI_REASON_UNKNOWN";
}

WiFiManager::WiFiManager()
{
    // CRITICAL FIX: Set log level to INFO for WiFiManager TAG
    // Something is setting it to ERROR, so we override it here
    esp_log_level_set(TAG, ESP_LOG_INFO);
    
    ESP_LOGE(TAG, "!!! WIFIMANAGER CONSTRUCTOR CALLED - ERROR LEVEL TEST !!!");
    ESP_LOGW(TAG, "!!! WIFIMANAGER CONSTRUCTOR CALLED - WARNING LEVEL TEST !!!");
    ESP_LOGI(TAG, "=== WIFIMANAGER INITIALIZED ===");
    ESP_LOGI(TAG, "  Firmware Build: ENHANCED_LOGGING_v2024.11.05");
    ESP_LOGI(TAG, "  Features: SNTP Time Sync, WiFi Management, Comprehensive Logging");
    ESP_LOGI(TAG, "  Log Level: Explicitly set to INFO");
    
    this->wifiInitialized = false;
    this->currentIPAddressAsSTA = "";
    this->currentWiFiMode = WIFI_MODE_NULL;
    this->currentConnectedApSSID = "";
    this->currentConnectedApGateWay = "";
    this->currentConnectedApSubnet = "";
    this->currentConnectedApMacID = "";
    this->currentConnectedStaMacID = "";
    this->currentConnectedStaIPAddress = "";
    this->onAccessPointInitialConnectionFailedCallback = nullptr;
    this->reconnectTimer = NULL;
    this->lastConnectedSSID = "";
    this->lastConnectedPassword = "";
    this->wifiTaskEventGroup = xEventGroupCreate();
    xTaskCreatePinnedToCore(
        wifi_reconnection_task_wrapper,
        "wifi_reconnect_task",
        CONFIG_WIFI_RECONNECTION_TASK_STACK_DEPTH,
        this,
        CONFIG_WIFI_RECONNECTION_TASK_PRIORITY,
        &this->wifiReconnectionTaskHandle,
        PRO_CPU_NUM);
}

void WiFiManager::startReconnectTimer()
{
    if (this->isConfigModeActive)
    {
        ESP_LOGI(TAG, "Config mode is active, reconnection timer will not be started.");
        return;
    }

    // Create and start reconnection timer if not already created
    if (this->reconnectTimer == NULL)
    {
        this->reconnectTimer = xTimerCreate(
            "WiFiReconnectTimer",
            pdMS_TO_TICKS(WIFI_AP_CONNECT_RETRY_INTERVAL),
            pdTRUE, // Auto reload
            this,
            reconnectTimerCallback);
        ESP_LOGI(TAG, "Reconnect timer created successfully");
    }

    if (this->reconnectTimer != NULL && !xTimerIsTimerActive(this->reconnectTimer))
    {
        xTimerStart(this->reconnectTimer, 0);
        ESP_LOGI(TAG, "Reconnect timer started successfully");
    }
}

void WiFiManager::stopReconnectTimer()
{
    // Stop reconnection timer if it's running
    if (this->reconnectTimer != NULL)
    {
        xTimerStop(this->reconnectTimer, 0);
        xTimerDelete(this->reconnectTimer, 0);
        this->reconnectTimer = NULL;
        ESP_LOGI(TAG, "Reconnect timer stopped successfully");
    }
}

void wifiEventHandler(void *arg, esp_event_base_t eventBase, int32_t eventID, void *eventData)
{

    esp_err_to_name(ESP_OK);

    WiFiManager *wifiManager = (WiFiManager *)arg;

    if (eventBase != WIFI_EVENT)
    {
        ESP_LOGE(TAG, "wifiEventHandler recevied an invalid event base");
        return;
    }

    switch (eventID)
    {
    case WIFI_EVENT_WIFI_READY:
        ESP_LOGI(TAG, "wifiEventHandler -> Wifi Ready !!");
        break;
    case WIFI_EVENT_SCAN_DONE:
        ESP_LOGI(TAG, "wifiEventHandler -> Wifi Scan done !!");
        break;
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_STA_START");
        xEventGroupSetBits(wifiManager->wifiEventGroupHandle, WIFI_AS_STATION_STARTED_BIT);
        break;
    case WIFI_EVENT_STA_STOP:
        ESP_LOGW(TAG, "wifiEventHandler recevied event : WIFI_EVENT_STA_STOP");
        xEventGroupClearBits(wifiManager->wifiEventGroupHandle, WIFI_AS_STATION_STARTED_BIT);
        break;
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_STA_CONNECTED");
        wifiManager->stopReconnectTimer();
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
    {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)eventData;
        ESP_LOGW(TAG, "WiFi: DISCONNECTED - SSID=%s, Reason=%s (code=%d)",
                 (char *)event->ssid, wifiManager->reasonToName((wifi_err_reason_t)event->reason), event->reason);

        wifiManager->disconnectionReason = (wifi_err_reason_t)event->reason;
        wifiManager->currentConnectedApSSID = "";
        wifiManager->currentConnectedApGateWay = "";
        wifiManager->currentIPAddressAsSTA = "";
        wifiManager->currentConnectedApSubnet = "";
        wifiManager->currentConnectedApMacID = "";

        xEventGroupClearBits(wifiManager->wifiEventGroupHandle, WIFI_ACESS_POINT_CONNECTED_BIT);
        if (wifiManager->connectedToAp == true)
        {
            // When the device was previously connected to the access point
            xEventGroupSetBits(wifiManager->wifiEventGroupHandle, WIFI_ACESS_POINT_DISCONNECTED_BIT);
            wifiManager->connectedToAp = false;
            if (wifiManager->onAccessPointDisconnectedListener != NULL)
            {
                if (wifiManager->isConfigModeActive || wifiManager->isTestingCredentials)
                {
                    ESP_LOGI(TAG, "Skipping onAccessPointDisconnectedListener call during credential testing or config mode.");
                }
                else
                {
                    wifiManager->onAccessPointDisconnectedListener(wifiManager->lastConnectedSSID, (wifi_err_reason_t)event->reason);
                }
            }
            wifiManager->clearConnectionInfo();
        }
        else
        {
            // device was trying to connected to the access point but failed in it's attempt
            xEventGroupSetBits(wifiManager->wifiEventGroupHandle, WIFI_ACESS_POINT_CONNECT_FAILED_BIT);
            if (wifiManager->onAccessPointInitialConnectionFailedCallback != NULL)
            {
                wifiManager->onAccessPointInitialConnectionFailedCallback();
            }
        }
        wifiManager->startReconnectTimer();
    }
    break;
        //////////////////////////////////////////////////
    case WIFI_EVENT_AP_START:
    {
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_AP_START");
        xEventGroupSetBits(wifiManager->wifiEventGroupHandle, WIFI_AS_ACCESS_POINT_STARTED_BIT);
    }
    break;
    case WIFI_EVENT_AP_STOP:
    {
        ESP_LOGW(TAG, "wifiEventHandler recevied event : WIFI_EVENT_AP_STOP");
        xEventGroupClearBits(wifiManager->wifiEventGroupHandle, WIFI_AS_ACCESS_POINT_STARTED_BIT);
    }
    break;
    case WIFI_EVENT_AP_STACONNECTED:
    {
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_AP_STACONNECTED");
        // Stop config mode inactivity timer when a station connects
        if (wifiManager->isConfigModeActive && wifiManager->configModeInactivityTimer != NULL)
        {
            xTimerStop(wifiManager->configModeInactivityTimer, 0);
            ESP_LOGI(TAG, "Config mode inactivity timer stopped - station connected to AP");
        }
    }
    break;
    case WIFI_EVENT_AP_STADISCONNECTED:
    {
        ESP_LOGW(TAG, "wifiEventHandler recevied event : WIFI_EVENT_AP_STADISCONNECTED");
        if (wifiManager->onNewStationDisconnectedListener != NULL)
            wifiManager->onNewStationDisconnectedListener();
        
        // Restart config mode inactivity timer when station disconnects (if in config mode and no other stations)
        if (wifiManager->isConfigModeActive && wifiManager->configModeInactivityTimer != NULL)
        {
            // Check if any stations still connected
            wifi_sta_list_t staList;
            memset(&staList, 0, sizeof(staList));
            esp_wifi_ap_get_sta_list(&staList);
            if (staList.num == 0)
            {
                xTimerReset(wifiManager->configModeInactivityTimer, 0);
                ESP_LOGI(TAG, "Config mode inactivity timer restarted - no stations connected to AP");
            }
            else
            {
                ESP_LOGI(TAG, "Station disconnected but %d station(s) still connected, timer not restarted", staList.num);
            }
        }
    }
    break;
    //////////////////////////////////////////////////
    case WIFI_EVENT_STA_AUTHMODE_CHANGE:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_STA_AUTHMODE_CHANGE");
        break;
    //////////////////////////////////////////////////
    case WIFI_EVENT_STA_WPS_ER_SUCCESS:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_STA_WPS_ER_SUCCESS");
        break;
    case WIFI_EVENT_STA_WPS_ER_FAILED:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_STA_WPS_ER_FAILED");
        break;
    case WIFI_EVENT_STA_WPS_ER_TIMEOUT:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_STA_WPS_ER_TIMEOUT");
        break;
    case WIFI_EVENT_STA_WPS_ER_PIN:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_STA_WPS_ER_PIN");
        break;
    case WIFI_EVENT_STA_WPS_ER_PBC_OVERLAP:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_STA_WPS_ER_PBC_OVERLAP");
        break;
    //////////////////////////////////////////////////
    case WIFI_EVENT_AP_PROBEREQRECVED:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_AP_PROBEREQRECVED");
        break;
    case WIFI_EVENT_FTM_REPORT:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_FTM_REPORT");
        break;
    case WIFI_EVENT_STA_BSS_RSSI_LOW:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_STA_BSS_RSSI_LOW");
        break;
    case WIFI_EVENT_ACTION_TX_STATUS:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_ACTION_TX_STATUS");
        break;
    case WIFI_EVENT_ROC_DONE:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_ROC_DONE");
        break;
    case WIFI_EVENT_STA_BEACON_TIMEOUT:
        ESP_LOGI(TAG, "wifiEventHandler recevied event : WIFI_EVENT_STA_BEACON_TIMEOUT");
        break;
    case 43: // Placeholder for the unknown event
        ESP_LOGW(TAG, "wifiEventHandler received unhandled event ID 43. No action taken.");
        break;
    default:
        ESP_LOGE(TAG, "wifiEventHandler received an unhandled WIFI_EVENT: %ld", eventID);
        break;
    }
}

void WiFiManager::clearConnectionInfo()
{
    this->currentIPAddressAsSTA.clear();
    this->currentConnectedApSSID.clear();
    this->currentConnectedApGateWay.clear();
    this->currentConnectedApSubnet.clear();
    this->currentConnectedApMacID.clear();
    this->currentConnectedStaMacID.clear();
    this->currentConnectedStaIPAddress.clear();
}

void ipEventHandler(void *arg, esp_event_base_t eventBase, int32_t eventID, void *eventData)
{
    WiFiManager *wifiManager = (WiFiManager *)arg;

    if (eventBase != IP_EVENT)
    {
        ESP_LOGE(TAG, "ipEventHandler recevied an invalid event base");
        return;
    }

    switch (eventID)
    {
    case IP_EVENT_STA_GOT_IP:
    {
        // Let's now get the IP address of the Access Point the esp32 is connected to.
        wifi_ap_record_t wifiApData;
        ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&wifiApData));

        esp_netif_ip_info_t ipInformation;
        ESP_ERROR_CHECK(esp_netif_get_ip_info(wifiManager->stationModeNetworkInterface, &ipInformation));
        char ip[16];
        char gw[16];
        char netmask[16];
        char macID[19];

        esp_ip4addr_ntoa(&ipInformation.ip, ip, 16);
        esp_ip4addr_ntoa(&ipInformation.netmask, netmask, 16);
        esp_ip4addr_ntoa(&ipInformation.gw, gw, 16);

        sprintf(macID, MACSTR, MAC2STR(wifiApData.bssid));

        wifiManager->currentConnectedApSSID = (char *)wifiApData.ssid;
        wifiManager->currentConnectedApGateWay = gw;
        wifiManager->currentIPAddressAsSTA = ip;
        wifiManager->currentConnectedApSubnet = netmask;
        wifiManager->currentConnectedApMacID = macID;

        ESP_LOGI(TAG, "WiFi: CONNECTED - SSID=%s, IP=%s, Gateway=%s, RSSI=%d",
                 wifiApData.ssid, ip, gw, wifiApData.rssi);

        xEventGroupClearBits(wifiManager->wifiEventGroupHandle, WIFI_ACESS_POINT_DISCONNECTED_BIT);
        wifiManager->connectedToAp = true;
        xEventGroupSetBits(wifiManager->wifiEventGroupHandle, WIFI_ACESS_POINT_CONNECTED_BIT);

        if (wifiManager->onAccessPointConnectedListener != NULL)
        {
            if (wifiManager->isTestingCredentials || wifiManager->isConfigModeActive)
            {
                ESP_LOGI(TAG, "Skipping onAccessPointConnectedListener call during credential testing or config mode.");
            }
            else
            {
                wifiManager->onAccessPointConnectedListener();
            }
        }
    }
    break;
    case IP_EVENT_STA_LOST_IP:
    {
        ESP_LOGW(TAG, "WiFi: LOST IP - Previous IP=%s, SSID=%s",
                 wifiManager->currentIPAddressAsSTA.c_str(),
                 wifiManager->currentConnectedApSSID.c_str());
        wifiManager->clearConnectionInfo();
        wifiManager->connectedToAp = false;
        xEventGroupClearBits(wifiManager->wifiEventGroupHandle, WIFI_ACESS_POINT_CONNECTED_BIT);
        xEventGroupSetBits(wifiManager->wifiEventGroupHandle, WIFI_ACESS_POINT_DISCONNECTED_BIT);
        if (wifiManager->onAccessPointDisconnectedListener != NULL)
        {
            if (wifiManager->isConfigModeActive)
            {
                ESP_LOGI(TAG, "Skipping onAccessPointDisconnectedListener call in config mode.");
            }
            else
            {
                wifiManager->onAccessPointDisconnectedListener(wifiManager->lastConnectedSSID, wifiManager->disconnectionReason);
            }
        }
    }
    break;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    case IP_EVENT_ASSIGNED_IP_TO_CLIENT:
    {
        // Get IP address from eventData
        ip_event_assigned_ip_to_client_t *event = (ip_event_assigned_ip_to_client_t *)eventData;
#else
    case IP_EVENT_AP_STAIPASSIGNED:
    {
        // Get IP address from eventData
        ip_event_ap_staipassigned_t *event = (ip_event_ap_staipassigned_t *)eventData;
#endif
        char ipChar[16];
        esp_ip4addr_ntoa(&event->ip, ipChar, sizeof(ipChar));
        string ipAddress = ipChar;

        // Obtain MAC address of the first station in list (assumes 1:1 mapping with event)
        wifi_sta_list_t connectedStationsList;
        memset(&connectedStationsList, 0, sizeof(connectedStationsList));
        esp_wifi_ap_get_sta_list(&connectedStationsList);
        char macID[19];
        sprintf(macID, MACSTR, MAC2STR(connectedStationsList.sta[0].mac));
        string macAddress = macID;

        ESP_LOGI(TAG, "WiFi AP: Client connected - IP=%s, MAC=%s", ipChar, macID);

        if (wifiManager->onNewStationConnectedListener != NULL)
            wifiManager->onNewStationConnectedListener(ipAddress, macAddress);
    }
    break;
    case IP_EVENT_GOT_IP6:
        ESP_LOGI(TAG, "ipEventHandler recevied event : IP_EVENT_GOT_IP6");
        break;
    case IP_EVENT_ETH_GOT_IP:
        ESP_LOGI(TAG, "ipEventHandler recevied event : IP_EVENT_ETH_GOT_IP");
        break;
    case IP_EVENT_ETH_LOST_IP:
        ESP_LOGI(TAG, "ipEventHandler recevied event : IP_EVENT_ETH_LOST_IP");
        break;
    case IP_EVENT_PPP_GOT_IP:
        ESP_LOGI(TAG, "ipEventHandler recevied event : IP_EVENT_PPP_GOT_IP");
        break;
    case IP_EVENT_PPP_LOST_IP:
        ESP_LOGI(TAG, "ipEventHandler recevied event : IP_EVENT_PPP_LOST_IP");
        break;
    default:
        ESP_LOGE(TAG, "ipEventHandler received an invalid IP_EVENT");
    }
}

void WiFiManager::initialize_wifi()
{
    // CRITICAL FIX: Initialize NVS before WiFi initialization
    // WiFi APSTA mode requires NVS to be initialized first
    // This is safe to call multiple times - returns ESP_OK if already initialized
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS partition was truncated or version mismatch, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    if (nvs_ret == ESP_OK)
    {
        ESP_LOGI(TAG, "NVS initialized successfully for WiFi");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(nvs_ret));
    }

    // Initialize the event group
    this->wifiEventGroupHandle = xEventGroupCreate();

    // Initialize the network interface only if not already initialized
    static bool netif_initialized = false;
    if (!netif_initialized)
    {
        ESP_ERROR_CHECK(esp_netif_init());
        netif_initialized = true;
    }

    // Initialize the event loop only if not already created
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(ret);
    }

    // Now let's register event handlers to the default event loop
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,        // Event Base
        ESP_EVENT_ANY_ID,  // Receive any kind of Wifi Event
        &wifiEventHandler, // Reference to the Wifi Event Handler
        (void *)this,      // Pass the address of the WiFiManager object
        NULL));            // No need to store the handler instance

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,         // Event Base
        ESP_EVENT_ANY_ID, // Receive any kind of IP Event
        &ipEventHandler,  // Reference to the IP Event Handler
        (void *)this,     // Pass the address of the WiFiManager object
        NULL));           // No need to store the handler instance

    // Set an Initial configuration of the wifi
    wifi_init_config_t wifiInitialConfig = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifiInitialConfig));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // If all works well, set the wifi initialized flag to true
    this->wifiInitialized = true;
    this->currentWiFiMode = WIFI_MODE_NULL;
}

esp_err_t WiFiManager::start_wifi_as_ap()
{
    // Check if the wifi as been initialized first.
    if (this->wifiInitialized != true)
    {
        ESP_LOGE(TAG, "Wifi needs to be initialized first");
        return ESP_FAIL;
    }

    if (this->currentWiFiMode != WIFI_MODE_NULL)
    {
        ESP_LOGE(TAG, "WiFi already running !, close & reinitialize to continue");
        return ESP_FAIL;
    }

    // Inititialize the tcpip stack and the network interface for the Access Point
    this->accessPointModeNetworkInterface = esp_netif_create_default_wifi_ap();

    wifi_config_t accessPointConfiguration = {
        .ap = {},
    };

    char ssid[32] = {0};
    char password[64] = {0};

#ifdef CONFIG_WIFI_USE_DYNAMIC_CREDENTIALS
    // Generate dynamic credentials based on device MAC address
    ESP_LOGI(TAG, "Generating dynamic AP credentials...");
    
    esp_err_t ret = DynamicCredentialGenerator::generateSSID(ssid, sizeof(ssid));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to generate dynamic SSID, using static fallback");
        strncpy(ssid, WIFI_AP_SSID, sizeof(ssid) - 1);
    }
    
    ret = DynamicCredentialGenerator::generatePassword(password, sizeof(password));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to generate dynamic password, using static fallback");
        strncpy(password, WIFI_AP_PASSWORD, sizeof(password) - 1);
    }
    
    ESP_LOGI(TAG, "Using dynamic AP credentials - SSID: %s", ssid);
#else
    // Use static credentials from configuration
    strncpy(ssid, WIFI_AP_SSID, sizeof(ssid) - 1);
    strncpy(password, WIFI_AP_PASSWORD, sizeof(password) - 1);
    ESP_LOGI(TAG, "Using static AP credentials - SSID: %s", ssid);
#endif

    // Configure Wifi Access Points
    memcpy(accessPointConfiguration.ap.ssid, ssid, (strlen(ssid) + 1));
    memcpy(accessPointConfiguration.ap.password, password, (strlen(password) + 1));
    accessPointConfiguration.ap.ssid_len = strlen(ssid);

    // Set the Wifi access point configuration
    accessPointConfiguration.ap.channel = WIFI_AP_CHANNEL;
    accessPointConfiguration.ap.ssid_hidden = WIFI_AP_SSID_HIDDEN;
    accessPointConfiguration.ap.max_connection = WIFI_AP_MAX_CONNECTIONS;
    accessPointConfiguration.ap.beacon_interval = WIFI_AP_BEACON_INTERVAL;
    accessPointConfiguration.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    // Now let's configure the DHCP Server and the network interface.
    esp_netif_ip_info_t accessPointIPInfo;
    memset(&accessPointIPInfo, 0x00, sizeof(accessPointIPInfo));
    esp_netif_dhcps_stop(this->accessPointModeNetworkInterface);
    inet_pton(AF_INET, WIFI_AP_IP, &accessPointIPInfo.ip);
    inet_pton(AF_INET, WIFI_AP_GATEWAY, &accessPointIPInfo.gw);
    inet_pton(AF_INET, WIFI_AP_NETMASK, &accessPointIPInfo.netmask);

    // configure the IP info & restart the dhcp server
    ESP_ERROR_CHECK(esp_netif_set_ip_info(this->accessPointModeNetworkInterface, &accessPointIPInfo));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(this->accessPointModeNetworkInterface));

    // Now configure set the WiFi Access Point Configuration
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &accessPointConfiguration));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_AP_BANDWIDTH));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_STA_POWER_SAVE));
    ESP_ERROR_CHECK(esp_wifi_start());

    this->currentWiFiMode = WIFI_MODE_AP;

    return ESP_OK;
}
esp_err_t WiFiManager::start_wifi_as_sta()
{
    // Check if the wifi as been initialized first.
    if (this->wifiInitialized != true)
    {
        ESP_LOGE(TAG, "Wifi needs to be initialized first");
        return ESP_FAIL;
    }

    if (this->currentWiFiMode != WIFI_MODE_NULL)
    {
        ESP_LOGE(TAG, "WiFi already running !, close & reinitialize to continue");
        return ESP_FAIL;
    }

    // Inititialize the tcpip stack and the network interface for the Station
    this->stationModeNetworkInterface = esp_netif_create_default_wifi_sta();

    string ssid = "myssid";
    string password = "myPassword";

    wifi_config_t wifiConfiguration = {};
    memcpy(wifiConfiguration.sta.ssid, ssid.c_str(), ssid.length());
    memcpy(wifiConfiguration.sta.password, password.c_str(), password.length());

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiConfiguration));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_STA_POWER_SAVE));
    ESP_ERROR_CHECK(esp_wifi_start());

    this->currentWiFiMode = WIFI_MODE_STA;
    return ESP_OK;
}
esp_err_t WiFiManager::start_wifi_as_apsta()
{
    // Check if the wifi as been initialized first.
    if (this->wifiInitialized != true)
    {
        ESP_LOGE(TAG, "Wifi needs to be initialized first");
        return ESP_FAIL;
    }

    if (this->currentWiFiMode != WIFI_MODE_NULL)
    {
        ESP_LOGE(TAG, "WiFi already running !, close & reinitialize to continue");
        return ESP_FAIL;
    }

    // Station mode config
    // Inititialize the tcpip stack and the network interface for the Access Point Station mode
    this->stationModeNetworkInterface = esp_netif_create_default_wifi_sta();
    this->accessPointModeNetworkInterface = esp_netif_create_default_wifi_ap();
    string ssidSta = "myssid";
    string passwordSta = "myPassword";

    wifi_config_t wifiStaConfiguration = {};
    memcpy(wifiStaConfiguration.sta.ssid, ssidSta.c_str(), ssidSta.length());
    memcpy(wifiStaConfiguration.sta.password, passwordSta.c_str(), passwordSta.length());

    // AP Mode config
    wifi_config_t accessPointConfiguration = {
        .ap = {},
    };

    char ssid[32] = {0};
    char password[64] = {0};

#ifdef CONFIG_WIFI_USE_DYNAMIC_CREDENTIALS
    // Generate dynamic credentials based on device MAC address
    ESP_LOGI(TAG, "Generating dynamic AP credentials for APSTA mode...");
    
    esp_err_t ret = DynamicCredentialGenerator::generateSSID(ssid, sizeof(ssid));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to generate dynamic SSID, using static fallback");
        strncpy(ssid, WIFI_AP_SSID, sizeof(ssid) - 1);
    }
    
    ret = DynamicCredentialGenerator::generatePassword(password, sizeof(password));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to generate dynamic password, using static fallback");
        strncpy(password, WIFI_AP_PASSWORD, sizeof(password) - 1);
    }
    
    ESP_LOGI(TAG, "Using dynamic AP credentials - SSID: %s", ssid);
#else
    // Use static credentials from configuration
    strncpy(ssid, WIFI_AP_SSID, sizeof(ssid) - 1);
    strncpy(password, WIFI_AP_PASSWORD, sizeof(password) - 1);
    ESP_LOGI(TAG, "Using static AP credentials - SSID: %s", ssid);
#endif

    // Configure Wifi Access Points
    memcpy(accessPointConfiguration.ap.ssid, ssid, (strlen(ssid) + 1));
    memcpy(accessPointConfiguration.ap.password, password, (strlen(password) + 1));
    accessPointConfiguration.ap.ssid_len = strlen(ssid);

    // Set the Wifi access point configuration
    accessPointConfiguration.ap.channel = WIFI_AP_CHANNEL;
    accessPointConfiguration.ap.ssid_hidden = WIFI_AP_SSID_HIDDEN;
    accessPointConfiguration.ap.max_connection = WIFI_AP_MAX_CONNECTIONS;
    accessPointConfiguration.ap.beacon_interval = WIFI_AP_BEACON_INTERVAL;
    accessPointConfiguration.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    // Now configure set the WiFi APSTA Configuration BEFORE configuring DHCP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiStaConfiguration));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &accessPointConfiguration));

    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_AP_BANDWIDTH));

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // Start WiFi before configuring DHCP
    ESP_ERROR_CHECK(esp_wifi_start());

    // Wait a bit for the AP interface to be fully up
    vTaskDelay(pdMS_TO_TICKS(100));

    // Now configure the DHCP Server and the network interface AFTER WiFi is started
    esp_netif_ip_info_t accessPointIPInfo;
    memset(&accessPointIPInfo, 0x00, sizeof(accessPointIPInfo));

    // Stop DHCP server first
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(this->accessPointModeNetworkInterface));

    // Configure IP addresses
    inet_pton(AF_INET, WIFI_AP_IP, &accessPointIPInfo.ip);
    inet_pton(AF_INET, WIFI_AP_GATEWAY, &accessPointIPInfo.gw);
    inet_pton(AF_INET, WIFI_AP_NETMASK, &accessPointIPInfo.netmask);

    // Set the IP info
    ESP_ERROR_CHECK(esp_netif_set_ip_info(this->accessPointModeNetworkInterface, &accessPointIPInfo));

    // Start DHCP server with default settings
    ESP_ERROR_CHECK(esp_netif_dhcps_start(this->accessPointModeNetworkInterface));
    ESP_LOGI(TAG, "DHCP server started on AP interface with IP range 192.168.4.2 - 192.168.4.254");

    this->currentWiFiMode = WIFI_MODE_APSTA;

    return ESP_OK;
}
esp_err_t WiFiManager::connect_to_ap(string ssid, string password)
{
    if (this->currentWiFiMode != WIFI_MODE_STA && this->currentWiFiMode != WIFI_MODE_APSTA)
    {
        ESP_LOGE(TAG, "This device needs to be initialized with the right wifi mode in order to connect to an Access point.");
        return ESP_FAIL;
    }
    wifi_config_t wifi_config = {};
    memset(&wifi_config, 0, sizeof(wifi_config_t));

    strncpy((char *)wifi_config.sta.ssid, ssid.c_str(), sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password.c_str(), sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    // Store the credentials for reconnection
    this->lastConnectedSSID = ssid;
    this->lastConnectedPassword = password;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    return esp_wifi_connect();
}
esp_err_t WiFiManager::disconnect_from_ap()
{
    return esp_wifi_disconnect();
}

void WiFiManager::startWiFiAsAccessPoint(function<void(bool)> onWiFiAsAccessPointStartedListener)
{
}
void WiFiManager::startWiFiAsStation(function<void(bool)> onWiFiAsStationStartedListener)
{
    if (this->currentWiFiMode == WIFI_MODE_STA)
    {
        ESP_LOGE(TAG, "Already running in STA mode");
        return;
    }

    if (this->wifiInitialized != true)
        this->initialize_wifi();

    if (this->start_wifi_as_sta() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start Wifi As Station !");
        return;
    }
}
void WiFiManager::startWiFiAsAccessPointStation(
    function<void(bool)> onWiFiAsAccessPointStartedListener,
    function<void(bool)> onWiFiAsStationStartedListener)
{
}
void WiFiManager::connectToAccessPoint(string ssid, string password)
{
    if (ssid.length() <= 0 || password.length() <= 0)
    {
        ESP_LOGE(TAG, "Invalid WiFi SSID or password");
        return;
    }

    if (this->connect_to_ap(ssid, password) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to connect to Access Point");
        return;
    }
}

void WiFiManager::startWiFiAsAccessPointSync()
{
    if (this->currentWiFiMode == WIFI_MODE_AP)
    {
        ESP_LOGE(TAG, "Already running in AP mode");
        return;
    }

    if (this->wifiInitialized != true)
        this->initialize_wifi();

    if (this->start_wifi_as_ap() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start Wifi As Access Point !");
        return;
    }

    // Wait for the WIFI_AS_STATION_STARTED_BIT
    xEventGroupWaitBits(
        this->wifiEventGroupHandle,
        WIFI_AS_ACCESS_POINT_STARTED_BIT,
        pdTRUE,
        pdFALSE,
        portMAX_DELAY);
}
void WiFiManager::startWiFiAsStationSync()
{
    if (this->currentWiFiMode == WIFI_MODE_STA)
    {
        ESP_LOGE(TAG, "Already running in STA mode");
        return;
    }

    if (this->wifiInitialized != true)
        this->initialize_wifi();

    if (this->start_wifi_as_sta() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start Wifi As Station !");
        return;
    }

    // Wait for the WIFI_AS_STATION_STARTED_BIT
    xEventGroupWaitBits(
        this->wifiEventGroupHandle,
        WIFI_AS_STATION_STARTED_BIT,
        pdTRUE,
        pdFALSE,
        portMAX_DELAY);
}
void WiFiManager::startWiFiAsAccessPointStationSync()
{
    if (this->currentWiFiMode == WIFI_MODE_APSTA)
    {
        ESP_LOGE(TAG, "Already running in APSTA mode");
        return;
    }

    if (this->wifiInitialized != true)
        this->initialize_wifi();

    if (this->start_wifi_as_apsta() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start Wifi As APSTA !");
        return;
    }

    // Wait for the WIFI_AS_STATION_STARTED_BIT
    xEventGroupWaitBits(
        this->wifiEventGroupHandle,
        WIFI_AS_STATION_STARTED_BIT | WIFI_AS_ACCESS_POINT_STARTED_BIT,
        pdTRUE,
        pdTRUE,
        portMAX_DELAY);
}
bool WiFiManager::connectToAccessPointSync(string ssid, string password)
{

    if (ssid.length() <= 0 || password.length() <= 0)
    {
        ESP_LOGE(TAG, "Invalid WiFi SSID or password");
        return false;
    }

    if (this->connect_to_ap(ssid, password) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to connect to Access Point");
        return false;
    }

    // Wait for the WIFI_ACESS_POINT_CONNECTED_BIT | WIFI_ACESS_POINT_CONNECT_FAILED_BIT
    EventBits_t waitBits = xEventGroupWaitBits(
        this->wifiEventGroupHandle,
        WIFI_ACESS_POINT_CONNECTED_BIT | WIFI_ACESS_POINT_CONNECT_FAILED_BIT,
        pdTRUE,
        pdFALSE,
        portMAX_DELAY);

    if ((waitBits & WIFI_ACESS_POINT_CONNECT_FAILED_BIT) != 0)
    {
        // Failed to connect to the Access Point
        ESP_LOGE(TAG, "Failed to connect to the access point");
        return false;
    }
    if ((waitBits & WIFI_ACESS_POINT_CONNECTED_BIT) != 0)
    {
        ESP_LOGI(TAG, "Connected to SSID : %s | got IP : %s | subnet : %s | gateway : %s | AP MACID : %s ",
                 this->currentConnectedApSSID.c_str(),
                 this->currentIPAddressAsSTA.c_str(),
                 this->currentConnectedApSubnet.c_str(),
                 this->currentConnectedApGateWay.c_str(),
                 this->currentConnectedApMacID.c_str());
        return true;
    }

    return false;
}
bool WiFiManager::disconnectFromApSync()
{
    // Check if the device is currently connected to any access point
    if (this->connectedToAp == false)
    {
        ESP_LOGE(TAG, "Not connected to any access points as of now");
        return false;
    }

    esp_err_t ret = this->disconnect_from_ap();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to disconnect from Access Point | %s", esp_err_to_name(ret));
        return false;
    }

    // Wait for the WIFI_ACESS_POINT_CONNECTED_BIT | WIFI_ACESS_POINT_DISCONNECTED_BIT
    EventBits_t waitBits = xEventGroupWaitBits(
        this->wifiEventGroupHandle,
        WIFI_ACESS_POINT_DISCONNECTED_BIT | WIFI_ACESS_POINT_DISCONNECT_FAILED_BIT,
        pdTRUE,
        pdFALSE,
        portMAX_DELAY);

    if ((waitBits & WIFI_ACESS_POINT_DISCONNECT_FAILED_BIT) != 0)
    {
        // Failed to connect to the Access Point
        ESP_LOGE(TAG, "Failed to connect to the access point");
        return false;
    }
    if ((waitBits & WIFI_ACESS_POINT_DISCONNECTED_BIT) != 0)
    {
        ESP_LOGI(TAG, "Disconnected from Access Point");
        return true;
    }

    return false;
}

bool WiFiManager::getWifiScanResult(WiFiScanResult &scanResult)
{
    scanResult.apList = NULL;
    scanResult.count = 0;

    if (this->currentWiFiMode != WIFI_MODE_STA && this->currentWiFiMode != WIFI_MODE_APSTA)
    {
        ESP_LOGE(TAG, "This device needs to be initialized with the right wifi mode in order to scan nearby Access points.");
        return false;
    }

    wifi_scan_config_t scanConf = {};
    scanConf.ssid = NULL;
    scanConf.bssid = NULL;
    scanConf.channel = 0;
    scanConf.show_hidden = true;
    scanConf.scan_type = WIFI_SCAN_TYPE_ACTIVE;

    esp_wifi_scan_start(&scanConf, true);
    esp_wifi_scan_get_ap_num(&(scanResult.count));

    if (scanResult.count == 0)
    {
        return false;
    }

    scanResult.apList = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * scanResult.count);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&(scanResult.count), scanResult.apList));

    return true;
}
bool WiFiManager::getWiFiScanResultInJson(char *&resultJsonString)
{
    // Limit scan results to prevent stack overflow during JSON serialization
    // With 16KB stack for command processor, 25 items is safe for JSON serialization
    static const int MAX_SCAN_RESULTS_IN_RESPONSE = 25;

    WiFiScanResult scanResult;
    scanResult.apList = NULL;
    scanResult.count = 0;

    if (this->getWifiScanResult(scanResult) == false)
    {
        resultJsonString = NULL;
        return false;
    }

    if (scanResult.apList == 0 || scanResult.count == 0)
    {
        resultJsonString = NULL;
        return false;
    }

    // Limit results to prevent buffer overflow (ESP-IDF sorts by RSSI, strongest first)
    int resultsToReturn = (scanResult.count > MAX_SCAN_RESULTS_IN_RESPONSE) 
                          ? MAX_SCAN_RESULTS_IN_RESPONSE 
                          : scanResult.count;

    cJSON *arrayRoot, *root;
    root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "count", resultsToReturn);
    cJSON_AddStringToObject(root, "success", "true");
    arrayRoot = cJSON_CreateArray();

    for (int i = 0; i < resultsToReturn; i++)
    {
        cJSON *element;

        char macAddress[19];
        sprintf(macAddress, MACSTR, MAC2STR(scanResult.apList[i].bssid));

        // Safely copy country code - wifi_country_t.cc is only 3 bytes and may not be null-terminated
        char countryCode[4] = {0};
        memcpy(countryCode, scanResult.apList[i].country.cc, 2);
        countryCode[2] = '\0';

        // Safely copy SSID - ensure null termination even if AP sends malformed data
        char ssidSafe[33] = {0};
        memcpy(ssidSafe, scanResult.apList[i].ssid, 32);
        ssidSafe[32] = '\0';

        element = cJSON_CreateObject();
        cJSON_AddNumberToObject(element, "rssi", scanResult.apList[i].rssi);
        cJSON_AddStringToObject(element, "mac", macAddress);
        cJSON_AddStringToObject(element, "ssid", ssidSafe);
        cJSON_AddNumberToObject(element, "authmode", scanResult.apList[i].authmode);
        cJSON_AddStringToObject(element, "country", countryCode);

        cJSON_AddItemToArray(arrayRoot, element);
    }

    cJSON_AddItemToObject(root, "scanResults", arrayRoot);
    resultJsonString = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    free(scanResult.apList);

    return true;
}
bool WiFiManager::getApConnectionStatusInJson(char *&resultJsonString)
{
    cJSON *root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "connected", this->connectedToAp ? "true" : "false");
    cJSON_AddStringToObject(root, "mode", modeToString(this->currentWiFiMode));

    if (this->connectedToAp)
    {
        cJSON_AddStringToObject(root, "ssid", this->currentConnectedApSSID.c_str());
        cJSON_AddStringToObject(root, "ipAddress", this->currentIPAddressAsSTA.c_str());
        cJSON_AddStringToObject(root, "macAddress", this->currentConnectedApMacID.c_str());
        cJSON_AddStringToObject(root, "gateway", this->currentConnectedApGateWay.c_str());
        cJSON_AddStringToObject(root, "subnet", this->currentConnectedApSubnet.c_str());
        
        // Get real-time RSSI of connected AP
        int rssi = 0;
        esp_err_t err = esp_wifi_sta_get_rssi(&rssi);
        if (err == ESP_OK)
        {
            cJSON_AddNumberToObject(root, "rssi", rssi);
            ESP_LOGD(TAG, "Current RSSI: %d dBm", rssi);
        }
        else
        {
            // If RSSI retrieval fails, add 0 to indicate unavailable
            cJSON_AddNumberToObject(root, "rssi", 0);
            ESP_LOGW(TAG, "Failed to get current RSSI, error: %d", err);
        }
    }

    resultJsonString = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return true;
}
bool WiFiManager::getDisconnectionReasonInJson(char *&resultJsonString)
{
    cJSON *root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "code", this->disconnectionReason);
    cJSON_AddStringToObject(root, "codeStr", reasonToName(this->disconnectionReason));

    resultJsonString = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);  // Clean up cJSON object (Requirement 6.1)

    return true;
}

void initSNTPTimeSync()
{
    if (sntpOpModeSet == false)
    {
        ESP_LOGI(TAG, "SNTP Init: Setting operating mode to POLL");
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntpOpModeSet = true;
    }
    
    ESP_LOGI(TAG, "SNTP Init: Configuring NTP servers");
    ESP_LOGI(TAG, "  Primary Server: %s", SNTP_SERVER_NAME_0);
    sntp_setservername(0, SNTP_SERVER_NAME_0);
    
    ESP_LOGI(TAG, "  Secondary Server: %s", SNTP_SERVER_NAME_1);
    sntp_setservername(1, SNTP_SERVER_NAME_1);
    
    ESP_LOGI(TAG, "SNTP Init: Initializing SNTP client...");
    sntp_init();
    ESP_LOGI(TAG, "SNTP Init: Client initialized successfully");
}

void timeSyncTask(void *pvParams)
{
    WiFiManager *wifiManager = (WiFiManager *)pvParams;

    uint64_t lastSyncTick = 0;
    bool firstSync = true;
    bool wifiWaitLogged = false;
    
    ESP_LOGI(TAG, "=== SNTP TIME SYNC TASK STARTED ===");
    ESP_LOGI(TAG, "  Sync Interval: %d ms (%.1f minutes)", 
             CONFIG_SNTP_TIME_SYNC_INTERVAL, CONFIG_SNTP_TIME_SYNC_INTERVAL / 60000.0f);
    ESP_LOGI(TAG, "  Retry Interval: %d ms", CONFIG_SNTP_TIME_SYNC_RETRY_INTERVAL);
    ESP_LOGI(TAG, "  Max Retry Attempts: %d", SNTP_SYNC_RETRY_COUNT);
    
    while (true)
    {
        // If WiFi not connected, wait a second and retry
        if (!wifiManager->connectedToAp)
        {
            if (!wifiWaitLogged)
            {
                ESP_LOGI(TAG, "SNTP Sync: Waiting for WiFi connection before syncing time...");
                wifiWaitLogged = true;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // WiFi just connected
        if (wifiWaitLogged)
        {
            ESP_LOGI(TAG, "SNTP Sync: WiFi connected, ready to sync time");
            wifiWaitLogged = false;
        }

        uint64_t nowTick = xTaskGetTickCount();
        // For the first sync, skip the interval check to sync immediately after WiFi connects
        // For subsequent syncs, wait for the configured interval
        if (!firstSync && (nowTick - lastSyncTick) < pdMS_TO_TICKS(CONFIG_SNTP_TIME_SYNC_INTERVAL))
        {
            // Not time yet for next sync
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        if (!firstSync)
        {
            ESP_LOGI(TAG, "SNTP Sync: Periodic sync interval reached, starting new sync cycle");
        }
        else
        {
            ESP_LOGI(TAG, "SNTP Sync: First sync - attempting immediately after WiFi connection");
        }
        firstSync = false;

        bool synced = false;
        ESP_LOGI(TAG, "=== SNTP TIME SYNC CYCLE START ===");
        ESP_LOGI(TAG, "  WiFi Status: %s", wifiManager->connectedToAp ? "CONNECTED" : "DISCONNECTED");
        ESP_LOGI(TAG, "  Max Attempts: %d", SNTP_SYNC_RETRY_COUNT);
        ESP_LOGI(TAG, "  Retry Interval: %d ms", CONFIG_SNTP_TIME_SYNC_RETRY_INTERVAL);
        
        // Log current system time before sync attempt
        time_t timeBefore = 0;
        struct tm timeInfoBefore = {};
        time(&timeBefore);
        localtime_r(&timeBefore, &timeInfoBefore);
        char timeStrBefore[32];
        strftime(timeStrBefore, sizeof(timeStrBefore), "%Y-%m-%d %H:%M:%S", &timeInfoBefore);
        ESP_LOGI(TAG, "  Current System Time (before sync): %s", timeStrBefore);
        
        for (int attempt = 0; attempt < SNTP_SYNC_RETRY_COUNT && wifiManager->connectedToAp; ++attempt)
        {
            ESP_LOGI(TAG, "SNTP Sync: Attempt %d/%d - Initializing SNTP client...", 
                     attempt + 1, SNTP_SYNC_RETRY_COUNT);
            
            // Try to obtain time via SNTP
            initSNTPTimeSync();

            // Wait 2 seconds for SNTP to update time
            ESP_LOGI(TAG, "SNTP Sync: Waiting 2 seconds for server response...");
            vTaskDelay(pdMS_TO_TICKS(2000));

            time_t now = 0;
            struct tm timeInfo = {};
            time(&now);
            localtime_r(&now, &timeInfo);

            // Format the received time for logging
            char timeStr[32];
            strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S %Z", &timeInfo);
            
            ESP_LOGI(TAG, "SNTP Sync: Received time from system: %s", timeStr);
            ESP_LOGI(TAG, "SNTP Sync: Year check: %d (threshold: 2022)", timeInfo.tm_year + 1900);

            if (timeInfo.tm_year >= (2022 - 1900))
            {
                synced = true;
                ESP_LOGI(TAG, "=== SNTP SYNC SUCCESS ===");
                ESP_LOGI(TAG, "  Attempt: %d/%d", attempt + 1, SNTP_SYNC_RETRY_COUNT);
                ESP_LOGI(TAG, "  Synced Time: %s", timeStr);
                ESP_LOGI(TAG, "  Unix Timestamp: %ld", (long)now);
                ESP_LOGI(TAG, "  Day of Week: %s", 
                         (const char*[]){"Sunday", "Monday", "Tuesday", "Wednesday", 
                                        "Thursday", "Friday", "Saturday"}[timeInfo.tm_wday]);
                
                // Calculate and log time adjustment
                long timeDelta = (long)(now - timeBefore);
                if (timeDelta != 0)
                {
                    ESP_LOGI(TAG, "  Time Adjustment: %+ld seconds (%s)", 
                             timeDelta, 
                             timeDelta > 0 ? "forward" : "backward");
                }
                else
                {
                    ESP_LOGI(TAG, "  Time Adjustment: No change (already synchronized)");
                }

                // If RTC manager available, update external RTC as well
                if (wifiManager->rtcManager != nullptr && wifiManager->rtcManager->isRTCConnected())
                {
                    ESP_LOGI(TAG, "DS1307 RTC: Updating external RTC with synced time...");
                    bool rtcUpdateSuccess = wifiManager->rtcManager->setTime(timeInfo);
                    
                    if (rtcUpdateSuccess)
                    {
                        ESP_LOGI(TAG, "DS1307 RTC: UPDATE SUCCESS");
                        ESP_LOGI(TAG, "  RTC Time Set: %s", timeStr);
                        
                        // Verify the RTC was updated correctly by reading it back
                        struct tm rtcTimeInfo = {};
                        if (wifiManager->rtcManager->readTime(rtcTimeInfo))
                        {
                            char rtcTimeStr[32];
                            strftime(rtcTimeStr, sizeof(rtcTimeStr), "%Y-%m-%d %H:%M:%S", &rtcTimeInfo);
                            ESP_LOGI(TAG, "  RTC Verification: %s", rtcTimeStr);
                            
                            // Check if times match (within 2 seconds tolerance)
                            time_t rtcTime = mktime(&rtcTimeInfo);
                            long timeDiff = (long)(now - rtcTime);
                            if (labs(timeDiff) <= 2)
                            {
                                ESP_LOGI(TAG, "  RTC Verification: PASSED (time matches within 2 seconds)");
                            }
                            else
                            {
                                ESP_LOGW(TAG, "  RTC Verification: WARNING (time difference: %ld seconds)", 
                                         timeDiff);
                            }
                        }
                        else
                        {
                            ESP_LOGW(TAG, "  RTC Verification: Could not read back RTC time");
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "DS1307 RTC: UPDATE FAILED");
                        ESP_LOGE(TAG, "  Failed to write time to external RTC");
                        ESP_LOGE(TAG, "  RTC may be disconnected or malfunctioning");
                    }
                }
                else
                {
                    if (wifiManager->rtcManager == nullptr)
                    {
                        ESP_LOGW(TAG, "DS1307 RTC: RTC Manager not initialized, skipping RTC update");
                    }
                    else if (!wifiManager->rtcManager->isRTCConnected())
                    {
                        ESP_LOGW(TAG, "DS1307 RTC: RTC not connected, skipping RTC update");
                        ESP_LOGW(TAG, "  Check I2C connections and RTC hardware");
                    }
                }
                ESP_LOGI(TAG, "=== SNTP SYNC COMPLETE ===");
                break;
            }
            else
            {
                ESP_LOGW(TAG, "SNTP Sync: ATTEMPT %d FAILED", attempt + 1);
                ESP_LOGW(TAG, "  Received invalid time (year: %d)", timeInfo.tm_year + 1900);
                ESP_LOGW(TAG, "  This usually indicates SNTP server not responding");
                
                if (attempt + 1 < SNTP_SYNC_RETRY_COUNT)
                {
                    ESP_LOGI(TAG, "  Retrying in %d ms...", CONFIG_SNTP_TIME_SYNC_RETRY_INTERVAL);
                    vTaskDelay(pdMS_TO_TICKS(CONFIG_SNTP_TIME_SYNC_RETRY_INTERVAL));
                }
            }
        }

        if (!synced)
        {
            ESP_LOGE(TAG, "=== SNTP SYNC FAILED ===");
            ESP_LOGE(TAG, "  All %d attempts exhausted", SNTP_SYNC_RETRY_COUNT);
            ESP_LOGE(TAG, "  Possible causes:");
            ESP_LOGE(TAG, "    - SNTP servers unreachable");
            ESP_LOGE(TAG, "    - Network firewall blocking NTP traffic (UDP port 123)");
            ESP_LOGE(TAG, "    - DNS resolution issues");
            ESP_LOGE(TAG, "    - Internet connectivity problems");
            ESP_LOGE(TAG, "  Next sync attempt in %d ms", CONFIG_SNTP_TIME_SYNC_INTERVAL);
        }
        lastSyncTick = xTaskGetTickCount();
    }
    vTaskDelete(NULL);
}

bool WiFiManager::startSNTPSync()
{
    if (this->sntpTaskEventGroup == NULL)
        this->sntpTaskEventGroup = xEventGroupCreate();

    if (this->sntpTaskEventGroup == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for sntp task event group");
        return true;
    }

    ESP_LOGI(TAG, "Starting SNTP time sync task");

    // Let's start up the task for syncing the stnp time
    BaseType_t result = xTaskCreatePinnedToCore(
        timeSyncTask,
        "timeSyncTask",
        SNTP_TIME_SYNC_TASK_STACK_DEPTH,
        this,
        SNTP_TIME_SYNC_TASK_PRIORITY,
        &(this->sntpTaskHandle),
        SNTP_TIME_SYNC_TASK_CORE);

    // EventBits_t eventBits = xEventGroupWaitBits(
    //     this->sntpTaskEventGroup,
    //     SNTP_TASK_STARTED_BIT,
    //     pdTRUE,
    //     pdFALSE,
    //     portMAX_DELAY);

    return (result == pdPASS) ? true : false;

    // if ((eventBits & SNTP_TASK_STARTED_BIT) != 0)
    // {
    //     // Task has been successfully created now delete the event group and return
    //     // vEventGroupDelete(this->sntpTaskEventGroup);
    //     this->sntpTaskEventGroup = NULL;
    //     return true;
    // }

    // return true;
}

void WiFiManager::stopSNTPSync()
{
    if (this->sntpTaskHandle != NULL)
    {
        vTaskDelete(this->sntpTaskHandle);
        this->sntpTaskEventGroup = NULL;
        ESP_LOGW(TAG, "SNTP time sync task deleted");
    }
    if (this->sntpTaskEventGroup != NULL)
    {
        vEventGroupDelete(this->sntpTaskEventGroup);
        this->sntpTaskEventGroup = NULL;
        ESP_LOGW(TAG, "SNTP time sync task event group deleted");
    }
}

bool WiFiManager::getTimestampMillis(uint64_t &timestamp)
{
    // Let's first check if the time is updated.
    time_t now = 0;
    struct tm timeInfo = {};
    time(&now);
    localtime_r(&now, &timeInfo);

    //
    if (timeInfo.tm_year < (2016 - 1900))
    {
        ESP_LOGE(TAG, "%s() -> Looks like the time is not yet updated ", __func__);
        return false;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);
    timestamp = (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
    return true;
}

void WiFiManager::setOnNewStationConnectedListener(function<void(string, string)> onNewStationConnectedListener)
{
    this->onNewStationConnectedListener = onNewStationConnectedListener;
}
void WiFiManager::setOnNewStationDisconnectedListener(function<void()> onNewStationDisconnectedListener)
{
    this->onNewStationDisconnectedListener = onNewStationDisconnectedListener;
}
void WiFiManager::setOnAccessPointConnectedListener(function<void()> onAccessPointConnectedListener)
{
    this->onAccessPointConnectedListener = onAccessPointConnectedListener;
}
void WiFiManager::setOnAccessPointDisconnectedListener(function<void(string, wifi_err_reason_t)> onAccessPointDisconnectedListener)
{
    this->onAccessPointDisconnectedListener = onAccessPointDisconnectedListener;
}

void WiFiManager::setOnAccessPointInitialConnectionFailedListener(function<void()> callback)
{
    this->onAccessPointInitialConnectionFailedCallback = callback;
}

void WiFiManager::wifi_reconnection_task_wrapper(void *pvParameters)
{
    WiFiManager *wifiManager = static_cast<WiFiManager *>(pvParameters);
    wifiManager->wifiReconnectionTask();
}

void WiFiManager::wifiReconnectionTask()
{
    while (true)
    {
        if (this->isConfigModeActive)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        EventBits_t bits = xEventGroupWaitBits(this->wifiTaskEventGroup, WIFI_RECONNECT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

        if ((bits & WIFI_RECONNECT_BIT) != 0)
        {
            ESP_LOGI(TAG, "WiFi reconnection task triggered. Attempting to reconnect...");

            if (this->nvsManager == nullptr)
            {
                ESP_LOGE(TAG, "NVSManager is not set. Cannot fetch credentials for reconnection.");
                continue;
            }

            size_t credentialsCount = 0;
            if (!nvsManager->getCredentialsCount(credentialsCount))
            {
                ESP_LOGE(TAG, "Failed to get credentials count from NVS.");
                continue;
            }

            if (credentialsCount == 0)
            {
                ESP_LOGW(TAG, "No saved credentials found in NVS. Cannot reconnect.");
                continue;
            }

            if (credentialsCount == 1)
            {
                WiFiCredentials *credential = nullptr;
                size_t count = 0;
                if (nvsManager->getSavedCredentials(credential, count) && credential != nullptr)
                {
                    ESP_LOGI(TAG, "Attempting to reconnect to single saved AP: %s", credential[0].ssid);
                    connectToAccessPoint(string(credential[0].ssid), string(credential[0].password));
                    free(credential);
                }
            }
            else
            {
                WiFiCredentials *credentials = nullptr;
                size_t count = 0;
                if (nvsManager->getSavedCredentials(credentials, count) && credentials != nullptr)
                {
                    ESP_LOGI(TAG, "Attempting to reconnect to nearest of %zu saved APs", count);
                    connectToNearestSavedAccessPoint(credentials, count);
                    free(credentials);
                }
            }
        }
    }
}

// Create two  functions named isWiFiConnected() & isWiFiDisconnected() that
// return true or false depending on whether the wifi is connected or disconnected.
bool WiFiManager::isWiFiConnected()
{
    return this->connectedToAp;
}

void WiFiManager::stopWiFi()
{
    ESP_LOGW(TAG, "Stopping WiFi...");

    // Stop and delete the reconnect timer if it's running
    if (this->reconnectTimer != NULL)
    {
        xTimerStop(this->reconnectTimer, portMAX_DELAY);
        xTimerDelete(this->reconnectTimer, portMAX_DELAY);
        this->reconnectTimer = NULL;
        ESP_LOGI(TAG, "Reconnect timer stopped and deleted.");
    }

    // Stop the WiFi driver
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT)
    {
        ESP_LOGW(TAG, "WiFi not initialized, no need to stop.");
        return;
    }
    ESP_ERROR_CHECK(err);

    // De-initialize WiFi driver
    ESP_ERROR_CHECK(esp_wifi_deinit());

    // Unregister event handlers
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, &ipEventHandler));

    // Don't delete the default event loop - it should persist
    // esp_event_loop_delete_default() removed

    // Destroy network interfaces if they exist
    if (this->stationModeNetworkInterface != NULL)
    {
        esp_netif_destroy(this->stationModeNetworkInterface);
        this->stationModeNetworkInterface = NULL;
    }
    if (this->accessPointModeNetworkInterface != NULL)
    {
        // Make sure DHCP server is stopped before destroying interface
        esp_netif_dhcps_stop(this->accessPointModeNetworkInterface);
        esp_netif_destroy(this->accessPointModeNetworkInterface);
        this->accessPointModeNetworkInterface = NULL;
    }

    this->wifiInitialized = false;
    this->currentWiFiMode = WIFI_MODE_NULL;
    ESP_LOGI(TAG, "WiFi stopped successfully.");
}

bool WiFiManager::connectToNearestSavedAccessPoint(WiFiCredentials *credentials, size_t count)
{
    if (credentials == NULL || count == 0)
    {
        ESP_LOGE(TAG, "Invalid credentials or count is zero");
        return false;
    }

    ESP_LOGI(TAG, "Scanning for nearby access points to match with %zu saved credentials", count);

    // Get WiFi scan results
    WiFiScanResult scanResult;
    if (!getWifiScanResult(scanResult))
    {
        ESP_LOGE(TAG, "Failed to get WiFi scan results");
        return false;
    }

    if (scanResult.apList == NULL || scanResult.count == 0)
    {
        ESP_LOGW(TAG, "No access points found in scan");
        return false;
    }

    // Structure to hold matching APs with their signal strength
    typedef struct
    {
        char ssid[MAX_SSID_LEN];
        char password[MAX_PASSWORD_LEN];
        int8_t rssi;
    } MatchingAP;

    MatchingAP *matchingAPs = (MatchingAP *)malloc(sizeof(MatchingAP) * count);
    size_t matchingCount = 0;

    // Find matching APs from scan results
    for (uint16_t i = 0; i < scanResult.count; i++)
    {
        for (size_t j = 0; j < count; j++)
        {
            if (strcmp((char *)scanResult.apList[i].ssid, credentials[j].ssid) == 0)
            {
                // Found a matching AP
                strcpy(matchingAPs[matchingCount].ssid, credentials[j].ssid);
                strcpy(matchingAPs[matchingCount].password, credentials[j].password);
                matchingAPs[matchingCount].rssi = scanResult.apList[i].rssi;
                matchingCount++;
                ESP_LOGI(TAG, "Found matching AP: %s with RSSI: %d", credentials[j].ssid, scanResult.apList[i].rssi);
                break; // Move to next scan result
            }
        }
    }

    // Free scan results as we no longer need them
    free(scanResult.apList);

    if (matchingCount == 0)
    {
        ESP_LOGW(TAG, "No saved credentials match nearby access points");
        free(matchingAPs);
        return false;
    }

    // Find the AP with the highest signal strength (highest RSSI)
    int bestIndex = 0;
    for (size_t i = 1; i < matchingCount; i++)
    {
        if (matchingAPs[i].rssi > matchingAPs[bestIndex].rssi)
        {
            bestIndex = i;
        }
    }

    ESP_LOGI(TAG, "Connecting to AP with highest signal strength: %s (RSSI: %d)",
             matchingAPs[bestIndex].ssid, matchingAPs[bestIndex].rssi);

    // Store the best AP info before freeing memory
    char bestSSID[MAX_SSID_LEN];
    char bestPassword[MAX_PASSWORD_LEN];
    strcpy(bestSSID, matchingAPs[bestIndex].ssid);
    strcpy(bestPassword, matchingAPs[bestIndex].password);

    // Attempt to connect to the best AP
    bool connectionResult = connectToAccessPointSync(
        string(bestSSID),
        string(bestPassword));

    free(matchingAPs);

    if (connectionResult)
    {
        ESP_LOGI(TAG, "Successfully connected to %s", bestSSID);
        return true;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to connect to %s", bestSSID);
        return false;
    }
}

bool WiFiManager::testConnection(const std::string &ssid, const std::string &password)
{
    this->isTestingCredentials = true;
    bool result = connectToAccessPointSync(ssid, password);
    disconnectFromApSync();
    this->isTestingCredentials = false;
    return result;
}

void WiFiManager::setConfigMode(bool isActive)
{
    this->isConfigModeActive = isActive;
    
    if (isActive)
    {
        ESP_LOGI(TAG, "Entering config mode - starting inactivity timer");
        // Create and start inactivity timer when entering config mode
        if (this->configModeInactivityTimer == NULL)
        {
            this->configModeInactivityTimer = xTimerCreate(
                "ConfigModeInactivityTimer",
                pdMS_TO_TICKS(CONFIG_MODE_IDLE_TIMEOUT),
                pdFALSE, // One-shot timer (not auto-reload)
                this,
                configModeInactivityTimerCallback);
            
            if (this->configModeInactivityTimer == NULL)
            {
                ESP_LOGE(TAG, "Failed to create config mode inactivity timer!");
                return;
            }
        }
        
        if (xTimerStart(this->configModeInactivityTimer, 0) == pdPASS)
        {
            ESP_LOGI(TAG, "Config mode inactivity timer started (5 minute timeout)");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to start config mode inactivity timer!");
        }
    }
    else
    {
        ESP_LOGI(TAG, "Exiting config mode - stopping inactivity timer");
        // Stop and delete timer when exiting config mode
        if (this->configModeInactivityTimer != NULL)
        {
            xTimerStop(this->configModeInactivityTimer, 0);
            xTimerDelete(this->configModeInactivityTimer, 0);
            this->configModeInactivityTimer = NULL;
            ESP_LOGI(TAG, "Config mode inactivity timer stopped and deleted");
        }
    }
}


void WiFiManager::setOnConfigModeTimeoutCallback(function<void()> callback)
{
    this->onConfigModeTimeoutCallback = callback;
    ESP_LOGI(TAG, "Config mode timeout callback registered");
}

// Destructor to clean up timer and task resources (Requirement 6.4)
WiFiManager::~WiFiManager()
{
    ESP_LOGI(TAG, "WiFiManager destructor called - cleaning up resources");

    // Clean up reconnect timer
    if (this->reconnectTimer != NULL)
    {
        xTimerStop(this->reconnectTimer, portMAX_DELAY);
        xTimerDelete(this->reconnectTimer, portMAX_DELAY);
        this->reconnectTimer = NULL;
        ESP_LOGI(TAG, "Reconnect timer cleaned up");
    }

    // Clean up config mode inactivity timer
    if (this->configModeInactivityTimer != NULL)
    {
        xTimerStop(this->configModeInactivityTimer, portMAX_DELAY);
        xTimerDelete(this->configModeInactivityTimer, portMAX_DELAY);
        this->configModeInactivityTimer = NULL;
        ESP_LOGI(TAG, "Config mode inactivity timer cleaned up");
    }

    // Clean up event groups
    if (this->wifiEventGroupHandle != NULL)
    {
        vEventGroupDelete(this->wifiEventGroupHandle);
        this->wifiEventGroupHandle = NULL;
    }

    if (this->sntpTaskEventGroup != NULL)
    {
        vEventGroupDelete(this->sntpTaskEventGroup);
        this->sntpTaskEventGroup = NULL;
    }

    if (this->wifiTaskEventGroup != NULL)
    {
        vEventGroupDelete(this->wifiTaskEventGroup);
        this->wifiTaskEventGroup = NULL;
    }

    ESP_LOGI(TAG, "WiFiManager cleanup complete");
}
