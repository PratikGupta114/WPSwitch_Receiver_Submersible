
#include "BluetoothManager.h"
#include "BluFiCrypto.h"

extern "C"
{
#include "esp_blufi.h"
}

// #include "esp_blufi.h"
#include "esp_blufi_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_log.h"
#include "esp_err.h"

#define TAG "BluetoothManager"

BluetoothManager *bluetoothManager = NULL;

#define BLUFI_INIT_FINISH_BIT BIT0

void bluFiEventCallback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLUFI_EVENT_INIT_FINISH:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_INIT_FINISH");
        if (bluetoothManager != NULL)
            xEventGroupSetBits(bluetoothManager->bluFiEventGroup, BLUFI_INIT_FINISH_BIT);

        // Start to advertise the packets for
        esp_blufi_adv_start();
    }
    break;
    case ESP_BLUFI_EVENT_DEINIT_FINISH:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_DEINIT_FINISH");
    }
    break;
    case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_SET_WIFI_OPMODE");
    }
    break;
    case ESP_BLUFI_EVENT_BLE_CONNECT:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_BLE_CONNECT");
        esp_blufi_adv_stop();
        bluetoothManager->connectedToClient = true;
        bluFiCryptoInit();
    }
    break;
    case ESP_BLUFI_EVENT_BLE_DISCONNECT:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_BLE_DISCONNECT");
        bluFiCryptoDeinit();
        bluetoothManager->connectedToClient = true;
        esp_blufi_adv_start();
    }
    break;
    case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP");
    }
    break;
    case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP");
    }
    break;
    case ESP_BLUFI_EVENT_GET_WIFI_STATUS:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_GET_WIFI_STATUS");
    }
    break;
    case ESP_BLUFI_EVENT_DEAUTHENTICATE_STA:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_DEAUTHENTICATE_STA");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_STA_BSSID:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_STA_BSSID");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_STA_SSID:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_STA_SSID");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
    {
        ESP_LOGI(TAG, "bluFi Event: case ESP_BLUFI_EVENT_RECV_STA_PASSWD");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_SSID:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_SOFTAP_SSID");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_MAX_CONN_NUM:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_SOFTAP_MAX_CONN_NUM");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_AUTH_MODE:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_SOFTAP_AUTH_MODE");
        ESP_LOGI(TAG, "Recv SOFTAP AUTH MODE %d\n", param->softap_auth_mode.auth_mode);
    }
    break;
    case ESP_BLUFI_EVENT_RECV_SOFTAP_CHANNEL:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_SOFTAP_CHANNEL");
        ESP_LOGI(TAG, "Recv SOFTAP CHANNEL %d\n", param->softap_channel.channel);
    }
    break;
    case ESP_BLUFI_EVENT_RECV_USERNAME:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_USERNAME");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_CA_CERT:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_CA_CERT");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_CLIENT_CERT");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_SERVER_CERT");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY");
    }
    break;
    case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE");
        esp_blufi_disconnect();
    }
    break;
    case ESP_BLUFI_EVENT_GET_WIFI_LIST:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_GET_WIFI_LIST");
    }
    break;
    case ESP_BLUFI_EVENT_REPORT_ERROR:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_REPORT_ERROR");
        esp_blufi_send_error_info(param->report_error.state);
    }
    break;
    case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
    {
        ESP_LOGI(TAG, "bluFi Event: ESP_BLUFI_EVENT_RECV_CUSTOM_DATA");
        ESP_LOGI(TAG, "Recv Custom Data %.*s\n", param->custom_data.data_len, param->custom_data.data);
    }
    break;
    }
}

esp_err_t BluetoothManager::espBluFiHostInit()
{
    // Initialize the bluedroid controller first
    int ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "Bluedroid init failed | %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    // Then enable the bluedroid controller and check for any errors
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "Bluedroid init failed | %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Bluedroid Initialized | BD ADDRESS : " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));

    return ESP_OK;
}

esp_err_t BluetoothManager::espBluFiGAPRegisterCallback()
{
    int returnCode = esp_ble_gap_register_callback(esp_blufi_gap_event_handler);

    if (returnCode)
    {
        return returnCode;
    }

    return esp_blufi_profile_init();
}

void BluetoothManager::initializeBluFi(function<void(bool)> onBluFiInitializedListener)
{
}

bool BluetoothManager::initializeBlufiSync()
{

    if (this->controllerInitialized == false)
    {
        ESP_LOGE(TAG, "Please Initialize bluetooth controller first");
        return false;
    }

    esp_err_t returnCode = espBluFiHostInit();

    if (returnCode)
    {
        ESP_LOGE(TAG, "Initializing blufi host failed | %s", esp_err_to_name(returnCode));
        return false;
    }

    returnCode = esp_blufi_register_callbacks(&(this->bluFiCallbacks));
    if (returnCode)
    {
        ESP_LOGE(TAG, "blufi callback register failed, error code : %x", returnCode);
        return false;
    }

    returnCode = espBluFiGAPRegisterCallback();
    if (returnCode)
    {
        ESP_LOGE(TAG, "Gap register failed, error code : %x", returnCode);
        return false;
    }

    // Now let's wait until the blufi init event is trigered
    xEventGroupWaitBits(
        this->bluFiEventGroup,
        BLUFI_INIT_FINISH_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY);

    return true;
}

bool BluetoothManager::inititalizeBluetoothControllerSync()
{
    // Deallocate all the memory allocated prior to this for classic bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_err_t returnCode = ESP_OK;

    // Let us initialize the bluetooth controller first
    esp_bt_controller_config_t bluetoothControllerConfiguration = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bluetoothControllerConfiguration.mode = ESP_BT_MODE_BLE;

    returnCode = esp_bt_controller_init(&bluetoothControllerConfiguration);

    if (returnCode != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initiaize bluetooth controller | %s", esp_err_to_name(returnCode));
        this->controllerInitialized = false;
        return false;
    }

    returnCode = esp_bt_controller_enable(ESP_BT_MODE_BLE);

    if (returnCode != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable Bluetooth LE | %s", esp_err_to_name(returnCode));
        this->controllerInitialized = false;
        return false;
    }

    this->controllerInitialized = true;
    return true;
}

bool BluetoothManager::sendDataToClient(uint8_t *data, size_t length)
{
    if (this->controllerInitialized != true && this->blufiInitialized != true && this->connectedToClient != true)
    {
        ESP_LOGE(TAG, "%s() -> Unable to send data to client", __func__);
        return false;
    }

    esp_err_t returnCode = esp_blufi_send_custom_data(data, (uint32_t)length);
    if (returnCode != ESP_OK)
    {
        ESP_LOGE(TAG, "%s() -> failed to send data to client | %s", __func__, esp_err_to_name(returnCode));
        return false;
    }

    return true;
}

BluetoothManager::BluetoothManager()
{
    bluetoothManager = this;
    this->bluFiEventGroup = xEventGroupCreate();
}
BluetoothManager::~BluetoothManager()
{
    vEventGroupDelete(this->bluFiEventGroup);
    bluetoothManager = NULL;
}