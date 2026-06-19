#ifndef BLUETOOTHMANAGER_H_
#define BLUETOOTHMANAGER_H_

#include "functional"
#include "esp_blufi_api.h"
#include "BluFiCrypto.h"
#include "freertos/event_groups.h"
#include "esp_event.h"

using namespace std;

class BluetoothManager;

void bluFiEventCallback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param);

class BluetoothManager
{
private:
    function<void(bool)> onBlutoothControllerInitializedListener;
    function<void(bool)> onBluFiInitializedListener;

    EventGroupHandle_t bluFiEventGroup;
    bool controllerInitialized;
    bool blufiInitialized;
    bool connectedToClient;

    esp_blufi_callbacks_t bluFiCallbacks = {
        .event_cb = bluFiEventCallback,
        .negotiate_data_handler = bluFiDHNegotiatiationDataHandler,
        .encrypt_func = bluFiAesEncryptFunction,
        .decrypt_func = bluFiAesDecryptFunction,
        .checksum_func = bluFiCRCChecksumFunction,
    };

    esp_err_t espBluFiHostInit();
    esp_err_t espBluFiGAPRegisterCallback();

public:
    void
    inititalizeBluetoothController(function<void(bool)> onBlutoothControllerInitializedListener);
    void initializeBluFi(function<void(bool)> onBluFiInitializedListener);

    bool inititalizeBluetoothControllerSync();
    bool initializeBlufiSync();
    bool sendDataToClient(uint8_t *data, size_t length);

    friend void bluFiEventCallback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param);

    BluetoothManager();
    ~BluetoothManager();
};

#endif