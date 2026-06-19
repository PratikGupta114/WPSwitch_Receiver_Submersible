
#include <string>
#include <list>
#include "string.h"
#include "nvsManager.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "esp_err.h"

#define NVS_WIFI_STATION_CREDENTIALS_NAMESPACE "stacreds"
#define NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE "params"
#define MAX_CREDENTIAL_COUNT (5)

#define SSID_KEY "ssid"
#define PASSWORD_KEY "passwd"
#define CREDENTIALS_KEY "creds"
#define CREDENTIALS_COUNT_KEY "credsCnt"

#define PARAM_DATA_RECEPTION_TIMEOUT_KEY "dr_timeout"
#define PARAM_MIN_WATER_LEVEL_TO_START_PUMP_KEY "max_wl_start_p"
#define PARAM_MAX_WATER_LEVEL_TO_STOP_PUMP_KEY "max_wl_stop_p"
#define PARAM_MIN_WATER_LEVEL_TO_ALLOW_PUMP_CONTROL_KEY "min_wl_apc"

#define TAG "NVSManager"

using namespace std;

NVSManager::NVSManager()
{
    this->nvsInitSuccess = false;
}

bool NVSManager::init()
{
    if (this->nvsInitSuccess)
    {
        ESP_LOGI(TAG, "NVS already initialized");
        return true;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize NVS Flash");
            this->nvsInitSuccess = false;
            return false;
        }
        else
        {
            ESP_LOGI(TAG, "NVS Flash initialized after erase");
            this->nvsInitSuccess = true;
            return true;
        }
    }
    else if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "NVS Flash initialized");
        this->nvsInitSuccess = true;
        return true;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize NVS Flash: %s", esp_err_to_name(err));
        this->nvsInitSuccess = false;
        return false;
    }
}

NVSManager::~NVSManager()
{
    esp_err_t err = nvs_flash_deinit();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to de-inititialize NVS FLash");
        this->nvsInitSuccess = true;
    }
    else
    {
        ESP_LOGI(TAG, "NVS FLash de-Inititalized ");
        this->nvsInitSuccess = false;
    }
}

bool NVSManager::addCredential(WiFiCredentials &credential)
{

    ESP_LOGI(TAG, "Call to add Credential");

    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager not initialized");
        return false;
    }

    string ssid = credential.ssid;
    if (this->hasCredential(ssid) == true)
    {
        ESP_LOGE(TAG, "A Credential with the name '%s' already exists", ssid.c_str());
        return false;
    }

    nvs_handle_t handle;
    esp_err_t result;
    result = nvs_open(NVS_WIFI_STATION_CREDENTIALS_NAMESPACE, NVS_READWRITE, &handle);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs flash : %s", esp_err_to_name(result));
        return false;
    }

    uint8_t count = 0;
    result = nvs_get_u8(handle, CREDENTIALS_COUNT_KEY, &count);

    if (result == ESP_ERR_NVS_NOT_FOUND)
        ESP_LOGE(TAG, "Credentials count key does not exist hence using initial value of 0 :");
    else if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read credentials count from nvs : %s", esp_err_to_name(result));
        return false;
    }

    if (count >= MAX_CREDENTIAL_COUNT)
    {
        ESP_LOGE(TAG, "Exceeded max credentials limit ! cannot store more than %d credentials ", MAX_CREDENTIAL_COUNT);
        return false;
    }

    // get the credentials data as blob inside a dynamically allocated array
    WiFiCredentials *credentialsArray = (WiFiCredentials *)malloc(sizeof(WiFiCredentials) * (count + 1));

    size_t requiredSize = count * sizeof(WiFiCredentials);
    result = nvs_get_blob(handle, CREDENTIALS_KEY, credentialsArray, &requiredSize);

    if (result == ESP_ERR_NVS_NOT_FOUND)
        ESP_LOGE(TAG, "Credentials key does not exist hence using an empty array as initial value ");
    else if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read credentials from nvs : %s", esp_err_to_name(result));
        free(credentialsArray);
        return false;
    }

    memcpy(credentialsArray[count].ssid, credential.ssid, (strlen(credential.ssid) + 1));
    memcpy(credentialsArray[count].password, credential.password, (strlen(credential.password) + 1));

    count++;
    result = nvs_set_u8(handle, CREDENTIALS_COUNT_KEY, count);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed update count value in nvs : %s", esp_err_to_name(result));
        free(credentialsArray);
        return false;
    }

    requiredSize = count * sizeof(WiFiCredentials);
    result = nvs_set_blob(handle, CREDENTIALS_KEY, credentialsArray, requiredSize);
    free(credentialsArray);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed update credentials in nvs : %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_commit(handle);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed commit changes in the credentials list in nvs : %s", esp_err_to_name(result));
        return false;
    }

    nvs_close(handle);

    return true;
}
bool NVSManager::hasCredential(string ssid)
{
    WiFiCredentials *credential = NULL;

    if (this->getCredentialBySSID(ssid, credential) == false)
        return false;

    if (credential == NULL)
        return false;
    else
        free(credential);

    return true;
}
bool NVSManager::deleteCredentialWithSSID(string ssid)
{

    ESP_LOGI(TAG, "Call to delete Credential");
    // Check if the credential with the given ssid even exists
    if (this->hasCredential(ssid) == false)
    {
        ESP_LOGE(TAG, "No credential exists with ssid %s", ssid.c_str());
        return false;
    }

    // Fetch all the credentials and store the values inside a list
    WiFiCredentials *credentials;
    size_t count = 0;

    if (this->getSavedCredentials(credentials, count) == false)
        return false;

    list<WiFiCredentials> credentialsList;
    int index = -1;
    for (int i = 0; i < count; i++)
    {
        credentialsList.push_back(credentials[i]);
        // store the index of the credential with the given ssid (if any)
        if (strcmp(credentials[i].ssid, ssid.c_str()) == 0)
            index = i;
    }
    // free(credentials);

    // remove the credential from the list and convert the list to an array,
    list<WiFiCredentials>::iterator indexIterator = credentialsList.begin();
    advance(indexIterator, index);
    credentialsList.erase(indexIterator);
    count--;

    WiFiCredentials *updatedCredentials = (WiFiCredentials *)malloc(count * (sizeof(WiFiCredentials)));
    list<WiFiCredentials>::iterator i;
    int j;
    for (i = credentialsList.begin(), j = 0; i != credentialsList.end(); i++, j++)
    {
        memcpy(updatedCredentials[j].ssid, (*i).ssid, strlen((*i).ssid) + 1);
        memcpy(updatedCredentials[j].password, (*i).password, strlen((*i).password) + 1);
    }
    if (!credentialsList.empty())
        credentialsList.clear();
    free(credentials);

    // open the nvs flash
    nvs_handle_t handle;
    esp_err_t result;
    result = nvs_open(NVS_WIFI_STATION_CREDENTIALS_NAMESPACE, NVS_READWRITE, &handle);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs flash : %s", esp_err_to_name(result));
        return false;
    }

    // update the blob field and credentials count in the nvs flash.
    result = nvs_set_u8(handle, CREDENTIALS_COUNT_KEY, (uint8_t)count);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed update count value in nvs : %s", esp_err_to_name(result));
        free(updatedCredentials);
        return false;
    }

    size_t byteSize = count * sizeof(WiFiCredentials);
    result = nvs_set_blob(handle, CREDENTIALS_KEY, updatedCredentials, byteSize);
    free(updatedCredentials);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed update credentials in nvs : %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_commit(handle);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed commit changes in the credentials list in nvs : %s", esp_err_to_name(result));
        return false;
    }

    nvs_close(handle);

    return true;
}
bool NVSManager::getSavedCredentials(WiFiCredentials *&credentials, size_t &count)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager not initialized");
        credentials = NULL;
        return false;
    }

    nvs_handle_t handle;
    esp_err_t result;
    result = nvs_open(NVS_WIFI_STATION_CREDENTIALS_NAMESPACE, NVS_READONLY, &handle);

    if (result != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to open nvs flash : %s", esp_err_to_name(result));
        credentials = NULL;
        return false;
    }

    uint8_t c = 0;
    result = nvs_get_u8(handle, CREDENTIALS_COUNT_KEY, &c);

    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "Credentials count key does not exist hence using initial value of 0 :");
        credentials = NULL;
        return false;
    }
    else if (result != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to read credentials count from nvs : %s", esp_err_to_name(result));
        credentials = NULL;
        return false;
    }

    // get the credentials data as blob inside a dynamically allocated array
    credentials = (WiFiCredentials *)malloc(sizeof(WiFiCredentials) * (c));

    size_t requiredSize = c * sizeof(WiFiCredentials);
    result = nvs_get_blob(handle, CREDENTIALS_KEY, credentials, &requiredSize);

    if (result != ESP_OK)
    {
        free(credentials);
        count = 0;
        credentials = NULL;
        if (result == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Credentials key does not exist hence using an empty array as initial value ");
            return true;
        }
        ESP_LOGE(TAG, "Failed to read credentials from nvs : %s", esp_err_to_name(result));
        return false;
    }

    nvs_close(handle);
    count = c;

    return true;
}
bool NVSManager::getCredentialsCount(size_t &count)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager not initialized");
        return false;
    }

    nvs_handle_t handle;
    esp_err_t result;
    result = nvs_open(NVS_WIFI_STATION_CREDENTIALS_NAMESPACE, NVS_READONLY, &handle);

    if (result != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to open nvs flash : %s", esp_err_to_name(result));
        count = 0;
        return false;
    }

    uint8_t c = 0;
    result = nvs_get_u8(handle, CREDENTIALS_COUNT_KEY, &c);

    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "Credentials count key does not exist hence using initial value of 0 :");
        count = 0;
        return true;
    }
    else if (result != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to read credentials count from nvs : %s", esp_err_to_name(result));
        count = 0;
        return false;
    }

    nvs_close(handle);

    count = c;
    ESP_LOGW(TAG, "Saved Credentials count : %d", count);
    return true;
}
bool NVSManager::getCredentialBySSID(string ssid, WiFiCredentials *&credential)
{
    size_t count = 0;
    WiFiCredentials *credentialsArray = NULL;

    if (this->getSavedCredentials(credentialsArray, count) == false)
    {
        ESP_LOGE(TAG, "Failed to read credentials from nvs");
        if (credentialsArray != NULL)
            free(credentialsArray);
        return false;
    }

    if (count == 0)
    {
        ESP_LOGE(TAG, "No saved credentials found");
        if (credentialsArray != NULL)
            free(credentialsArray);
        credential = NULL;
        return true;
    }

    // now let's search for the credential
    for (int i = 0; i < count; i++)
    {
        ESP_LOGI(TAG, "str 1 : %s | str 2 : %s", credentialsArray[i].ssid, ssid.c_str());
        if (strcmp(credentialsArray[i].ssid, ssid.c_str()) == 0)
        {
            // allocate memory for this credential and copy the contents to the reference passed.
            credential = (WiFiCredentials *)malloc(sizeof(WiFiCredentials));
            memcpy(credential->ssid, credentialsArray[i].ssid, strlen(credentialsArray[i].ssid) + 1);
            memcpy(credential->password, credentialsArray[i].password, strlen(credentialsArray[i].password) + 1);
            if (credentialsArray != NULL)
                free(credentialsArray);
            return true;
        }
    }

    ESP_LOGW(TAG, "No credential found with given ssid");
    if (credentialsArray != NULL)
    {
        free(credentialsArray);
        credentialsArray = NULL;
    }
    credential = NULL;
    return true;
}
void NVSManager::printCredentials()
{
    // get the credentials data as blob inside a dynamically allocated array
    size_t count = 0;
    WiFiCredentials *credentialsArray = NULL;

    if (this->getSavedCredentials(credentialsArray, count) == false)
    {
        ESP_LOGE(TAG, "Failed to read credentials from nvs");
        return;
    }

    if (count <= 0)
    {
        ESP_LOGW(TAG, "\n[ NO CREDENTIALS SAVED\n ]");
        return;
    }

    int bfpl = 0;
    char *printBuffer = (char *)malloc(700 * sizeof(char));

    bfpl += sprintf(printBuffer + bfpl, "Total Credentials : %d", count);
    bfpl += sprintf(printBuffer + bfpl, "\n-----------------------------------");
    bfpl += sprintf(printBuffer + bfpl, "\nSSID \t\t| PASSWORD \t\t");

    for (int i = 0; i < count; i++)
    {
        bfpl += sprintf(printBuffer + bfpl, "\n%s \t| %s \t", credentialsArray[i].ssid, credentialsArray[i].password);
    }

    bfpl += sprintf(printBuffer + bfpl, "\n-----------------------------------\n");

    ESP_LOGW(TAG, "%.*s", bfpl, printBuffer);
    free(printBuffer);
    free(credentialsArray);
}
bool NVSManager::clearCredentials()
{

    if (this->nvsInitSuccess == false)
        return false;

    nvs_handle handle;
    esp_err_t result;
    ESP_LOGI(TAG, "Clearing Wifi Credentials from the flash ");

    result = nvs_open(NVS_WIFI_STATION_CREDENTIALS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs handler : %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_erase_key(handle, CREDENTIALS_COUNT_KEY);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to delete credentials count : %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_erase_key(handle, CREDENTIALS_KEY);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to delete credentials : %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_commit(handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit NVS Flash after clearing credentials : %s", esp_err_to_name(result));
        return false;
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "NVS flash cleared successfully!");
    return true;
}
bool NVSManager::getSavedCredentialsInJson(char *&jsonResultStringHolder)
{
    WiFiCredentials *credentials;
    size_t count = 0;

    if (this->getSavedCredentials(credentials, count) == false)
    {
        jsonResultStringHolder = NULL;
        return false;
    }

    // if (count <= 0)
    // {
    //     jsonResultStringHolder = NULL;
    //     return false;
    // }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "count", count);
    cJSON_AddStringToObject(root, "success", "true");
    cJSON *arrayRoot = cJSON_CreateArray();

    for (int i = 0; i < count; i++)
    {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "ssid", credentials[i].ssid);
        cJSON_AddStringToObject(item, "password", credentials[i].password);
        cJSON_AddItemToArray(arrayRoot, item);
    }

    cJSON_AddItemToObject(root, "credentials", arrayRoot);
    jsonResultStringHolder = cJSON_PrintUnformatted(root);

    cJSON_Delete(root);
    return true;
}

esp_err_t NVSManager::setuint8(const char *namespaceName, const char *keyName, uint8_t num)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager needs to be initialized first");
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }

    nvs_handle handle;
    esp_err_t result = nvs_open(namespaceName, NVS_READWRITE, &handle);
    if (result != ESP_OK)
        return result;

    result = nvs_set_u8(handle, keyName, num);
    if (result != ESP_OK)
    {
        nvs_close(handle);
        return result;
    }

    result = nvs_commit(handle);
    if (result != ESP_OK)
    {
        nvs_close(handle);
        return result;
    }

    nvs_close(handle);
    return ESP_OK;
}
esp_err_t NVSManager::getuint8(const char *namespaceName, const char *keyName, uint8_t &num)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager needs to be initialized first");
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }

    nvs_handle handle;
    esp_err_t result = nvs_open(namespaceName, NVS_READONLY, &handle);
    if (result != ESP_OK)
        return result;

    result = nvs_get_u8(handle, keyName, &num);
    nvs_close(handle);

    if (result != ESP_OK)
        return result;
    return ESP_OK;
}
bool NVSManager::setMinWaterLevelToStartPump(uint8_t level)
{
    esp_err_t result = setuint8(
        NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE,
        PARAM_MIN_WATER_LEVEL_TO_START_PUMP_KEY,
        level);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set min water level to start pump, to nvs flash : %s", esp_err_to_name(result));
        return false;
    }
    return true;
}
bool NVSManager::setMaxWaterLevelToStopPump(uint8_t level)
{
    esp_err_t result = setuint8(
        NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE,
        PARAM_MAX_WATER_LEVEL_TO_STOP_PUMP_KEY,
        level);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set max water level to stop pump to nvs flash : %s", esp_err_to_name(result));
        return false;
    }
    return true;
}
bool NVSManager::setMinWaterLevelToAllowPumpControl(uint8_t level)
{
    esp_err_t result = setuint8(
        NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE,
        PARAM_MIN_WATER_LEVEL_TO_ALLOW_PUMP_CONTROL_KEY,
        level);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set min water level to allow pump control to nvs flash : %s", esp_err_to_name(result));
        return false;
    }
    return true;
}

esp_err_t NVSManager::setManualLockoutDurationMs(uint32_t duration_ms)
{
    if (duration_ms < MANUAL_LOCKOUT_DURATION_MS_MIN || duration_ms > MANUAL_LOCKOUT_DURATION_MS_MAX)
    {
        ESP_LOGE(TAG, "Invalid manual lockout duration: %" PRIu32 " (must be %d-%d ms)", duration_ms, MANUAL_LOCKOUT_DURATION_MS_MIN, MANUAL_LOCKOUT_DURATION_MS_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return result;
    }

    result = nvs_set_u32(handle, "man_lock_dur", duration_ms);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set manual lockout duration: %s", esp_err_to_name(result));
        nvs_close(handle);
        return result;
    }

    result = nvs_commit(handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit nvs: %s", esp_err_to_name(result));
        nvs_close(handle);
        return result;
    }

    nvs_close(handle);
    return ESP_OK;
}

uint32_t NVSManager::getManualLockoutDurationMs()
{
    uint32_t duration_ms;
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return 7200000; // Default 2 hours
    }

    result = nvs_get_u32(handle, "man_lock_dur", &duration_ms);
    nvs_close(handle);

    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        return 7200000; // Default 2 hours
    }
    else if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get manual lockout duration: %s", esp_err_to_name(result));
        return 7200000; // Default 2 hours on error
    }

    return duration_ms;
}

// Kalman filter parameter methods
bool NVSManager::setKalmanProcessNoiseVariance(float variance)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_set_u32(handle, "kf_Q", (uint32_t)(variance * 1000.0f));
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set Kalman process noise variance: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    result = nvs_commit(handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit nvs: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    return true;
}

bool NVSManager::setKalmanMeasurementNoiseVariance(float variance)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_set_u32(handle, "kf_R", (uint32_t)(variance * 1000.0f));
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set Kalman measurement noise variance: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    result = nvs_commit(handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit nvs: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    return true;
}

bool NVSManager::setKalmanStallDetectionMultiplier(float multiplier)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_set_u32(handle, "kf_k", (uint32_t)(multiplier * 1000.0f));
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set Kalman stall detection multiplier: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    result = nvs_commit(handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit nvs: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    return true;
}

float NVSManager::getKalmanProcessNoiseVariance()
{
    float variance;
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return 0.01f; // Default value
    }

    uint32_t value;
    result = nvs_get_u32(handle, "kf_Q", &value);
    if (result == ESP_OK)
    {
        variance = (float)value / 1000.0f;
    }
    nvs_close(handle);

    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        return 0.01f; // Default value
    }
    else if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get Kalman process noise variance: %s", esp_err_to_name(result));
        return 0.01f; // Default value on error
    }

    return variance;
}

float NVSManager::getKalmanMeasurementNoiseVariance()
{
    float variance;
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return 1.0f; // Default value
    }

    uint32_t value;
    result = nvs_get_u32(handle, "kf_R", &value);
    if (result == ESP_OK)
    {
        variance = (float)value / 1000.0f;
    }
    nvs_close(handle);

    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        return 1.0f; // Default value
    }
    else if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get Kalman measurement noise variance: %s", esp_err_to_name(result));
        return 1.0f; // Default value on error
    }

    return variance;
}

float NVSManager::getKalmanStallDetectionMultiplier()
{
    float multiplier;
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return 3.0f; // Default value
    }

    uint32_t value;
    result = nvs_get_u32(handle, "kf_k", &value);
    if (result == ESP_OK)
    {
        multiplier = (float)value / 1000.0f;
    }
    nvs_close(handle);

    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        return 3.0f; // Default value
    }
    else if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get Kalman stall detection multiplier: %s", esp_err_to_name(result));
        return 3.0f; // Default value on error
    }

    return multiplier;
}

// Kalman filter state methods
bool NVSManager::setKalmanStateXHat(float x_hat)
{
    // Validation: Reject unreasonable SPPR values
    // SPPR > 100 s/% means 100 seconds per 1% rise = 2.78 hours to fill 0-100%
    // This indicates a serious problem and should not be stored as "normal"
    if (x_hat > 100.0f || x_hat < 0.0f)
    {
        ESP_LOGE(TAG, "!!! REJECTED INVALID SPPR VALUE !!!");
        ESP_LOGE(TAG, "  Attempted to store: %.2f s/%%", x_hat);
        ESP_LOGE(TAG, "  Valid range: 0.0 - 100.0 s/%%");
        ESP_LOGE(TAG, "  This value indicates pump stall or sensor malfunction");
        ESP_LOGE(TAG, "  Value NOT stored to NVS");
        return false;
    }

    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_set_u32(handle, "kf_x_hat", (uint32_t)(x_hat * 1000.0f));
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set Kalman state x_hat: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    result = nvs_commit(handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit nvs: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Kalman state x_hat stored to NVS: %.2f s/%%", x_hat);
    return true;
}

bool NVSManager::setKalmanStateP(float p)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_set_u32(handle, "kf_P", (uint32_t)(p * 1000.0f));
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set Kalman state P: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    result = nvs_commit(handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit nvs: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    return true;
}

bool NVSManager::getKalmanStateXHat(float &x_hat)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    uint32_t value;
    result = nvs_get_u32(handle, "kf_x_hat", &value);
    if (result == ESP_OK)
    {
        x_hat = (float)value / 1000.0f;
        
        // Validation: Reject unreasonable loaded values
        if (x_hat > 100.0f || x_hat < 0.0f)
        {
            ESP_LOGE(TAG, "!!! REJECTED CORRUPTED SPPR VALUE FROM NVS !!!");
            ESP_LOGE(TAG, "  Loaded value: %.2f s/%%", x_hat);
            ESP_LOGE(TAG, "  Valid range: 0.0 - 100.0 s/%%");
            ESP_LOGE(TAG, "  NVS data may be corrupted");
            ESP_LOGE(TAG, "  System will use default values");
            nvs_close(handle);
            return false;
        }
        
        ESP_LOGI(TAG, "Kalman state x_hat loaded from NVS: %.2f s/%%", x_hat);
    }
    nvs_close(handle);

    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "Kalman state x_hat not found in NVS (first boot or cleared)");
        return false;
    }
    else if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get Kalman state x_hat: %s", esp_err_to_name(result));
        return false;
    }

    return true;
}

bool NVSManager::getKalmanStateP(float &p)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    uint32_t value;
    result = nvs_get_u32(handle, "kf_P", &value);
    if (result == ESP_OK)
    {
        p = (float)value / 1000.0f;
    }
    nvs_close(handle);

    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        return false;
    }
    else if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get Kalman state P: %s", esp_err_to_name(result));
        return false;
    }

    return true;
}

bool NVSManager::getMinWaterLevelToStartPump(uint8_t &level)
{
    esp_err_t result = getuint8(
        NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE,
        PARAM_MIN_WATER_LEVEL_TO_START_PUMP_KEY,
        level);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get min water level to start pump from nvs flash : %s", esp_err_to_name(result));
        return false;
    }
    return true;
}

bool NVSManager::getMaxWaterLevelToStopPump(uint8_t &level)
{
    esp_err_t result = getuint8(
        NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE,
        PARAM_MAX_WATER_LEVEL_TO_STOP_PUMP_KEY,
        level);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get max water level to stop pump from nvs flash : %s", esp_err_to_name(result));
        return false;
    }
    return true;
}

bool NVSManager::getMinWaterLevelToAllowPumpControl(uint8_t &level)
{
    esp_err_t result = getuint8(
        NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE,
        PARAM_MIN_WATER_LEVEL_TO_ALLOW_PUMP_CONTROL_KEY,
        level);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get min water level to allow pump control from nvs flash : %s", esp_err_to_name(result));
        return false;
    }
    return true;
}

bool NVSManager::setDataReceptionTimeout(uint32_t timeout)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_set_u32(handle, PARAM_DATA_RECEPTION_TIMEOUT_KEY, timeout);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set data reception timeout: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    result = nvs_commit(handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit nvs: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    return true;
}

bool NVSManager::getDataReceptionTimeout(uint32_t &timeout)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_get_u32(handle, PARAM_DATA_RECEPTION_TIMEOUT_KEY, &timeout);
    nvs_close(handle);

    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        return false;
    }
    else if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get data reception timeout: %s", esp_err_to_name(result));
        return false;
    }

    return true;
}

bool NVSManager::setLockoutActive(bool active)
{
    esp_err_t result = setuint8(
        NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE,
        "lockout_active",
        active ? 1 : 0);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set lockout active status to nvs flash : %s", esp_err_to_name(result));
        return false;
    }
    return true;
}

bool NVSManager::getLockoutActive(bool &active)
{
    uint8_t value = 0;
    esp_err_t result = getuint8(
        NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE,
        "lockout_active",
        value);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get lockout active status from nvs flash : %s", esp_err_to_name(result));
        return false;
    }

    active = (value != 0);
    return true;
}

bool NVSManager::setLockoutStartedAtMillis(uint64_t timestamp)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager needs to be initialized first");
        return false;
    }

    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_set_u64(handle, "lockout_start", timestamp);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set lockout start timestamp: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    result = nvs_commit(handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit nvs: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    return true;
}

bool NVSManager::getLockoutStartedAtMillis(uint64_t &timestamp)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager needs to be initialized first");
        return false;
    }

    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_get_u64(handle, "lockout_start", &timestamp);
    nvs_close(handle);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get lockout start timestamp: %s", esp_err_to_name(result));
        return false;
    }

    return true;
}

bool NVSManager::setLastPumpRunMetaData(const LastPumpRunMetaData &metaData)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager needs to be initialized first");
        return false;
    }

    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_set_blob(handle, "last_pump_run", &metaData, sizeof(LastPumpRunMetaData));
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set last pump run metadata: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    result = nvs_commit(handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit nvs: %s", esp_err_to_name(result));
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    return true;
}

bool NVSManager::getLastPumpRunMetaData(LastPumpRunMetaData &metaData)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager needs to be initialized first");
        return false;
    }

    nvs_handle_t handle;
    esp_err_t result = nvs_open(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open nvs namespace: %s", esp_err_to_name(result));
        return false;
    }

    size_t required_size = sizeof(LastPumpRunMetaData);
    result = nvs_get_blob(handle, "last_pump_run", &metaData, &required_size);
    nvs_close(handle);

    if (result == ESP_ERR_NVS_NOT_FOUND || result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get last pump run metadata: %s", esp_err_to_name(result));
        return false;
    }

    return true;
}

bool NVSManager::clearAllStoredParams()
{
    // This method can be implemented to clear all stored parameters if needed
    // For now, return true as a placeholder
    ESP_LOGW(TAG, "clearAllStoredParams() called but not fully implemented");
    return true;
}

bool NVSManager::readBool(const char *key, bool defaultValue)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager needs to be initialized first");
        return defaultValue;
    }

    if (key == NULL)
    {
        ESP_LOGE(TAG, "Invalid key: NULL pointer");
        return defaultValue;
    }

    uint8_t value = 0;
    esp_err_t result = getuint8(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, key, value);

    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGD(TAG, "Key '%s' not found in NVS, using default value: %s", key, defaultValue ? "true" : "false");
        return defaultValue;
    }
    else if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read bool key '%s' from NVS: %s, using default value", key, esp_err_to_name(result));
        return defaultValue;
    }

    return (value != 0);
}

esp_err_t NVSManager::writeBool(const char *key, bool value)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager needs to be initialized first");
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }

    if (key == NULL)
    {
        ESP_LOGE(TAG, "Invalid key: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = setuint8(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, key, value ? 1 : 0);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write bool key '%s' to NVS: %s", key, esp_err_to_name(result));
        return result;
    }

    ESP_LOGI(TAG, "Successfully wrote bool key '%s' = %s to NVS", key, value ? "true" : "false");
    return ESP_OK;
}

uint8_t NVSManager::readUint8(const char *key, uint8_t defaultValue)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager needs to be initialized first");
        return defaultValue;
    }

    if (key == NULL)
    {
        ESP_LOGE(TAG, "Invalid key: NULL pointer");
        return defaultValue;
    }

    uint8_t value = 0;
    esp_err_t result = getuint8(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, key, value);

    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGD(TAG, "Key '%s' not found in NVS, using default value: %u", key, defaultValue);
        return defaultValue;
    }
    else if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read uint8 key '%s' from NVS: %s, using default value", key, esp_err_to_name(result));
        return defaultValue;
    }

    return value;
}

esp_err_t NVSManager::writeUint8(const char *key, uint8_t value)
{
    if (this->nvsInitSuccess == false)
    {
        ESP_LOGE(TAG, "NVS Manager needs to be initialized first");
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }

    if (key == NULL)
    {
        ESP_LOGE(TAG, "Invalid key: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = setuint8(NVS_RECEIVER_DEVICE_PARAMS_NAMESPACE, key, value);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write uint8 key '%s' to NVS: %s", key, esp_err_to_name(result));
        return result;
    }

    ESP_LOGW(TAG, "Wrote uint8 key '%s' = %u to NVS", key, value);
    return ESP_OK;
}
