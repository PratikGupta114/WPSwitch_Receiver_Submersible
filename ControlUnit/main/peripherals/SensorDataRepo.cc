
#include "SensorDataRepo.h"
#include "esp_log.h"
#include "string.h"
#include "esp_mac.h"
#include "esp_timer.h"  // SAFETY FIX: For timestamp tracking

#define TAG "SensorDataRepo"

#define VALID_RANGE_HUMIDITY(x, t) (x >= MAX_HUMIDITY_LEVEL) ? MAX_HUMIDITY_LEVEL : (x <= MIN_HUMIDITY_LEVEL) ? MIN_HUMIDITY_LEVEL \
                                                                                                              : (t)x
#define VALID_RANGE_TEMPERATURE(x, t) (x >= MAX_TEMPERATURE_LEVEL) ? MAX_TEMPERATURE_LEVEL : (x <= MIN_TEMPERATURE_LEVEL) ? MIN_TEMPERATURE_LEVEL \
                                                                                                                          : (t)x
#define VALID_RANGE_WATER_LEVEL(x, t) (x >= MAX_WATER_LEVEL) ? MAX_WATER_LEVEL : (x <= MIN_WATER_LEVEL) ? MIN_WATER_LEVEL \
                                                                                                        : (t)x

void SensorDataRepo::updateSensorData(SensorData *newSensorData) {
    if (!newSensorData) return; // Basic validation

    xSemaphoreTake(this->dataAccessMutex, portMAX_DELAY);

    bool dataChanged = false;
    if (this->waterLevel != newSensorData->waterLevel ||
        this->temperature != newSensorData->temperatureCelsius ||
        this->humidity != newSensorData->humidity ||
        this->sequenceNumber != newSensorData->sequenceNumber) { // Also consider sequenceNumber for change
        dataChanged = true;
        this->sequenceNumber = newSensorData->sequenceNumber;
        this->temperature = newSensorData->temperatureCelsius;
        this->humidity = newSensorData->humidity;
        this->waterLevel = newSensorData->waterLevel;
        
        // SAFETY FIX: Update timestamp when data changes
        this->lastUpdateTimestamp = esp_timer_get_time();
    }

    if (dataChanged && sensorDataUpdatedListener) {
        SensorData updatedDataSnapshot; // Create a snapshot to pass to the callback
        updatedDataSnapshot.dataType = TYPE_SENSOR_DATA; // Assuming this is constant
        updatedDataSnapshot.sequenceNumber = this->sequenceNumber;
        updatedDataSnapshot.waterLevel = this->waterLevel;
        updatedDataSnapshot.temperatureCelsius = this->temperature;
        updatedDataSnapshot.humidity = this->humidity;
        // Populate deviceID and checksum if the callback needs a full, valid SensorData object
        esp_err_t ret = esp_read_mac(updatedDataSnapshot.deviceID, ESP_MAC_WIFI_STA);
        if (ret != ESP_OK) {
             ESP_LOGE(TAG, "Failed to get deviceID in updateSensorData callback context | %s", esp_err_to_name(ret));
        }
        updatedDataSnapshot.checksum = getChecksum(&updatedDataSnapshot); // Pass by pointer

        sensorDataUpdatedListener(updatedDataSnapshot);
    }

    xSemaphoreGive(this->dataAccessMutex);
}

uint8_t SensorDataRepo::getChecksum(const SensorData* s_data_ptr) {
    if (!s_data_ptr) return 0; // Or some error indicator

    uint8_t checksum = 0xFE;
    const uint8_t* p_byte = nullptr;

    checksum ^= s_data_ptr->dataType;

    p_byte = reinterpret_cast<const uint8_t*>(&s_data_ptr->sequenceNumber);
    for (size_t i = 0; i < sizeof(s_data_ptr->sequenceNumber); ++i) checksum ^= p_byte[i];

    checksum ^= s_data_ptr->waterLevel;
    checksum ^= s_data_ptr->temperatureCelsius;
    checksum ^= s_data_ptr->humidity;

    for (size_t i = 0; i < sizeof(s_data_ptr->deviceID); ++i) checksum ^= s_data_ptr->deviceID[i];
    // Note: The original checksum calculation in main.cc's getSensorData was on a buffer excluding the checksum field itself.
    // This implementation iterates over the fields that constitute the data to be checksummed.
    return checksum;
}

uint8_t SensorDataRepo::getChecksum(char *buffer, size_t size)
{
    uint8_t checksum = 0xFE;
    for (int i = 0; i < size; i++)
        checksum ^= buffer[i];
    return checksum;
}

DataFetchResult SensorDataRepo::getSensorData(SensorData *&currentSensorData) {
    // The original check was `if (sensorData != NULL)` which meant if the *caller's pointer* was already pointing to something.
    // The intention here is that this function allocates.
    if (currentSensorData != NULL) { 
        ESP_LOGE(TAG, "Output parameter currentSensorData must be null for this function to allocate it.");
        return ERROR_INVALID_ARG;
    }

    currentSensorData = (SensorData *)malloc(sizeof(SensorData));
    if (!currentSensorData) {
        ESP_LOGE(TAG, "Failed to allocate memory for currentSensorData");
        return ERROR_INVALID_ARG; // Or a more specific error like ERROR_NO_MEM
    }

    // CRITICAL FIX: Use timeout instead of portMAX_DELAY to prevent task watchdog timeout
    // If mutex is held for >100ms, something is wrong - fail gracefully and allow IDLE task to run
    if (xSemaphoreTake(this->dataAccessMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Mutex acquisition timeout in getSensorData - freeing allocated memory");
        free(currentSensorData);
        currentSensorData = NULL;
        return ERROR_TIMEOUT;
    }
    
    currentSensorData->dataType = TYPE_SENSOR_DATA; // Assuming
    currentSensorData->humidity = this->humidity;
    currentSensorData->waterLevel = this->waterLevel;
    currentSensorData->temperatureCelsius = this->temperature;
    currentSensorData->sequenceNumber = this->sequenceNumber;
    esp_err_t ret = esp_read_mac(currentSensorData->deviceID, ESP_MAC_WIFI_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get deviceID in getSensorData | %s", esp_err_to_name(ret));
        // Potentially zero out deviceID or handle error
        memset(currentSensorData->deviceID, 0, sizeof(currentSensorData->deviceID));
    }
    xSemaphoreGive(this->dataAccessMutex); // Release mutex before checksum on copied data

    currentSensorData->checksum = getChecksum(currentSensorData); // Pass by pointer

    return DATA_FETCH_SUCCESS;
}

SensorDataRepo::SensorDataRepo() {
    this->dataAccessMutex = xSemaphoreCreateMutex();
    if (this->dataAccessMutex == NULL) {
        ESP_LOGE(TAG, "FATAL: Failed to create sensor data mutex - thread safety compromised!");
        ESP_ERROR_CHECK(ESP_FAIL);  // Halt system - cannot continue safely without mutex
    }
    this->sequenceNumber = 0;
    this->lastUpdateTimestamp = 0;  // SAFETY FIX: Initialize to 0 (no data received yet)
}

// SAFETY FIX: Check if sensor data is fresh (within specified milliseconds)
bool SensorDataRepo::isDataFresh(uint32_t maxAgeMillis) const {
    if (this->lastUpdateTimestamp == 0) {
        // No data has been received yet
        return false;
    }
    
    uint64_t currentTime = esp_timer_get_time();
    uint64_t ageMillis = (currentTime - this->lastUpdateTimestamp) / 1000;
    
    return ageMillis <= maxAgeMillis;
}

// SAFETY FIX: Get age of current sensor data in milliseconds
uint64_t SensorDataRepo::getDataAgeMillis() const {
    if (this->lastUpdateTimestamp == 0) {
        // No data has been received yet
        return 0;
    }
    
    uint64_t currentTime = esp_timer_get_time();
    return (currentTime - this->lastUpdateTimestamp) / 1000;
}

void SensorDataRepo::setOnSensorDataUpdatedListener(SensorDataUpdatedCallback callback) {
    // No mutex needed here if it's just assigning a std::function, 
    // but if there was a risk of concurrent set/call, it might be.
    // For simplicity, assume it's called during setup.
    this->sensorDataUpdatedListener = callback;
}

SensorDataRepo::~SensorDataRepo()
{
    vSemaphoreDelete(this->dataAccessMutex);
}
