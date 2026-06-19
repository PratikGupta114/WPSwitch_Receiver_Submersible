#ifndef SENSORDATAREPO_H_
#define SENSORDATAREPO_H_

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <functional> // For std::function

#include "../data/dataTypes.h"

#define MIN_HUMIDITY_LEVEL CONFIG_MIN_HUMIDITY_LEVEL
#define MAX_HUMIDITY_LEVEL CONFIG_MAX_HUMIDITY_LEVEL
#define MIN_TEMPERATURE_LEVEL CONFIG_MIN_TEMPERATURE_LEVEL
#define MAX_TEMPERATURE_LEVEL CONFIG_MAX_TEMPERATURE_LEVEL
#define MIN_WATER_LEVEL CONFIG_MIN_WATER_LEVEL
#define MAX_WATER_LEVEL CONFIG_MAX_WATER_LEVEL

using namespace std;

class SensorDataRepo
{
public:
    // Callback for when any sensor data is updated and has changed
    using SensorDataUpdatedCallback = std::function<void(const SensorData &newData)>;

    SensorDataRepo();
    ~SensorDataRepo();

    void updateSensorData(SensorData *newSensorData);              // Parameter name more descriptive
    DataFetchResult getSensorData(SensorData *&currentSensorData); // Parameter name more descriptive
    uint8_t getChecksum(const SensorData *s_data_ptr);             // Optimized: takes const SensorData*
    uint8_t getChecksum(char *buffer, size_t size);                // Optimized: takes char* and size

    void setOnSensorDataUpdatedListener(SensorDataUpdatedCallback callback);
    
    // SAFETY FIX: Check if sensor data is fresh (within specified milliseconds)
    // Returns true if data was updated within the specified time window
    bool isDataFresh(uint32_t maxAgeMillis) const;
    
    // SAFETY FIX: Get age of current sensor data in milliseconds
    // Returns 0 if no data has been received yet
    uint64_t getDataAgeMillis() const;

private:
    SemaphoreHandle_t dataAccessMutex; // Will be created as a mutex
    uint8_t temperature;
    uint8_t humidity;
    uint8_t waterLevel;
    uint64_t sequenceNumber;
    uint64_t lastUpdateTimestamp;  // SAFETY FIX: Track when data was last updated (microseconds)
    // Removed: uint64_t consumedSequenceNumber = -1;

    SensorDataUpdatedCallback sensorDataUpdatedListener = nullptr;
    // Removed unused listeners:
    // function<void(uint8_t, uint8_t)> onWaterLevelChangedListener;
    // function<void(uint8_t, uint8_t)> onTemperatureChangedListener;
    // function<void(uint8_t, uint8_t)> onHumidityChangedListener;
};

#endif