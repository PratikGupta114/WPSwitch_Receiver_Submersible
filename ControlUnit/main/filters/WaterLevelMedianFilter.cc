#include "filters/WaterLevelMedianFilter.h"
#include "esp_log.h"
#include <algorithm>

static const char *TAG = "WaterLevelMedianFilter";

void WaterLevelMedianFilter::init() {
    index = 0;
    count = 0;
    initialized = true;
    
    // Clear buffer
    for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
        buffer[i] = 0;
    }
    
    ESP_LOGI(TAG, "Median filter initialized (window size: %d)", WINDOW_SIZE);
}

void WaterLevelMedianFilter::update(uint8_t raw_level) {
    if (!initialized) {
        ESP_LOGW(TAG, "Update called before init()");
        return;
    }
    
    // Add new sample to circular buffer
    buffer[index] = raw_level;
    index = (index + 1) % WINDOW_SIZE;
    
    // Track number of samples (up to WINDOW_SIZE)
    if (count < WINDOW_SIZE) {
        count++;
    }
}

uint8_t WaterLevelMedianFilter::getFilteredLevel() const {
    if (!initialized || count == 0) {
        return 0;
    }
    
    return calculateMedian();
}

bool WaterLevelMedianFilter::isReady() const {
    // Need at least 4 samples for reliable median (majority of 7-sample window)
    return initialized && count >= 4;
}

uint8_t WaterLevelMedianFilter::getWindow(uint8_t* window) const {
    if (!window) {
        return 0;
    }
    
    // Copy current window contents
    for (uint8_t i = 0; i < count; i++) {
        window[i] = buffer[i];
    }
    
    return count;
}

uint8_t WaterLevelMedianFilter::calculateMedian() const {
    // Create temporary sorted array
    uint8_t sorted[WINDOW_SIZE];
    
    // Copy valid samples
    for (uint8_t i = 0; i < count; i++) {
        sorted[i] = buffer[i];
    }
    
    // Simple insertion sort (optimal for small arrays)
    for (uint8_t i = 1; i < count; i++) {
        uint8_t key = sorted[i];
        int8_t j = i - 1;
        
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    
    // Return median (middle element)
    return sorted[count / 2];
}
