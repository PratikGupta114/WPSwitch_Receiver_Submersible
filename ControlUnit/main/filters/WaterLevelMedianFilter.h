#pragma once

#include <stdint.h>

/**
 * @brief Simple 7-sample median filter for water level noise rejection
 * 
 * Rejects burst oscillations (e.g., 90% ↔ 91% ↔ 90%) and single-sample
 * glitches by requiring majority consensus in the sliding window.
 * Detects all real 1% changes with 2.0-3.5 second confirmation time.
 * 
 * Memory: 28 bytes (7 samples × 4 bytes)
 * CPU: O(n log n) sort on 7 elements = ~14 comparisons per update
 */
class WaterLevelMedianFilter {
public:
    /**
     * @brief Initialize the median filter
     */
    void init();

    /**
     * @brief Update filter with new raw sensor reading
     * 
     * @param raw_level Raw water level percentage (0-100)
     */
    void update(uint8_t raw_level);

    /**
     * @brief Get the filtered (median) water level
     * 
     * @return Filtered water level percentage (0-100)
     */
    uint8_t getFilteredLevel() const;

    /**
     * @brief Check if filter has enough samples for reliable output
     * 
     * @return true if at least 4 samples collected
     */
    bool isReady() const;

    /**
     * @brief Get the current window contents (for debugging)
     * 
     * @param window Output array (must be at least 7 elements)
     * @return Number of valid samples in window
     */
    uint8_t getWindow(uint8_t* window) const;

private:
    static const uint8_t WINDOW_SIZE = 7;
    
    uint8_t buffer[WINDOW_SIZE];  // Circular buffer for last 7 samples
    uint8_t index;                // Current write position in buffer
    uint8_t count;                // Number of samples collected (0-7)
    bool initialized;             // Filter initialization flag

    /**
     * @brief Calculate median of current window
     * 
     * Uses simple insertion sort (optimal for small arrays)
     * 
     * @return Median value
     */
    uint8_t calculateMedian() const;
};
