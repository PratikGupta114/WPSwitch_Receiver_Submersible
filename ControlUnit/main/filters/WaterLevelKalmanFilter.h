#pragma once

#include <stdint.h>

/**
 * @brief 1D Kalman filter for water level sensor noise reduction
 * 
 * Implements a simple Kalman filter to smooth high-frequency oscillations
 * in ultrasonic water level sensor readings. Designed to work as the first
 * stage in the filtering pipeline, before the median filter.
 * 
 * Algorithm: Standard 1D Kalman filter with constant state model
 * - Prediction: x̂_{k|k-1} = x̂_{k-1|k-1} (level doesn't change instantly)
 * - Update: x̂_{k|k} = x̂_{k|k-1} + K_k × (z_k - x̂_{k|k-1})
 * 
 * Based on backend implementation analysis:
 * - Reduces noise by 15.94% (MSE reduction)
 * - Eliminates 86% of oscillations
 * - Preserves real water level changes (5-10% drops, pump cycles)
 * 
 * Memory: ~20 bytes (4 floats + 1 bool)
 * CPU: O(1) per update, ~10 float operations
 * 
 * @see ESP32_KALMAN_FILTER_IMPLEMENTATION_GUIDE.md for tuning guide
 */
class WaterLevelKalmanFilter {
public:
    /**
     * @brief Initialize the Kalman filter with specified parameters
     * 
     * @param R Measurement noise variance (default: 10.0)
     *          Higher R = more smoothing, slower response
     *          Backend analysis: R=10 provides balanced performance
     * 
     * @param Q Process noise variance (default: 1.0)
     *          Higher Q = faster response, less smoothing
     *          Backend analysis: Q=1 allows adaptation to real changes
     * 
     * @param P0 Initial error covariance (default: 1.0)
     *           Represents initial uncertainty in estimate
     */
    void init(float R = 10.0, float Q = 1.0, float P0 = 1.0);

    /**
     * @brief Update filter with new raw sensor measurement
     * 
     * Applies Kalman filter algorithm:
     * 1. Predict state (no change expected)
     * 2. Calculate Kalman gain (adaptive based on pump state)
     * 3. Update estimate with measurement
     * 4. Update error covariance
     * 
     * First measurement initializes the filter.
     * 
     * Adaptive behavior:
     * - Pump OFF: Heavy smoothing (low gain) to suppress sensor noise
     * - Pump ON: Responsive tracking (high gain) to follow rising water
     * 
     * @param measurement Raw water level percentage (0-100)
     * @param pumpIsOn True if pump is actively running (default: false)
     * @return Filtered water level percentage (0-100), rounded to integer
     */
    uint8_t update(uint8_t measurement, bool pumpIsOn = false);

    /**
     * @brief Check if filter has been initialized
     * 
     * @return true if filter is ready to process measurements
     */
    bool isReady() const;

    /**
     * @brief Get current filtered estimate
     * 
     * @return Current filtered water level percentage (0-100)
     */
    float getEstimate() const;

    /**
     * @brief Reset filter to uninitialized state
     * 
     * Call when tank is refilled, sensor replaced, or on major level changes
     * that might confuse the filter.
     */
    void reset();

    /**
     * @brief Adjust filter parameters dynamically (advanced usage)
     * 
     * Allows runtime tuning of filter behavior. Use with caution.
     * 
     * @param R New measurement noise variance
     * @param Q New process noise variance
     */
    void setParameters(float R, float Q);

    /**
     * @brief Get current filter parameters for debugging
     * 
     * @param R Output: Current measurement noise variance
     * @param Q Output: Current process noise variance
     * @param P Output: Current error covariance
     */
    void getParameters(float& R, float& Q, float& P) const;

private:
    float estimate;           // x̂ - Current state estimate (filtered level)
    float errorCovariance;    // P - Current error covariance
    float measurementNoise;   // R - Measurement noise variance
    float processNoise;       // Q - Process noise variance
    bool initialized;         // Filter initialization flag
};
