#include "filters/WaterLevelKalmanFilter.h"
#include "esp_log.h"
#include <cmath>

static const char *TAG = "WaterLevelKalmanFilter";

void WaterLevelKalmanFilter::init(float R, float Q, float P0) {
    measurementNoise = R;
    processNoise = Q;
    errorCovariance = P0;
    estimate = 0.0;
    initialized = false;
    
    // Calculate expected steady-state gain for verification
    // At steady state: P² + PQ - QR = 0, so P = (-Q + sqrt(Q² + 4QR)) / 2
    // Gain: K = (P + Q) / (P + Q + R)
    float expectedP = (-Q + sqrtf(Q*Q + 4*Q*R)) / 2.0f;
    float expectedGain = (expectedP + Q) / (expectedP + Q + R);
    
    ESP_LOGI(TAG, "Kalman filter initialized: R=%.1f, Q=%.1f, P0=%.1f", R, Q, P0);
    ESP_LOGI(TAG, "Expected steady-state: P=%.3f, gain=%.3f", expectedP, expectedGain);
}

uint8_t WaterLevelKalmanFilter::update(uint8_t measurement, bool pumpIsOn) {
    // Initialize on first measurement
    if (!initialized) {
        estimate = static_cast<float>(measurement);
        initialized = true;
        ESP_LOGI(TAG, "Filter initialized with first measurement: %d%%", measurement);
        return measurement;
    }
    
    // ========== ADAPTIVE PARAMETERS BASED ON PUMP STATE ==========
    // User observation: Current gain (0.155 from R=35) works well for pump ON
    // Need MUCH MORE smoothing when pump OFF to suppress ±1% oscillations
    // during gradual water level decrease
    
    float adaptiveR = measurementNoise;  // Default R from init
    
    if (pumpIsOn) {
        // Pump ON: Keep proven gain that works well for rising water
        // R = 35.0 gives gain ≈ 0.155 (84.5% prediction, 15.5% measurement)
        adaptiveR = 35.0;
    } else {
        // Pump OFF: Very heavy smoothing to eliminate ±1% oscillations
        // R = 150.0 gives gain ≈ 0.08 (92% prediction, 8% measurement)
        adaptiveR = 150.0;
    }
    
    // ========== PREDICTION STEP ==========
    // State prediction: x̂_{k|k-1} = x̂_{k-1|k-1}
    // (Water level doesn't change without control input)
    float predictedEstimate = estimate;
    
    // Covariance prediction: P_{k|k-1} = P_{k-1|k-1} + Q
    // (Uncertainty increases slightly over time)
    float predictedCovariance = errorCovariance + processNoise;
    
    ESP_LOGD(TAG, "Prediction: estimate=%.2f%%, covariance=%.4f, pump=%s", 
             predictedEstimate, predictedCovariance, pumpIsOn ? "ON" : "OFF");
    
    // ========== UPDATE STEP ==========
    // Calculate Kalman gain: K_k = P_{k|k-1} / (P_{k|k-1} + R)
    // (How much to trust the new measurement vs prediction)
    float kalmanGain = predictedCovariance / (predictedCovariance + adaptiveR);
    
    // Update estimate: x̂_{k|k} = x̂_{k|k-1} + K_k × (z_k - x̂_{k|k-1})
    // (Blend prediction with measurement)
    float innovation = static_cast<float>(measurement) - predictedEstimate;
    estimate = predictedEstimate + kalmanGain * innovation;
    
    // Update error covariance: P_{k|k} = (1 - K_k) × P_{k|k-1}
    // (Update confidence in estimate)
    errorCovariance = (1.0 - kalmanGain) * predictedCovariance;
    
    ESP_LOGD(TAG, "Update: measurement=%d%%, innovation=%.2f, gain=%.4f (R=%.1f), estimate=%.2f%%, covariance=%.4f",
             measurement, innovation, kalmanGain, adaptiveR, estimate, errorCovariance);
    
    // Return rounded value (integer percentage for compatibility)
    uint8_t filtered = static_cast<uint8_t>(round(estimate));
    
    // Clamp to valid range (0-100%)
    if (filtered > 100) {
        ESP_LOGW(TAG, "Filtered value %d%% exceeds 100%%, clamping to 100%%", filtered);
        filtered = 100;
    }
    
    // Log significant changes at INFO level for monitoring
    static uint8_t lastLoggedFiltered = 255;  // Invalid initial value
    if (lastLoggedFiltered == 255 || abs(filtered - lastLoggedFiltered) >= 1) {
        ESP_LOGI(TAG, "Filter output: raw=%d%% → filtered=%d%% (innovation=%.2f, gain=%.3f, pump=%s)",
                 measurement, filtered, innovation, kalmanGain, pumpIsOn ? "ON" : "OFF");
        lastLoggedFiltered = filtered;
    }
    
    return filtered;
}

bool WaterLevelKalmanFilter::isReady() const {
    return initialized;
}

float WaterLevelKalmanFilter::getEstimate() const {
    return estimate;
}

void WaterLevelKalmanFilter::reset() {
    initialized = false;
    estimate = 0.0;
    errorCovariance = 1.0;
    ESP_LOGI(TAG, "Filter reset");
}

void WaterLevelKalmanFilter::setParameters(float R, float Q) {
    measurementNoise = R;
    processNoise = Q;
    ESP_LOGI(TAG, "Parameters updated: R=%.1f, Q=%.1f", R, Q);
}

void WaterLevelKalmanFilter::getParameters(float& R, float& Q, float& P) const {
    R = measurementNoise;
    Q = processNoise;
    P = errorCovariance;
}
