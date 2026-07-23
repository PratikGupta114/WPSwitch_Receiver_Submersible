/**
 * @file test_critical_protection_suite.cpp
 * @brief Host unit test suite for Submersible Control Unit firmware safety algorithms
 * 
 * Verifies:
 * 1. Two-variable protected window calculation (effectivePriming + gracePeriod)
 * 2. Maximum protected window clamping (CONFIG_PUMP_MAX_PROTECTED_WINDOW_SEC)
 * 3. EMA early exit logic for submersible pump startup
 * 4. RF sequence number staleness detection (RF packet loss resilience)
 * 5. NVS priming period bounds validation (0s <= seconds <= 600s)
 * 6. Submersible OTA URL query parameter formatting & null safety
 */

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cassert>
#include <cstring>
#include <cstdlib>

// Mock Logging Macros for Host Execution
#define ESP_LOGI(tag, fmt, ...) printf("[INFO][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[WARN][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERR ][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) // Silent debug logs in host test

// Mock Configuration Parameters
#define CONFIG_PUMP_STALL_GRACE_PERIOD_SEC 60
#define CONFIG_PUMP_PRIMING_SAFETY_MULTIPLIER 15
#define CONFIG_PUMP_MAX_PROTECTED_WINDOW_SEC 240

// Test statistics
static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(condition, message) \
    do { \
        if (condition) { \
            printf("  ✓ PASS: %s\n", message); \
            g_tests_passed++; \
        } else { \
            printf("  ✗ FAIL: %s (Line %d)\n", message, __LINE__); \
            g_tests_failed++; \
        } \
    } while(0)

// -----------------------------------------------------------------------------
// 1. Protected Window Math & Clamping Logic Test
// -----------------------------------------------------------------------------
float compute_protected_window(float primingPeriodSeconds, uint8_t pumpStartWaterLevel)
{
    float effectivePriming = primingPeriodSeconds * (CONFIG_PUMP_PRIMING_SAFETY_MULTIPLIER / 10.0f);
    float protectedWindow;
    if (pumpStartWaterLevel == 0)
    {
        protectedWindow = effectivePriming + CONFIG_PUMP_STALL_GRACE_PERIOD_SEC;
    }
    else
    {
        protectedWindow = effectivePriming;
    }

    if (protectedWindow > CONFIG_PUMP_MAX_PROTECTED_WINDOW_SEC)
    {
        protectedWindow = CONFIG_PUMP_MAX_PROTECTED_WINDOW_SEC;
    }
    return protectedWindow;
}

void test_protected_window_math()
{
    printf("\n=== Submersible Test Suite 1: Protected Window Calculation & Clamping ===\n");

    // Case 1: Uncalibrated (0s stored priming), start level 0% -> 60s grace
    float w1 = compute_protected_window(0.0f, 0);
    TEST_ASSERT(std::abs(w1 - 60.0f) < 0.001f, "Uncalibrated 0% start level yields 60s grace period");

    // Case 2: Uncalibrated (0s stored priming), start level 15% -> 0s window
    float w2 = compute_protected_window(0.0f, 15);
    TEST_ASSERT(std::abs(w2 - 0.0f) < 0.001f, "Uncalibrated non-zero start level yields 0s window");

    // Case 3: Calibrated 40s priming, start level 0% -> 40*1.5 + 60 = 120s
    float w3 = compute_protected_window(40.0f, 0);
    TEST_ASSERT(std::abs(w3 - 120.0f) < 0.001f, "Calibrated 40s priming at 0% start level yields 120s window");

    // Case 4: Calibrated 40s priming, start level 20% -> 40*1.5 = 60s
    float w4 = compute_protected_window(40.0f, 20);
    TEST_ASSERT(std::abs(w4 - 60.0f) < 0.001f, "Calibrated 40s priming at 20% start level yields 60s window");

    // Case 5: Large priming period 200s (effective 300s + 60s = 360s) -> Clamped to 240s
    float w5 = compute_protected_window(200.0f, 0);
    TEST_ASSERT(std::abs(w5 - 240.0f) < 0.001f, "Large priming period is strictly clamped to max ceiling (240s)");
}

// -----------------------------------------------------------------------------
// 2. EMA Early Exit Logic Test
// -----------------------------------------------------------------------------
bool check_ema_early_exit(float current_ema_level, uint8_t start_level)
{
    return (current_ema_level >= (float)start_level + 2.0f);
}

float update_ema(float raw_water_level, float previous_ema, float alpha = 0.3f)
{
    return (alpha * raw_water_level) + ((1.0f - alpha) * previous_ema);
}

void test_ema_early_exit()
{
    printf("\n=== Submersible Test Suite 2: EMA Early Exit Logic ===\n");

    uint8_t startLevel = 0;
    float ema = 0.0f;

    // Single raw reading spike to 2% (EMA = 0.6%) -> Should NOT trigger early exit
    ema = update_ema(2.0f, ema);
    TEST_ASSERT(!check_ema_early_exit(ema, startLevel), "Single reading noise spike (EMA=0.6%) does not trigger early exit");

    // Second reading of 2% (EMA = 1.02%) -> Should NOT trigger early exit
    ema = update_ema(2.0f, ema);
    TEST_ASSERT(!check_ema_early_exit(ema, startLevel), "Second reading (EMA=1.02%) does not trigger early exit");

    // Sustained rise: 3% readings
    ema = update_ema(3.0f, ema); // 1.61%
    ema = update_ema(3.0f, ema); // 2.03%
    TEST_ASSERT(check_ema_early_exit(ema, startLevel), "Sustained water rise (EMA >= 2.0%) triggers early exit to MONITORING");
}

// -----------------------------------------------------------------------------
// 3. RF Sequence Number Staleness Detection Test
// -----------------------------------------------------------------------------
struct StallConfirmationResult
{
    bool is_stall_confirmed;
    bool is_rf_dropout;
    uint8_t valid_readings;
    uint8_t increase_count;
};

StallConfirmationResult evaluate_stall_confirmation(
    uint8_t initial_level,
    uint64_t initial_seq_num,
    const uint8_t readings[3],
    const uint64_t seq_nums[3])
{
    StallConfirmationResult res = {};
    res.valid_readings = 0;
    res.increase_count = 0;

    for (int i = 0; i < 3; i++)
    {
        uint64_t prevSeq = (i == 0) ? initial_seq_num : seq_nums[i - 1];
        bool isFresh = (seq_nums[i] != prevSeq);

        if (isFresh)
        {
            res.valid_readings++;
            if (readings[i] > initial_level)
            {
                res.increase_count++;
            }
        }
    }

    if (res.valid_readings < 2)
    {
        res.is_rf_dropout = true;
        res.is_stall_confirmed = false;
    }
    else if (res.increase_count * 2 >= res.valid_readings)
    {
        res.is_rf_dropout = false;
        res.is_stall_confirmed = false; // Level rising, not stalled
    }
    else
    {
        res.is_rf_dropout = false;
        res.is_stall_confirmed = true; // Confirmed stall
    }

    return res;
}

void test_rf_sequence_staleness()
{
    printf("\n=== Submersible Test Suite 3: RF Sequence Number Staleness & Dropout Detection ===\n");

    uint8_t initialLevel = 10;
    uint64_t initialSeq = 500;

    // Case A: 3 fresh readings, no water level increase -> Stall CONFIRMED
    uint8_t readingsA[3] = {10, 10, 10};
    uint64_t seqsA[3] = {501, 502, 503};
    StallConfirmationResult resA = evaluate_stall_confirmation(initialLevel, initialSeq, readingsA, seqsA);
    TEST_ASSERT(resA.is_stall_confirmed == true && resA.is_rf_dropout == false,
                "Fresh readings with no water rise correctly confirms submersible pump stall");

    // Case B: RF dropout (duplicate sequence numbers 501, 501, 501) -> RF Dropout detected, stall NOT confirmed
    uint8_t readingsB[3] = {10, 10, 10};
    uint64_t seqsB[3] = {501, 501, 501};
    StallConfirmationResult resB = evaluate_stall_confirmation(initialLevel, initialSeq, readingsB, seqsB);
    TEST_ASSERT(resB.is_rf_dropout == true && resB.is_stall_confirmed == false,
                "Duplicate sequence numbers trigger RF dropout retry instead of false stall stop");

    // Case C: Fresh readings with water level rise -> Stall NOT confirmed (returns to MONITORING)
    uint8_t readingsC[3] = {12, 13, 13};
    uint64_t seqsC[3] = {501, 502, 503};
    StallConfirmationResult resC = evaluate_stall_confirmation(initialLevel, initialSeq, readingsC, seqsC);
    TEST_ASSERT(resC.is_stall_confirmed == false && resC.is_rf_dropout == false,
                "Water level rise during confirmation returns pump to MONITORING");
}

// -----------------------------------------------------------------------------
// 4. NVS Priming Period Bounds Validation Test
// -----------------------------------------------------------------------------
bool validate_priming_period_value(float seconds)
{
    if (seconds < 0.0f || seconds > 600.0f)
    {
        return false;
    }
    return true;
}

void test_nvs_bounds_validation()
{
    printf("\n=== Submersible Test Suite 4: NVS Priming Period Bounds Validation ===\n");

    TEST_ASSERT(validate_priming_period_value(0.0f) == true, "0.0s priming period is valid");
    TEST_ASSERT(validate_priming_period_value(35.2f) == true, "35.2s priming period is valid");
    TEST_ASSERT(validate_priming_period_value(600.0f) == true, "600.0s (10 min) priming period is valid");

    TEST_ASSERT(validate_priming_period_value(-0.1f) == false, "Negative priming period (-0.1s) is rejected");
    TEST_ASSERT(validate_priming_period_value(600.1f) == false, "Excessive priming period (600.1s) is rejected");
    TEST_ASSERT(validate_priming_period_value(-50.0f) == false, "Invalid negative priming period (-50s) is rejected");
}

// -----------------------------------------------------------------------------
// 5. OTA URL Query Parameter Formatting Test
// -----------------------------------------------------------------------------
std::string build_ota_url(const char *baseUrl, const char *projectName, const char *deviceID)
{
    const char *proj = (projectName != nullptr && projectName[0] != '\0') ? projectName : "wpswitch-submersible";
    char sep = (strchr(baseUrl, '?') != nullptr) ? '&' : '?';

    char buffer[512];
    if (deviceID != nullptr && strlen(deviceID) > 0)
    {
        snprintf(buffer, sizeof(buffer), "%s%cprojectName=%s&deviceID=%s", baseUrl, sep, proj, deviceID);
    }
    else
    {
        snprintf(buffer, sizeof(buffer), "%s%cprojectName=%s", baseUrl, sep, proj);
    }
    return std::string(buffer);
}

void test_ota_url_formatting()
{
    printf("\n=== Submersible Test Suite 5: OTA URL Formatting & Parameter Escaping ===\n");

    // Case 1: Standard URL without query parameters + deviceID
    std::string url1 = build_ota_url("https://api.example.com/ota", "wpswitch-submersible-rev1", "SUB_8877");
    TEST_ASSERT(url1 == "https://api.example.com/ota?projectName=wpswitch-submersible-rev1&deviceID=SUB_8877",
                "Formatted URL correctly uses '?' separator for first query param");

    // Case 2: URL with existing query parameters + deviceID
    std::string url2 = build_ota_url("https://api.example.com/ota?version=v1", "wpswitch-submersible-prototype", "SUB_8877");
    TEST_ASSERT(url2 == "https://api.example.com/ota?version=v1&projectName=wpswitch-submersible-prototype&deviceID=SUB_8877",
                "Formatted URL correctly uses '&' separator when '?' exists");

    // Case 3: Null deviceID (unauthenticated check)
    std::string url3 = build_ota_url("https://api.example.com/ota", "wpswitch-submersible-rev1", nullptr);
    TEST_ASSERT(url3 == "https://api.example.com/ota?projectName=wpswitch-submersible-rev1",
                "Null deviceID formats projectName parameter without deviceID key");

    // Case 4: Null projectName fallback
    std::string url4 = build_ota_url("https://api.example.com/ota", nullptr, "SUB_123");
    TEST_ASSERT(url4 == "https://api.example.com/ota?projectName=wpswitch-submersible&deviceID=SUB_123",
                "Null projectName falls back safely to 'wpswitch-submersible'");
}

// -----------------------------------------------------------------------------
// Main Test Entrypoint
// -----------------------------------------------------------------------------
int main()
{
    printf("=================================================================\n");
    printf("  SUBMERSIBLE CONTROL UNIT CRITICAL PROTECTION HOST TEST SUITE  \n");
    printf("=================================================================\n");

    test_protected_window_math();
    test_ema_early_exit();
    test_rf_sequence_staleness();
    test_nvs_bounds_validation();
    test_ota_url_formatting();

    printf("\n-----------------------------------------------------------------\n");
    printf(" TEST RESULTS SUMMARY: %d Passed, %d Failed\n", g_tests_passed, g_tests_failed);
    printf("-----------------------------------------------------------------\n");

    return (g_tests_failed == 0) ? 0 : 1;
}
