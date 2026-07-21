
#ifndef NVSMANAGER_H_
#define NVSMANAGER_H_

#define MANUAL_LOCKOUT_DURATION_MS_MIN (30 * 60 * 1000)
#define MANUAL_LOCKOUT_DURATION_MS_MAX (5 * 60 * 60 * 1000)

#include <string>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "../data/dataTypes.h"

#define MAX_SSID_LEN 32
#define MAX_PASSWORD_LEN 64

using namespace std;

typedef struct credentials
{
    char ssid[MAX_SSID_LEN];
    char password[MAX_PASSWORD_LEN];
} WiFiCredentials;

class NVSManager
{
private:
    // static constexpr char TAG[] = "NVSManager";
    // static constexpr char NVS_WIFI_STATION_CREDENTIALS_NAMESPACE[] = "stacreds";
    // static constexpr char SSID_KEY[] = "ssid";
    // static constexpr char PASSWORD_KEY[] = "passwd";

    bool nvsInitSuccess = false;
    esp_err_t setuint8(const char *namespaceName, const char *keyName, uint8_t num);
    esp_err_t getuint8(const char *namespaceName, const char *keyName, uint8_t &num);

public:
    // constructor and destructor
    NVSManager();
    ~NVSManager();

    // Initialize NVS
    bool init();

    bool addCredential(WiFiCredentials &credential);
    bool hasCredential(string ssid);
    bool deleteCredentialWithSSID(string ssid);
    bool getSavedCredentials(WiFiCredentials *&credentials, size_t &count);
    bool getCredentialsCount(size_t &count);
    bool getCredentialBySSID(string ssid, WiFiCredentials *&credential);
    bool getSavedCredentialsInJson(char *&jsonResultStringHolder);
    void printCredentials();

    bool setMinWaterLevelToStartPump(uint8_t level);
    bool setMaxWaterLevelToStopPump(uint8_t level);
    bool setMinWaterLevelToAllowPumpControl(uint8_t level);
    bool getMinWaterLevelToStartPump(uint8_t &level);
    bool getMaxWaterLevelToStopPump(uint8_t &level);
    bool getMinWaterLevelToAllowPumpControl(uint8_t &level);
    bool setDataReceptionTimeout(uint32_t timeout);
    bool getDataReceptionTimeout(uint32_t &timeout);

    esp_err_t setManualLockoutDurationMs(uint32_t duration_ms);
    uint32_t getManualLockoutDurationMs();

    // Kalman filter parameter methods
    bool setKalmanProcessNoiseVariance(float variance);
    bool setKalmanMeasurementNoiseVariance(float variance);
    bool setKalmanStallDetectionMultiplier(float multiplier);
    float getKalmanProcessNoiseVariance();
    float getKalmanMeasurementNoiseVariance();
    float getKalmanStallDetectionMultiplier();

    // Kalman filter state methods
    bool setKalmanStateXHat(float x_hat);
    bool setKalmanStateP(float p);
    bool getKalmanStateXHat(float &x_hat);
    bool getKalmanStateP(float &p);

    // Priming period methods
    bool setPrimingPeriodSeconds(float seconds);
    bool getPrimingPeriodSeconds(float &seconds);

    // Lockout status methods
    bool setLockoutActive(bool active);
    bool getLockoutActive(bool &active);
    bool setLockoutStartedAtMillis(uint64_t timestamp);
    bool getLockoutStartedAtMillis(uint64_t &timestamp);

    bool setLastPumpRunMetaData(const LastPumpRunMetaData &metaData);
    bool getLastPumpRunMetaData(LastPumpRunMetaData &metaData);

    /**
     * @brief Read a boolean value from NVS
     * 
     * Reads a boolean value stored in NVS. If the key doesn't exist or read fails,
     * returns the provided default value.
     * 
     * @param key NVS key to read (max 15 chars, e.g., "lockout_en")
     * @param defaultValue Value to return if key doesn't exist or read fails
     * @return The stored boolean value, or defaultValue if not found
     * 
     * @note Boolean values are stored as uint8_t (0 or 1) in NVS
     * @note This method is used to read lockout feature state on boot
     */
    bool readBool(const char *key, bool defaultValue);
    
    /**
     * @brief Write a boolean value to NVS
     * 
     * Writes a boolean value to NVS with automatic commit. The value is stored
     * as uint8_t (0 for false, 1 for true).
     * 
     * @param key NVS key to write (max 15 chars, e.g., "lockout_en")
     * @param value Boolean value to store
     * @return ESP_OK on success, error code on failure
     * 
     * @note Changes are committed immediately to flash
     * @note This method is used to persist lockout feature state changes
     */
    esp_err_t writeBool(const char *key, bool value);

    /**
     * @brief Read a uint8_t value from NVS
     * 
     * Reads a uint8_t value stored in NVS. If the key doesn't exist or read fails,
     * returns the provided default value.
     * 
     * @param key NVS key to read (max 15 chars, e.g., "last_wl")
     * @param defaultValue Value to return if key doesn't exist or read fails
     * @return The stored uint8_t value, or defaultValue if not found
     * 
     * @note This method is used to read last published water level on boot
     */
    uint8_t readUint8(const char *key, uint8_t defaultValue);
    
    /**
     * @brief Write a uint8_t value to NVS
     * 
     * Writes a uint8_t value to NVS with automatic commit.
     * 
     * @param key NVS key to write (max 15 chars, e.g., "last_wl")
     * @param value uint8_t value to store (0-255)
     * @return ESP_OK on success, error code on failure
     * 
     * @note Changes are committed immediately to flash
     * @note This method is used to persist last published water level
     */
    esp_err_t writeUint8(const char *key, uint8_t value);

    // other operations
    bool clearCredentials();
    bool clearAllStoredParams();
};

#endif