#ifndef DYNAMIC_CREDENTIAL_GENERATOR_H
#define DYNAMIC_CREDENTIAL_GENERATOR_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

/**
 * @brief Dynamic WiFi AP Credential Generator
 * 
 * Implements HKDF-SHA256 based credential derivation for generating unique
 * WiFi AP SSID and password from device MAC address.
 * 
 * Algorithm specification: WiFi_AP_Dynamic_Credentials_Algorithm.md v2.0
 * 
 * Features:
 * - SSID: 30-character cryptic alphanumeric string (Base62 encoded)
 * - Password: 32-33 character alphanumeric string (Base62 encoded)
 * - HKDF-SHA256 key derivation with application secret
 * - Unique credentials per device based on MAC address
 */
class DynamicCredentialGenerator {
public:
    /**
     * @brief Generate dynamic WiFi AP SSID from device MAC address
     * 
     * Retrieves the device MAC address automatically and generates a unique
     * 30-character SSID using HKDF-SHA256 and Base62 encoding.
     * 
     * @param ssid_out Output buffer for SSID (minimum 31 bytes for null terminator)
     * @param ssid_out_len Size of output buffer
     * @return ESP_OK on success
     *         ESP_ERR_INVALID_SIZE if buffer too small
     *         ESP_FAIL if generation fails
     */
    static esp_err_t generateSSID(char* ssid_out, size_t ssid_out_len);
    
    /**
     * @brief Generate dynamic WiFi AP password from device MAC address
     * 
     * Retrieves the device MAC address automatically and generates a unique
     * 32-33 character password using HKDF-SHA256 and Base62 encoding.
     * 
     * @param password_out Output buffer for password (minimum 64 bytes recommended)
     * @param password_out_len Size of output buffer
     * @return ESP_OK on success
     *         ESP_ERR_INVALID_SIZE if buffer too small
     *         ESP_FAIL if generation fails
     */
    static esp_err_t generatePassword(char* password_out, size_t password_out_len);
    
    /**
     * @brief Generate SSID from specific MAC address (for testing)
     * 
     * @param device_mac 6-byte MAC address
     * @param ssid_out Output buffer for SSID (minimum 31 bytes)
     * @param ssid_out_len Size of output buffer
     * @return ESP_OK on success, ESP_ERR_* on failure
     */
    static esp_err_t generateSSID(const uint8_t* device_mac, char* ssid_out, size_t ssid_out_len);
    
    /**
     * @brief Generate password from specific MAC address (for testing)
     * 
     * @param device_mac 6-byte MAC address
     * @param password_out Output buffer for password (minimum 64 bytes)
     * @param password_out_len Size of output buffer
     * @return ESP_OK on success, ESP_ERR_* on failure
     */
    static esp_err_t generatePassword(const uint8_t* device_mac, char* password_out, size_t password_out_len);

    // Constants
    static const size_t SSID_LENGTH = 30;  // Fixed SSID length in characters
    static const size_t MAC_ADDRESS_LENGTH = 6;  // MAC address length in bytes

private:
    /**
     * @brief HKDF-Extract: Derive pseudorandom key from input key material
     * 
     * Implements RFC 5869 HKDF-Extract using HMAC-SHA256.
     * PRK = HMAC-SHA256(salt, IKM)
     * 
     * @param salt Salt value (typically APPLICATION_SECRET)
     * @param salt_len Length of salt in bytes
     * @param ikm Input key material (typically device MAC)
     * @param ikm_len Length of IKM in bytes
     * @param prk Output pseudorandom key (32 bytes for SHA-256)
     * @return ESP_OK on success, ESP_FAIL on mbedTLS error
     */
    static esp_err_t hkdfExtract(const uint8_t* salt, size_t salt_len,
                                  const uint8_t* ikm, size_t ikm_len,
                                  uint8_t* prk);
    
    /**
     * @brief HKDF-Expand: Expand pseudorandom key to desired length
     * 
     * Implements RFC 5869 HKDF-Expand using HMAC-SHA256.
     * OKM = HMAC-SHA256(PRK, info || 0x01)
     * 
     * @param prk Pseudorandom key from HKDF-Extract (32 bytes)
     * @param prk_len Length of PRK in bytes
     * @param info Context and application specific information
     * @param info_len Length of info in bytes
     * @param okm Output keying material
     * @param okm_len Desired length of OKM in bytes
     * @return ESP_OK on success, ESP_FAIL on mbedTLS error
     */
    static esp_err_t hkdfExpand(const uint8_t* prk, size_t prk_len,
                                 const uint8_t* info, size_t info_len,
                                 uint8_t* okm, size_t okm_len);
    
    /**
     * @brief Encode bytes to Base62 string
     * 
     * Converts arbitrary byte array to Base62 encoding using alphabet:
     * 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz
     * 
     * @param input Input byte array
     * @param input_len Length of input in bytes
     * @param output Output string buffer (null-terminated)
     * @param output_len Size of output buffer
     * @param fixed_width If > 0, pad output to this width with leading '0's
     * @return ESP_OK on success, ESP_ERR_INVALID_SIZE if buffer too small
     */
    static esp_err_t base62Encode(const uint8_t* input, size_t input_len,
                                   char* output, size_t output_len, 
                                   size_t fixed_width = 0);
    
    // Application secret (32 bytes) - must match Flutter app
    // WARNING: Replace with your own randomly generated secret in production!
    static const uint8_t APPLICATION_SECRET[32];
    
    // Info strings for HKDF-Expand
    static const char* INFO_SSID;
    static const char* INFO_PASSWORD;
    
    // Base62 alphabet
    static const char BASE62_ALPHABET[63];
};

#endif // DYNAMIC_CREDENTIAL_GENERATOR_H
