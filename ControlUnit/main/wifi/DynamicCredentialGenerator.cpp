#include "DynamicCredentialGenerator.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "psa/crypto.h"
#include <string.h>

#define TAG "DynamicCredGen"

// Application secret (32 bytes) - from specification
// WARNING: Replace with your own randomly generated secret in production!
const uint8_t DynamicCredentialGenerator::APPLICATION_SECRET[32] = {
    0xA7, 0xB3, 0xC9, 0xD2, 0xE5, 0xF1, 0xA8, 0xB4,
    0xC6, 0xD8, 0xE2, 0xF5, 0xA9, 0xB7, 0xC3, 0xD1,
    0xE4, 0xF6, 0xA2, 0xB8, 0xC5, 0xD9, 0xE3, 0xF7,
    0xA1, 0xB6, 0xC4, 0xD2, 0xE8, 0xF5, 0xA3, 0xB9
};

// Info strings for HKDF-Expand
const char* DynamicCredentialGenerator::INFO_SSID = "SSID_v1";
const char* DynamicCredentialGenerator::INFO_PASSWORD = "PASSWORD_v1";

// Base62 alphabet: 0-9, A-Z, a-z
const char DynamicCredentialGenerator::BASE62_ALPHABET[63] = 
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

esp_err_t DynamicCredentialGenerator::hkdfExtract(const uint8_t* salt, size_t salt_len,
                                                   const uint8_t* ikm, size_t ikm_len,
                                                   uint8_t* prk) {
    if (!salt || !ikm || !prk) {
        ESP_LOGE(TAG, "HKDF-Extract: NULL pointer parameter");
        return ESP_ERR_INVALID_ARG;
    }

    psa_status_t status = psa_crypto_init();
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize PSA crypto: %d", status);
        return ESP_FAIL;
    }

    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attributes, PSA_KEY_TYPE_HMAC);
    psa_set_key_bits(&attributes, salt_len * 8);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attributes, PSA_ALG_HMAC(PSA_ALG_SHA_256));

    psa_key_id_t key_id = 0;
    status = psa_import_key(&attributes, salt, salt_len, &key_id);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "HKDF-Extract: Failed to import salt key: %d", status);
        return ESP_FAIL;
    }

    size_t mac_len = 0;
    status = psa_mac_compute(
        key_id,
        PSA_ALG_HMAC(PSA_ALG_SHA_256),
        ikm, ikm_len,
        prk, 32,
        &mac_len
    );

    psa_destroy_key(key_id);

    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "HKDF-Extract: MAC computation failed: %d", status);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t DynamicCredentialGenerator::hkdfExpand(const uint8_t* prk, size_t prk_len,
                                                  const uint8_t* info, size_t info_len,
                                                  uint8_t* okm, size_t okm_len) {
    if (!prk || !info || !okm) {
        ESP_LOGE(TAG, "HKDF-Expand: NULL pointer parameter");
        return ESP_ERR_INVALID_ARG;
    }
    
    // For SHA-256, max output length is 255 * 32 = 8160 bytes
    if (okm_len > 255 * 32) {
        ESP_LOGE(TAG, "HKDF-Expand: Requested length too large");
        return ESP_ERR_INVALID_SIZE;
    }

    psa_status_t status = psa_crypto_init();
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize PSA crypto: %d", status);
        return ESP_FAIL;
    }

    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attributes, PSA_KEY_TYPE_HMAC);
    psa_set_key_bits(&attributes, prk_len * 8);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attributes, PSA_ALG_HMAC(PSA_ALG_SHA_256));

    psa_key_id_t key_id = 0;
    status = psa_import_key(&attributes, prk, prk_len, &key_id);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "HKDF-Expand: Failed to import PRK key: %d", status);
        return ESP_FAIL;
    }

    uint8_t t[32] = {0};  // Previous T value (empty for first iteration)
    size_t t_len = 0;
    uint8_t counter = 1;
    size_t generated = 0;
    
    while (generated < okm_len) {
        psa_mac_operation_t operation = PSA_MAC_OPERATION_INIT;
        status = psa_mac_sign_setup(&operation, key_id, PSA_ALG_HMAC(PSA_ALG_SHA_256));
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "HKDF-Expand: HMAC sign setup failed: %d", status);
            psa_destroy_key(key_id);
            return ESP_FAIL;
        }

        if (t_len > 0) {
            status = psa_mac_update(&operation, t, t_len);
            if (status != PSA_SUCCESS) {
                ESP_LOGE(TAG, "HKDF-Expand: HMAC update T failed: %d", status);
                psa_mac_abort(&operation);
                psa_destroy_key(key_id);
                return ESP_FAIL;
            }
        }

        status = psa_mac_update(&operation, info, info_len);
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "HKDF-Expand: HMAC update info failed: %d", status);
            psa_mac_abort(&operation);
            psa_destroy_key(key_id);
            return ESP_FAIL;
        }

        status = psa_mac_update(&operation, &counter, 1);
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "HKDF-Expand: HMAC update counter failed: %d", status);
            psa_mac_abort(&operation);
            psa_destroy_key(key_id);
            return ESP_FAIL;
        }

        size_t out_len = 0;
        status = psa_mac_sign_finish(&operation, t, sizeof(t), &out_len);
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "HKDF-Expand: HMAC sign finish failed: %d", status);
            psa_mac_abort(&operation);
            psa_destroy_key(key_id);
            return ESP_FAIL;
        }

        t_len = out_len;

        // Copy to output
        size_t to_copy = (okm_len - generated < t_len) ? (okm_len - generated) : t_len;
        memcpy(okm + generated, t, to_copy);
        generated += to_copy;
        counter++;
    }

    psa_destroy_key(key_id);
    return ESP_OK;
}


esp_err_t DynamicCredentialGenerator::base62Encode(const uint8_t* input, size_t input_len,
                                                    char* output, size_t output_len,
                                                    size_t fixed_width) {
    if (!input || !output) {
        ESP_LOGE(TAG, "Base62: NULL pointer parameter");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (output_len < 2) {
        ESP_LOGE(TAG, "Base62: Output buffer too small");
        return ESP_ERR_INVALID_SIZE;
    }
    
    ESP_LOGD(TAG, "Base62 Encode: input_len=%zu, fixed_width=%zu", input_len, fixed_width);
    
    // Create a working copy of input for division
    uint8_t num[input_len];
    memcpy(num, input, input_len);
    
    // Result buffer (max ~43 chars for 24 bytes, ~30 for 18 bytes)
    char result[64];
    int result_len = 0;
    
    // Check if input is all zeros
    bool all_zero = true;
    for (size_t i = 0; i < input_len; i++) {
        if (num[i] != 0) {
            all_zero = false;
            break;
        }
    }
    
    if (all_zero) {
        result[result_len++] = BASE62_ALPHABET[0];
    } else {
        // Repeated division by 62
        size_t num_len = input_len;
        
        while (num_len > 0) {
            int remainder = 0;
            size_t new_len = 0;
            
            // Divide the number by 62
            for (size_t i = 0; i < num_len; i++) {
                int current = remainder * 256 + num[i];
                num[i] = current / 62;
                remainder = current % 62;
                
                // Keep track of significant digits
                if (num[i] != 0 || new_len > 0) {
                    if (new_len < i) {
                        num[new_len] = num[i];
                    }
                    new_len++;
                }
            }
            
            result[result_len++] = BASE62_ALPHABET[remainder];
            num_len = new_len;
        }
    }
    
    // Apply fixed width padding if requested
    size_t final_len = result_len;
    if (fixed_width > 0 && result_len < fixed_width) {
        final_len = fixed_width;
    }
    
    // Check if output buffer is large enough
    if (output_len < final_len + 1) {
        ESP_LOGE(TAG, "Base62: Output buffer too small (need %zu, have %zu)", 
                 final_len + 1, output_len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Add padding if needed
    size_t padding = (fixed_width > result_len) ? (fixed_width - result_len) : 0;
    for (size_t i = 0; i < padding; i++) {
        output[i] = BASE62_ALPHABET[0];
    }
    
    // Reverse and copy the result
    for (int i = 0; i < result_len; i++) {
        output[padding + i] = result[result_len - 1 - i];
    }
    output[final_len] = '\0';
    
    ESP_LOGD(TAG, "Base62 Encode: result_len=%d, padding=%zu, final_len=%zu", 
             result_len, padding, final_len);
    
    return ESP_OK;
}

esp_err_t DynamicCredentialGenerator::generateSSID(const uint8_t* device_mac, 
                                                    char* ssid_out, 
                                                    size_t ssid_out_len) {
    if (!device_mac || !ssid_out) {
        ESP_LOGE(TAG, "generateSSID: NULL pointer parameter");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ssid_out_len < SSID_LENGTH + 1) {
        ESP_LOGE(TAG, "generateSSID: Buffer too small (need %zu, have %zu)", 
                 SSID_LENGTH + 1, ssid_out_len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    ESP_LOGD(TAG, "SSID Generation: MAC=%02X:%02X:%02X:%02X:%02X:%02X", 
             device_mac[0], device_mac[1], device_mac[2], 
             device_mac[3], device_mac[4], device_mac[5]);
    
    // HKDF-Extract: PRK = HMAC-SHA256(APPLICATION_SECRET, device_mac)
    uint8_t prk[32];
    esp_err_t ret = hkdfExtract(APPLICATION_SECRET, 32, device_mac, MAC_ADDRESS_LENGTH, prk);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "generateSSID: HKDF-Extract failed");
        return ret;
    }
    
    ESP_LOGD(TAG, "SSID Generation: PRK=%02X%02X%02X%02X...%02X%02X (32 bytes)", 
             prk[0], prk[1], prk[2], prk[3], prk[30], prk[31]);
    
    // HKDF-Expand: OKM = HKDF-Expand(PRK, "SSID_v1", 18)
    uint8_t okm[18];
    ret = hkdfExpand(prk, 32, (const uint8_t*)INFO_SSID, strlen(INFO_SSID), okm, 18);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "generateSSID: HKDF-Expand failed");
        return ret;
    }
    
    ESP_LOGD(TAG, "SSID Generation: OKM=%02X%02X%02X%02X...%02X%02X (18 bytes)", 
             okm[0], okm[1], okm[2], okm[3], okm[16], okm[17]);
    
    // Base62 encode with fixed width of 30 characters
    ret = base62Encode(okm, 18, ssid_out, ssid_out_len, SSID_LENGTH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "generateSSID: Base62 encoding failed");
        return ret;
    }
    
    ESP_LOGI(TAG, "Generated SSID: %s (length: %d)", ssid_out, strlen(ssid_out));
    return ESP_OK;
}

esp_err_t DynamicCredentialGenerator::generatePassword(const uint8_t* device_mac,
                                                        char* password_out,
                                                        size_t password_out_len) {
    if (!device_mac || !password_out) {
        ESP_LOGE(TAG, "generatePassword: NULL pointer parameter");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (password_out_len < 64) {
        ESP_LOGE(TAG, "generatePassword: Buffer too small (need 64, have %zu)", 
                 password_out_len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    ESP_LOGD(TAG, "Password Generation: MAC=%02X:%02X:%02X:%02X:%02X:%02X", 
             device_mac[0], device_mac[1], device_mac[2], 
             device_mac[3], device_mac[4], device_mac[5]);
    
    // HKDF-Extract: PRK = HMAC-SHA256(APPLICATION_SECRET, device_mac)
    uint8_t prk[32];
    esp_err_t ret = hkdfExtract(APPLICATION_SECRET, 32, device_mac, MAC_ADDRESS_LENGTH, prk);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "generatePassword: HKDF-Extract failed");
        return ret;
    }
    
    ESP_LOGD(TAG, "Password Generation: PRK=%02X%02X%02X%02X...%02X%02X (32 bytes)", 
             prk[0], prk[1], prk[2], prk[3], prk[30], prk[31]);
    
    // HKDF-Expand: OKM = HKDF-Expand(PRK, "PASSWORD_v1", 24)
    uint8_t okm[24];
    ret = hkdfExpand(prk, 32, (const uint8_t*)INFO_PASSWORD, strlen(INFO_PASSWORD), okm, 24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "generatePassword: HKDF-Expand failed");
        return ret;
    }
    
    ESP_LOGD(TAG, "Password Generation: OKM=%02X%02X%02X%02X...%02X%02X (24 bytes)", 
             okm[0], okm[1], okm[2], okm[3], okm[22], okm[23]);
    
    // Base62 encode (variable width, typically 32-33 characters)
    ret = base62Encode(okm, 24, password_out, password_out_len, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "generatePassword: Base62 encoding failed");
        return ret;
    }
    
    // Don't log the actual password for security
    ESP_LOGI(TAG, "Generated password (length: %d)", strlen(password_out));
    return ESP_OK;
}

esp_err_t DynamicCredentialGenerator::generateSSID(char* ssid_out, size_t ssid_out_len) {
    uint8_t mac[MAC_ADDRESS_LENGTH];
    esp_err_t ret = esp_efuse_mac_get_default(mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "generateSSID: Failed to get MAC address: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "=== Dynamic SSID Generation ===");
    ESP_LOGI(TAG, "Device MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    ret = generateSSID(mac, ssid_out, ssid_out_len);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SSID generation successful");
    } else {
        ESP_LOGE(TAG, "SSID generation failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t DynamicCredentialGenerator::generatePassword(char* password_out, size_t password_out_len) {
    uint8_t mac[MAC_ADDRESS_LENGTH];
    esp_err_t ret = esp_efuse_mac_get_default(mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "generatePassword: Failed to get MAC address: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "=== Dynamic Password Generation ===");
    
    ret = generatePassword(mac, password_out, password_out_len);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Password generation successful");
    } else {
        ESP_LOGE(TAG, "Password generation failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}
