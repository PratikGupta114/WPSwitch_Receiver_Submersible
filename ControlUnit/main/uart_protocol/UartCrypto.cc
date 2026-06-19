#include "UartCrypto.h"
#include "UartProtocol.h"
#include "esp_log.h"
#include "esp_random.h"
#include <string.h>

static const char *TAG = "UartCrypto";

esp_err_t uart_crypto_init(UartCryptoContext *ctx) {
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(ctx, 0, sizeof(UartCryptoContext));

    psa_status_t status = psa_crypto_init();
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize PSA crypto: %d", status);
        return ESP_FAIL;
    }

    ctx->is_initialized = true;
    ESP_LOGI(TAG, "Crypto context initialized");
    return ESP_OK;
}

esp_err_t uart_crypto_generate_keypair(UartCryptoContext *ctx, uint8_t *public_key) {
    if (!ctx || !ctx->is_initialized || !public_key) {
        return ESP_ERR_INVALID_ARG;
    }

    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_MONTGOMERY));
    psa_set_key_bits(&attributes, 255);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DERIVE);
    psa_set_key_algorithm(&attributes, PSA_ALG_ECDH);

    psa_status_t status = psa_generate_key(&attributes, &ctx->private_key_id);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_generate_key failed: %d", status);
        return ESP_FAIL;
    }

    // Export public key
    size_t public_key_len = 0;
    status = psa_export_public_key(ctx->private_key_id, public_key, ECDH_PUBLIC_KEY_SIZE, &public_key_len);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_export_public_key failed: %d", status);
        psa_destroy_key(ctx->private_key_id);
        ctx->private_key_id = 0;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Keypair generated");
    return ESP_OK;
}

esp_err_t uart_crypto_compute_shared_secret(UartCryptoContext *ctx, 
                                             const uint8_t *peer_public_key) {
    if (!ctx || !ctx->is_initialized || !ctx->private_key_id || !peer_public_key) {
        return ESP_ERR_INVALID_ARG;
    }

    // Compute shared secret using raw key agreement (ECDH Curve25519)
    uint8_t shared_secret[32];
    size_t shared_secret_len = 0;
    psa_status_t status = psa_raw_key_agreement(
        PSA_ALG_ECDH,
        ctx->private_key_id,
        peer_public_key, ECDH_PUBLIC_KEY_SIZE,
        shared_secret, sizeof(shared_secret),
        &shared_secret_len
    );

    // Destroy the private key since it's no longer needed after handshake (saves slots and memory)
    psa_destroy_key(ctx->private_key_id);
    ctx->private_key_id = 0;

    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_raw_key_agreement failed: %d", status);
        return ESP_FAIL;
    }

    // Hash the shared secret with SHA-256 to derive the AES key
    uint8_t hash[32];
    size_t hash_len = 0;
    status = psa_hash_compute(
        PSA_ALG_SHA_256,
        shared_secret, shared_secret_len,
        hash, sizeof(hash),
        &hash_len
    );

    // Securely wipe shared secret from memory
    memset(shared_secret, 0, sizeof(shared_secret));

    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_hash_compute failed: %d", status);
        return ESP_FAIL;
    }

    // Import derived AES-128 key into the PSA keystore (take first 16 bytes of the hash)
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_CBC_NO_PADDING);

    status = psa_import_key(&attributes, hash, AES_KEY_SIZE, &ctx->aes_key_id);

    // Securely wipe hash from memory
    memset(hash, 0, sizeof(hash));

    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to import derived AES key: %d", status);
        return ESP_FAIL;
    }

    ctx->is_secured = true;
    ESP_LOGI(TAG, "Shared secret computed, session secured");
    return ESP_OK;
}

void uart_crypto_free_ecdh(UartCryptoContext *ctx) {
    if (!ctx) {
        return;
    }
    if (ctx->private_key_id) {
        psa_destroy_key(ctx->private_key_id);
        ctx->private_key_id = 0;
    }
    ESP_LOGI(TAG, "ECDH contexts freed (PSA Key destroyed)");
}

esp_err_t uart_crypto_encrypt(UartCryptoContext *ctx,
                               const uint8_t *plaintext, size_t plaintext_len,
                               uint8_t *iv,
                               uint8_t *ciphertext, size_t *ciphertext_len) {
    if (!ctx || !ctx->is_secured || !ctx->aes_key_id || !plaintext || !iv || !ciphertext || !ciphertext_len) {
        return ESP_ERR_INVALID_ARG;
    }

    // Generate random IV
    esp_fill_random(iv, AES_IV_SIZE);

    // Calculate padded length (PKCS#7 padding)
    size_t padded_len = ((plaintext_len + AES_BLOCK_SIZE) / AES_BLOCK_SIZE) * AES_BLOCK_SIZE;
    uint8_t padded_plaintext[MAX_ENCRYPTED_PAYLOAD_SIZE];
    
    if (padded_len > MAX_ENCRYPTED_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Plaintext too large: %d bytes", plaintext_len);
        return ESP_ERR_INVALID_SIZE;
    }

    // Copy plaintext and add PKCS#7 padding
    memcpy(padded_plaintext, plaintext, plaintext_len);
    uint8_t padding_value = padded_len - plaintext_len;
    memset(padded_plaintext + plaintext_len, padding_value, padding_value);

    // Encrypt using PSA cipher API
    psa_cipher_operation_t operation = PSA_CIPHER_OPERATION_INIT;
    psa_status_t status = psa_cipher_encrypt_setup(&operation, ctx->aes_key_id, PSA_ALG_CBC_NO_PADDING);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_cipher_encrypt_setup failed: %d", status);
        return ESP_FAIL;
    }

    status = psa_cipher_set_iv(&operation, iv, AES_IV_SIZE);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_cipher_set_iv failed: %d", status);
        psa_cipher_abort(&operation);
        return ESP_FAIL;
    }

    size_t out_len = 0;
    status = psa_cipher_update(&operation, padded_plaintext, padded_len, ciphertext, padded_len, &out_len);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_cipher_update failed: %d", status);
        psa_cipher_abort(&operation);
        return ESP_FAIL;
    }

    size_t final_len = 0;
    status = psa_cipher_finish(&operation, ciphertext + out_len, padded_len - out_len, &final_len);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_cipher_finish failed: %d", status);
        psa_cipher_abort(&operation);
        return ESP_FAIL;
    }

    *ciphertext_len = out_len + final_len;
    return ESP_OK;
}

esp_err_t uart_crypto_decrypt(UartCryptoContext *ctx,
                               const uint8_t *iv,
                               const uint8_t *ciphertext, size_t ciphertext_len,
                               uint8_t *plaintext, size_t *plaintext_len) {
    if (!ctx || !ctx->is_secured || !ctx->aes_key_id || !iv || !ciphertext || !plaintext || !plaintext_len) {
        return ESP_ERR_INVALID_ARG;
    }

    if (ciphertext_len % AES_BLOCK_SIZE != 0) {
        ESP_LOGE(TAG, "Invalid ciphertext length: %d", ciphertext_len);
        return ESP_ERR_INVALID_SIZE;
    }

    // Decrypt using PSA cipher API
    psa_cipher_operation_t operation = PSA_CIPHER_OPERATION_INIT;
    psa_status_t status = psa_cipher_decrypt_setup(&operation, ctx->aes_key_id, PSA_ALG_CBC_NO_PADDING);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_cipher_decrypt_setup failed: %d", status);
        return ESP_FAIL;
    }

    status = psa_cipher_set_iv(&operation, iv, AES_IV_SIZE);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_cipher_set_iv failed: %d", status);
        psa_cipher_abort(&operation);
        return ESP_FAIL;
    }

    size_t out_len = 0;
    status = psa_cipher_update(&operation, ciphertext, ciphertext_len, plaintext, ciphertext_len, &out_len);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_cipher_update failed: %d", status);
        psa_cipher_abort(&operation);
        return ESP_FAIL;
    }

    size_t final_len = 0;
    status = psa_cipher_finish(&operation, plaintext + out_len, ciphertext_len - out_len, &final_len);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_cipher_finish failed: %d", status);
        psa_cipher_abort(&operation);
        return ESP_FAIL;
    }

    size_t total_decrypted = out_len + final_len;

    // Remove PKCS#7 padding
    uint8_t padding_value = plaintext[total_decrypted - 1];
    if (padding_value > AES_BLOCK_SIZE || padding_value == 0) {
        ESP_LOGE(TAG, "Invalid padding: %d", padding_value);
        return ESP_FAIL;
    }

    *plaintext_len = total_decrypted - padding_value;
    return ESP_OK;
}

void uart_crypto_deinit(UartCryptoContext *ctx) {
    if (!ctx) {
        return;
    }

    if (ctx->private_key_id) {
        psa_destroy_key(ctx->private_key_id);
    }
    if (ctx->aes_key_id) {
        psa_destroy_key(ctx->aes_key_id);
    }
    
    memset(ctx, 0, sizeof(UartCryptoContext));
    ESP_LOGI(TAG, "Crypto context deinitialized");
}
