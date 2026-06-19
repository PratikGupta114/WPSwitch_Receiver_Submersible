#ifndef UART_CRYPTO_H_
#define UART_CRYPTO_H_

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "psa/crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UART Crypto context
 */
typedef struct {
    psa_key_id_t private_key_id;  // ECDH private key handle (destroyed after handshake)
    psa_key_id_t aes_key_id;      // Session AES-128 key handle
    
    // Session state
    bool is_initialized;
    bool is_secured;
} UartCryptoContext;

/**
 * @brief Initialize crypto context
 * 
 * @param ctx Crypto context
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_crypto_init(UartCryptoContext *ctx);

/**
 * @brief Generate ECDH keypair and export public key
 * 
 * @param ctx Crypto context
 * @param public_key Buffer to store public key (32 bytes)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_crypto_generate_keypair(UartCryptoContext *ctx, uint8_t *public_key);

/**
 * @brief Compute shared secret from peer's public key
 * 
 * @param ctx Crypto context
 * @param peer_public_key Peer's public key (32 bytes)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_crypto_compute_shared_secret(UartCryptoContext *ctx, 
                                             const uint8_t *peer_public_key);

/**
 * @brief Free ECDH contexts to save memory (call after handshake)
 * 
 * @param ctx Crypto context
 */
void uart_crypto_free_ecdh(UartCryptoContext *ctx);

/**
 * @brief Encrypt data with AES-128-CBC
 * 
 * @param ctx Crypto context
 * @param plaintext Input plaintext
 * @param plaintext_len Length of plaintext
 * @param iv Initialization vector (16 bytes, will be generated)
 * @param ciphertext Output ciphertext buffer
 * @param ciphertext_len Pointer to store ciphertext length
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_crypto_encrypt(UartCryptoContext *ctx,
                               const uint8_t *plaintext, size_t plaintext_len,
                               uint8_t *iv,
                               uint8_t *ciphertext, size_t *ciphertext_len);

/**
 * @brief Decrypt data with AES-128-CBC
 * 
 * @param ctx Crypto context
 * @param iv Initialization vector (16 bytes)
 * @param ciphertext Input ciphertext
 * @param ciphertext_len Length of ciphertext
 * @param plaintext Output plaintext buffer
 * @param plaintext_len Pointer to store plaintext length
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_crypto_decrypt(UartCryptoContext *ctx,
                               const uint8_t *iv,
                               const uint8_t *ciphertext, size_t ciphertext_len,
                               uint8_t *plaintext, size_t *plaintext_len);

/**
 * @brief Deinitialize crypto context
 * 
 * @param ctx Crypto context
 */
void uart_crypto_deinit(UartCryptoContext *ctx);

#ifdef __cplusplus
}
#endif

#endif // UART_CRYPTO_H_
