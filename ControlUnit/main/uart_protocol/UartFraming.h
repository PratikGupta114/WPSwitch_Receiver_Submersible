#ifndef UART_FRAMING_H_
#define UART_FRAMING_H_

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculate CRC32 checksum
 * 
 * @param data Pointer to data buffer
 * @param length Length of data
 * @return uint32_t CRC32 checksum
 */
uint32_t uart_framing_crc32(const uint8_t *data, size_t length);

/**
 * @brief Encode packet with COBS framing
 * 
 * Adds CRC32 to the packet, then COBS encodes it, and appends delimiter
 * 
 * @param input Input data buffer
 * @param input_len Length of input data
 * @param output Output buffer (must be large enough)
 * @param output_len Pointer to store output length
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_framing_encode(const uint8_t *input, size_t input_len,
                               uint8_t *output, size_t *output_len);

/**
 * @brief Decode COBS-encoded packet and verify CRC32
 * 
 * @param input COBS-encoded data (without delimiter)
 * @param input_len Length of encoded data
 * @param output Output buffer for decoded data
 * @param output_len Pointer to store output length
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_CRC if CRC mismatch
 */
esp_err_t uart_framing_decode(const uint8_t *input, size_t input_len,
                               uint8_t *output, size_t *output_len);

#ifdef __cplusplus
}
#endif

#endif // UART_FRAMING_H_
