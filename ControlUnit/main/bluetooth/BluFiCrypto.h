#ifndef BLUFICRYPTO_H_
#define BLUFICRYPTO_H_

#include "mbedtls/aes.h"
#include "mbedtls/dhm.h"
#include "mbedtls/md5.h"
#include "esp_crc.h"

#include "esp_event.h"

#include "esp_err.h"

#define DH_SELF_PUBLIC_KEY_LENGTH 128
#define DH_SELF_PUBLIC_KEY_LENGTH_IN_BITS 1024

#define SHARE_LEY_LENGTH 128
#define SHARE_LEY_LENGTH_IN_BITS 1024

#define PRE_SHARED_KEY_LENGTH 16
#define INITIALIZATION_VECTOR_LENGTH 16

typedef struct blufi_crypto
{
    uint8_t selfPublicKey[DH_SELF_PUBLIC_KEY_LENGTH];
    uint8_t shareKey[SHARE_LEY_LENGTH];
    size_t shareLength;
    uint8_t preSharedKey[PRE_SHARED_KEY_LENGTH];
    uint8_t initializationVector[INITIALIZATION_VECTOR_LENGTH];
    uint8_t *dhParameters;
    int dhParametersLength;

    mbedtls_dhm_context dhmContext;
    mbedtls_aes_context aesContext;

} BluFiCrypto;

typedef enum blufi_frame_type
{
    TYPE_DH_PARAM_LEN = 0x00,
    TYPE_DH_PARAM_DATA = 0x01,
    TYPE_DH_P = 0x02,
    TYPE_DH_G = 0x03,
    TYPE_DH_PUBLIC = 0x04,
} BluFiFrameType;

int bluFiAesEncryptFunction(uint8_t initializationVector8, uint8_t *data, int cryptLength);
int bluFiAesDecryptFunction(uint8_t initializationVector8, uint8_t *data, int cryptLength);
uint16_t bluFiCRCChecksumFunction(uint8_t initializationVector8, uint8_t *data, int cryptLength);
void bluFiDHNegotiatiationDataHandler(uint8_t *data, int length, uint8_t **output_data, int *output_len, bool *need_free);

esp_err_t bluFiCryptoInit();
void bluFiCryptoDeinit();

#endif