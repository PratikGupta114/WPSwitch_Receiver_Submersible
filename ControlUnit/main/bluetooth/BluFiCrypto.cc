
#include "BluFiCrypto.h"

// #include "esp_blufi.h"
extern "C"
{
#include "esp_blufi.h"
    extern void btc_blufi_report_error(esp_blufi_error_state_t state);
}

#include "string.h"
#include "stdlib.h"
#include "esp_log.h"

#define TAG "BluFiCrypto"

BluFiCrypto *blufiCrypto = NULL;

int randomGenerator(void *rngState, unsigned char *output, size_t length)
{
    esp_fill_random(output, length);
    return (0);
}

int bluFiAesEncryptFunction(uint8_t iv8, uint8_t *data, int cryptLength)
{
    size_t ivOffset = 0;
    uint8_t iv0[INITIALIZATION_VECTOR_LENGTH];

    // copies the initialization vector inside the struct to the local array
    memcpy(iv0, blufiCrypto->initializationVector, sizeof(blufiCrypto->initializationVector));
    iv0[0] = iv8;

    // Function description here : https://os.mbed.com/teams/sandbox/code/mbedtls/docs/tip/aes_8h.html
    // returns 0 if successful
    if (mbedtls_aes_crypt_cfb128(
            &(blufiCrypto->aesContext), // AES context object defined in the bluFiCrypto struct
            MBEDTLS_AES_ENCRYPT,        // Mode of encryption (encryption here)
            cryptLength,                // length of the data to be encrypted
            &ivOffset,                  // ivOffest, which initially is 0
            iv0,                        // Initialization Vector
            data,                       // Buffer holding the input data
            data))                      // Buffer holding the output data
    {
        return -1;
    }

    // return the length of the data encrypted.
    return cryptLength;
}
int bluFiAesDecryptFunction(uint8_t iv8, uint8_t *data, int cryptLength)
{
    size_t ivOffset = 0;
    uint8_t iv0[INITIALIZATION_VECTOR_LENGTH];

    memcpy(iv0, blufiCrypto->initializationVector, sizeof(blufiCrypto->initializationVector));
    iv0[0] = iv8;

    // Function description here : https://os.mbed.com/teams/sandbox/code/mbedtls/docs/tip/aes_8h.html
    // returns 0 if successful
    if (mbedtls_aes_crypt_cfb128(
            &(blufiCrypto->aesContext), // AES context object defined in the bluFiCrypto struct
            MBEDTLS_AES_DECRYPT,        // Mode of encryption (decryption here)
            cryptLength,                // length of the data to be encrypted
            &ivOffset,                  // ivOffest, which initially is 0
            iv0,                        // Initialization Vector
            data,                       // Buffer holding the input data
            data))                      // Buffer holding the output data
    {
        return -1;
    }

    return cryptLength;
}
uint16_t bluFiCRCChecksumFunction(uint8_t iv8, uint8_t *data, int cryptLength)
{
    // iv8 is ignored here.
    return esp_crc16_be(0, data, cryptLength);
}
void bluFiDHNegotiatiationDataHandler(uint8_t *data, int length, uint8_t **output_data, int *output_len, bool *need_free)
{
    // extract the first byte from the frame which represent the type of the payload data sent
    BluFiFrameType type = (BluFiFrameType)data[0];

    // Check if the blufiCrypto struct is initialized first
    if (blufiCrypto == NULL)
    {
        ESP_LOGE(TAG, "Blufi not initialized; blufiCrpyto struct is null !");
        btc_blufi_report_error(ESP_BLUFI_INIT_SECURITY_ERROR);
        return;
    }

    switch (type)
    {
    case TYPE_DH_PARAM_LEN:
    {
        // The next two bytes represent the length of the data, hence
        blufiCrypto->dhParametersLength = ((data[1] << 8) | (data[2]));

        // If the DH paramters already exist then free them and allocate memory again.
        if (blufiCrypto->dhParameters)
        {
            free(blufiCrypto->dhParameters);
            blufiCrypto->dhParameters = NULL;
        }

        // allocate the dh buffer again
        blufiCrypto->dhParameters = (uint8_t *)malloc(sizeof(uint8_t) * blufiCrypto->dhParametersLength);

        // Check if the memory allocation was successful
        if (blufiCrypto->dhParameters == NULL)
        {
            btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
            ESP_LOGE(TAG, " %s() Failed to allocate memory for DH params", __func__);
            return;
        }
    }
    break;
    case TYPE_DH_PARAM_DATA:
    {
        if (blufiCrypto->dhParameters == NULL)
        {
            ESP_LOGE(TAG, "%s() -> dh parameter buffer not allocated", __func__);
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        uint8_t *dhParams = blufiCrypto->dhParameters;
        // copy the data from the second byte to the dh params buffer
        memcpy(blufiCrypto->dhParameters, &(data[1]), blufiCrypto->dhParametersLength);

        // Read the params from the data and report an error if read was a failure
        if (mbedtls_dhm_read_params(
                &(blufiCrypto->dhmContext),
                &(dhParams),
                &(dhParams[blufiCrypto->dhParametersLength])))
        {
            ESP_LOGE(TAG, "Failed to read params from the data");
            btc_blufi_report_error(ESP_BLUFI_READ_PARAM_ERROR);
            return;
        }

        // De allocate the memory fro the dh parameters since the dhConectext already has the data
        free(blufiCrypto->dhParameters);
        blufiCrypto->dhParameters = NULL;

        // Now let us calculate the shared secret
        if (mbedtls_dhm_make_public(
                &(blufiCrypto->dhmContext),
                mbedtls_mpi_size(&(blufiCrypto->dhmContext.P)),
                blufiCrypto->selfPublicKey,
                blufiCrypto->dhmContext.len,
                randomGenerator,
                NULL))
        {
            ESP_LOGE(TAG, "make public failed !");
            btc_blufi_report_error(ESP_BLUFI_MAKE_PUBLIC_ERROR);
            return;
        }

        mbedtls_dhm_calc_secret(
            &(blufiCrypto->dhmContext),
            blufiCrypto->shareKey,
            SHARE_LEY_LENGTH_IN_BITS,
            &(blufiCrypto->shareLength),
            NULL,
            NULL);

        // Now that we have the shared secret ready, let's now generate the
        // AES key which apparently is the md5 digest of the shared key
        mbedtls_md5(
            blufiCrypto->shareKey,
            blufiCrypto->shareLength,
            blufiCrypto->preSharedKey);

        mbedtls_aes_setkey_enc(
            &(blufiCrypto->aesContext),
            blufiCrypto->preSharedKey,
            SHARE_LEY_LENGTH);

        // allocate the output data as pointers to the encrypted data
        *output_data = &(blufiCrypto->selfPublicKey[0]);
        *output_len = blufiCrypto->dhmContext.len;
        *need_free = false;
    }
    break;
    case TYPE_DH_P:
    {
    }
    break;
    case TYPE_DH_G:
    {
    }
    break;
    case TYPE_DH_PUBLIC:
    {
    }
    break;
    default:
    {
    }
    break;
    }
}

esp_err_t bluFiCryptoInit()
{
    // Let's allocate the memory for the BluFiCrypto struct
    blufiCrypto = (BluFiCrypto *)malloc(sizeof(BluFiCrypto));
    // Check if the memory was successfully allocated
    if (blufiCrypto == NULL)
    {
        ESP_LOGE(TAG, "BluFiCrypto malloc failed ");
        return ESP_FAIL;
    }

    // Let's initialize the memory for the blufiCrypto struct
    memset(blufiCrypto, 0x0, sizeof(BluFiCrypto));

    // Initialize the DH context
    mbedtls_dhm_init(&(blufiCrypto->dhmContext));
    // Initialize the AES context
    mbedtls_aes_init(&(blufiCrypto->aesContext));

    // set the initialization vector to 0
    memset(blufiCrypto->initializationVector, 0x0, INITIALIZATION_VECTOR_LENGTH);
    return ESP_OK;
}

void bluFiCryptoDeinit()
{
    // Check if the blufiCrypto struct is already null
    if (blufiCrypto == NULL)
    {
        ESP_LOGW(TAG, " blufiCryptoDeinit() -> blufiCrypto struct was already null");
        return;
    }

    // Free the dynamically allocated dh parameters
    if (blufiCrypto->dhParameters)
    {
        free(blufiCrypto->dhParameters);
        blufiCrypto->dhParameters = NULL;
    }

    // deinit the dhm and aes contexts inside the blufiCrypto struct
    mbedtls_dhm_free(&(blufiCrypto->dhmContext));
    mbedtls_aes_free(&(blufiCrypto->aesContext));

    // set the blufiCrypto struct to 0 , ready to be deallocated
    memset(blufiCrypto, 0x0, sizeof(BluFiCrypto));

    // deallocate the dynamically allocated blufiCrypto struct
    free(blufiCrypto);
    blufiCrypto = NULL;
}