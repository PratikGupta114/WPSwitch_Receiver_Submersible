#include "UartFraming.h"
#include "UartProtocol.h"
#include "esp_log.h"
#include "cobs.hpp"
#include <string.h>
#include <span>

// Software CRC32 implementation - used on all platforms for consistency
// Uses IEEE 802.3 polynomial (0xEDB88320)
static const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

static uint32_t software_crc32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return ~crc;
}

static const char *TAG = "UartFraming";

uint32_t uart_framing_crc32(const uint8_t *data, size_t length) {
    // Use software CRC32 implementation to match ESP8266 transmitter exactly
    return software_crc32(data, length);
}

esp_err_t uart_framing_encode(const uint8_t *input, size_t input_len,
                               uint8_t *output, size_t *output_len) {
    if (!input || !output || !output_len || input_len == 0) {
        ESP_LOGE(TAG, "encode: Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "encode: Input %d bytes", input_len);

    // Calculate CRC32 of input data
    uint32_t crc = uart_framing_crc32(input, input_len);
    ESP_LOGI(TAG, "encode: CRC32 calculated: 0x%08X", crc);

    // Create temporary buffer with data + CRC32
    uint8_t temp_buffer[MAX_PACKET_SIZE];
    if (input_len + CRC32_SIZE > MAX_PACKET_SIZE) {
        ESP_LOGE(TAG, "encode: Input too large: %d bytes (max: %d)", input_len, MAX_PACKET_SIZE - CRC32_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(temp_buffer, input, input_len);
    memcpy(temp_buffer + input_len, &crc, CRC32_SIZE);

    size_t total_len = input_len + CRC32_SIZE;
    ESP_LOGI(TAG, "encode: Data + CRC32: %d bytes", total_len);

    // COBS encode using C++ API
    std::span<const uint8_t> input_span(temp_buffer, total_len);
    std::span<uint8_t> output_span(output, MAX_ENCODED_SIZE);
    
    size_t encoded_len = espp::Cobs::encode_packet(input_span, output_span);
    if (encoded_len == 0) {
        ESP_LOGE(TAG, "encode: COBS encode failed");
        return ESP_FAIL;
    }

    *output_len = encoded_len;  // encode_packet already includes delimiter
    ESP_LOGI(TAG, "encode: COBS encoded: %d bytes (includes delimiter)", encoded_len);

    return ESP_OK;
}

esp_err_t uart_framing_decode(const uint8_t *input, size_t input_len,
                               uint8_t *output, size_t *output_len) {
    if (!input || !output || !output_len || input_len == 0) {
        ESP_LOGE(TAG, "decode: Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "decode: Input %d bytes (COBS encoded)", input_len);

    // COBS decode using C++ API
    uint8_t temp_buffer[MAX_PACKET_SIZE];
    std::span<const uint8_t> input_span(input, input_len);
    std::span<uint8_t> output_span(temp_buffer, MAX_PACKET_SIZE);
    
    size_t decoded_len = espp::Cobs::decode_packet(input_span, output_span);
    
    if (decoded_len == 0) {
        ESP_LOGW(TAG, "decode: COBS decode failed (invalid encoding)");
        return ESP_ERR_INVALID_SIZE;
    }
    
    if (decoded_len < CRC32_SIZE) {
        ESP_LOGW(TAG, "decode: Packet too small: %d bytes (need at least %d)", decoded_len, CRC32_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "decode: COBS decoded: %d bytes", decoded_len);

    // Extract CRC32 (last 4 bytes)
    uint32_t received_crc;
    memcpy(&received_crc, temp_buffer + decoded_len - CRC32_SIZE, CRC32_SIZE);

    // Calculate CRC32 of payload (everything except last 4 bytes)
    size_t payload_len = decoded_len - CRC32_SIZE;
    uint32_t calculated_crc = uart_framing_crc32(temp_buffer, payload_len);

    // ESP_LOGW(TAG, "decode: CRC32 check - received: 0x%08lX, calculated: 0x%08lX", 
    //          (unsigned long)received_crc, (unsigned long)calculated_crc);

    // Log hex dump of decoded payload for debugging (commented out to reduce verbosity)
    // Structure: [type(1)][dataType(1)][seq(8)][waterLevel(1)][temp(1)][humidity(1)][mac(6)][checksum(1)] = 20 bytes
    // if (payload_len >= 20) {
    //     uint8_t pktType = temp_buffer[0];      // Packet type (0x01)
    //     uint8_t dataType = temp_buffer[1];     // Data type (0x93)
    //     uint8_t waterLevel = temp_buffer[10];  // Water level %
    //     uint8_t temp = temp_buffer[11];        // Temperature °C
    //     uint8_t humidity = temp_buffer[12];    // Humidity %
    //     ESP_LOGW(TAG, "decode: Data - pktType=0x%02X dataType=0x%02X WL=%d%% Temp=%d°C Hum=%d%%",
    //              pktType, dataType, waterLevel, temp, humidity);
    //     
    //     // Log first 20 bytes as hex
    //     ESP_LOGW(TAG, "decode: Hex[0-9]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
    //              temp_buffer[0], temp_buffer[1], temp_buffer[2], temp_buffer[3], temp_buffer[4],
    //              temp_buffer[5], temp_buffer[6], temp_buffer[7], temp_buffer[8], temp_buffer[9]);
    //     ESP_LOGW(TAG, "decode: Hex[10-19]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
    //              temp_buffer[10], temp_buffer[11], temp_buffer[12], temp_buffer[13], temp_buffer[14],
    //              temp_buffer[15], temp_buffer[16], temp_buffer[17], temp_buffer[18], temp_buffer[19]);
    // }

    // Verify CRC32
    if (received_crc != calculated_crc) {
        ESP_LOGW(TAG, "decode: CRC32 MISMATCH! received=0x%08lX calc=0x%08lX (decoded %d bytes, payload %d bytes)", 
                 (unsigned long)received_crc, (unsigned long)calculated_crc, decoded_len, payload_len);
        return ESP_ERR_INVALID_CRC;
    }

    ESP_LOGI(TAG, "decode: CRC32 OK, payload: %d bytes", payload_len);

    // Copy payload to output
    memcpy(output, temp_buffer, payload_len);
    *output_len = payload_len;

    return ESP_OK;
}
