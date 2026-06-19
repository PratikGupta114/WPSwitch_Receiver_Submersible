#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "mbedtls/base64.h"
#include "UartManager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "string.h"
#include "math.h"
#include "driver/gpio.h"
#include <string>

#define TAG "UartManager.cc"

// Enable verbose chunk detection logging (disable for production)
// Set to 0 to disable byte-level logging, 1 to enable
#define ENABLE_CHUNK_DETECTION_LOGGING 0

#define B64_SIZE(n) ((size_t)(4.0 * ceil((double)n / 3.0)))

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static SemaphoreHandle_t uartInitializationMutex;

// Mutex lock to ensure that no encoded sensor data is sent while
// RF module is in configuration mode.
static SemaphoreHandle_t uartPortAccessMutex;

// Flag to indicate RF module is in configuration mode
// Prevents data transmission from interfering with configuration commands
static volatile bool rfModuleInConfigMode = false;

// Forward declarations for helper functions
char *sendRFModuleQueryCommandAndGetResponse(const char *command, int expectedResponseSize);
bool sendRFModuleConfigCommand(const char *command);
bool isValidChannelFrequency(int freq);

void UartManager::uart_timeout_task_wrapper(void *pvParameters)
{
    UartManager *uartManager = static_cast<UartManager *>(pvParameters);
    uartManager->uartTimeoutTask();
}

void UartManager::uartTimeoutTask()
{
    while (true)
    {
        EventBits_t bits = xEventGroupWaitBits(this->uartTaskEventGroup, UART_TIMEOUT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

        if ((bits & UART_TIMEOUT_BIT) != 0)
        {
            ESP_LOGI(TAG, "UART timeout task triggered.");
            if (this->dataReceptionTimeoutListener != NULL)
            {
                this->dataReceptionTimeoutListener();
            }
        }
    }
}

void UartManager::processReceivedPacket(const uint8_t* data, size_t length)
{
    if (data == NULL || length == 0) {
        ESP_LOGW(TAG, "processReceivedPacket: Invalid data (NULL or zero length)");
        return;
    }

    // ESP_LOGI(TAG, "Processing received packet: %d bytes", length);

    // Allocate buffer for decoded data
    char *decodedData = (char *)malloc(length);
    if (decodedData == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for decoded data");
        return;
    }

    // Call the registered callback
    if (this->onReceiverPortDataReceivedListener != NULL)
    {
        memcpy(decodedData, data, length);
        bool result = this->onReceiverPortDataReceivedListener(decodedData, length);
        if (result == true && this->dataReceptionTimerHandle != NULL)
        {
            // ESP_LOGI(TAG, "Packet processed successfully, restarting watchdog timer");
            xTimerStart(this->dataReceptionTimerHandle, portMAX_DELAY);
        } else {
            ESP_LOGW(TAG, "Packet processing callback returned false");
        }
    } else {
        ESP_LOGW(TAG, "No data reception listener registered");
    }
    
    free(decodedData);
}

void receiverPortDataReceptionTask(void *args)
{
    UartManager *uartManager = (UartManager *)args;
    if (uartManager == NULL) {
        ESP_LOGE(TAG, "UART task: UartManager pointer is NULL");
        vTaskDelete(NULL);
        return;
    }
    
    int consecutiveErrors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 10;

    // Chunk detection variables - track chunks received from uart_read_bytes()
    // Each call to uart_read_bytes() that returns data is considered one "chunk"
    static uint32_t totalChunksInPacket = 0;
    static int64_t packetStartTime = 0;    // microseconds
    [[maybe_unused]] static int64_t lastChunkTime = 0;      // microseconds

    xSemaphoreGive(uartInitializationMutex);

#if ENABLE_CHUNK_DETECTION_LOGGING
    ESP_LOGI(TAG, "UART reception task started with OPTIMIZED CHUNK DETECTION");
    ESP_LOGI(TAG, "Logging at CHUNK level (per uart_read_bytes call), not byte level");
    ESP_LOGW(TAG, "Use esp_log_level_set(\"UartManager.cc\", ESP_LOG_WARN) to reduce verbosity");
#else
    ESP_LOGI(TAG, "UART reception task started with COBS framing protocol");
#endif

    while (1)
    {
        // Check if RF module is in configuration mode
        if (rfModuleInConfigMode)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Read bytes from UART - this returns a "chunk" (what UART FIFO delivers at once)
        // Reading multiple bytes at once is more efficient than byte-by-byte
        uint8_t chunkBuffer[64];  // Read up to 64 bytes per call
        int rxBytes = uart_read_bytes(UART_PORT_FOR_RECEIVER, chunkBuffer, sizeof(chunkBuffer), 10 / portTICK_PERIOD_MS);
        
        if (rxBytes > 0)
        {
            consecutiveErrors = 0;
            
            // Use high-resolution timer for precise gap measurement
            [[maybe_unused]] int64_t currentTime = esp_timer_get_time();  // microseconds since boot
            
#if ENABLE_CHUNK_DETECTION_LOGGING
            // Calculate inter-chunk gap in milliseconds
            int64_t gapMs = 0;
            if (lastChunkTime != 0) {
                gapMs = (currentTime - lastChunkTime) / 1000;  // Convert to ms
            }
            
            // Log chunk reception (ONCE per uart_read_bytes call, not per byte)
            ESP_LOGI(TAG, "UART_CHK[%d]: %dB, gap=%lldms, total=%dB", 
                     totalChunksInPacket + 1, rxBytes, gapMs, 
                     uartManager->rxBufferIndex + rxBytes);
            
            lastChunkTime = currentTime;
#endif

            // Process each byte in the chunk through the FSM
            for (int i = 0; i < rxBytes; i++)
            {
                uint8_t byte = chunkBuffer[i];

            switch (uartManager->rxState)
            {
                case UartManager::RxState::SYNC:
                    // In SYNC state, we're looking for the START of a COBS packet
                    // COBS packets start with a non-zero overhead byte, NOT a delimiter
                    // The 0x00 delimiter marks the END of a packet
                    // So in SYNC, we should start collecting on ANY non-zero byte
                    if (byte != 0x00) {
                        // ESP_LOGI(TAG, "FSM: SYNC → COLLECT (first byte: 0x%02X)", byte);
                        
                        uartManager->rxState = UartManager::RxState::COLLECT;
                        uartManager->rxBufferIndex = 0;
                        uartManager->rxBuffer[uartManager->rxBufferIndex++] = byte;
                        
                        // Initialize packet-level tracking
                        int64_t currentTime = esp_timer_get_time();
                        uartManager->lastByteTime = currentTime;
                        packetStartTime = currentTime;
                        totalChunksInPacket = 1;  // First chunk
                    } else {
                        ESP_LOGD(TAG, "FSM: SYNC - Ignoring delimiter 0x00 (inter-packet gap)");
                    }
                    // If we receive 0x00 in SYNC, just ignore it (inter-packet gap)
                    break;

                case UartManager::RxState::COLLECT:
                    {
                    // Check for timeout
                    int64_t currentTime = esp_timer_get_time();
                    int64_t timeSinceLastByte = currentTime - uartManager->lastByteTime;
                    
                    if (timeSinceLastByte > (PACKET_TIMEOUT_MS * 1000)) {  // Convert ms to us
                        uint32_t timeoutMs = timeSinceLastByte / 1000;
                        ESP_LOGW(TAG, "UART_TIMEOUT: %ums, %dB in %d chunks - DISCARDING", 
                                 timeoutMs, uartManager->rxBufferIndex, totalChunksInPacket);
                        
                        // Hex dump of partial data (only if verbose logging enabled)
#if ENABLE_CHUNK_DETECTION_LOGGING
                        if (uartManager->rxBufferIndex > 0) {
                            ESP_LOG_BUFFER_HEX_LEVEL(TAG, uartManager->rxBuffer, 
                                                     MIN(uartManager->rxBufferIndex, 32), ESP_LOG_WARN);
                        }
#endif
                        
                        // CRITICAL FIX: Clear all state to prevent corruption
                        uartManager->rxState = UartManager::RxState::SYNC;
                        uartManager->rxBufferIndex = 0;
                        totalChunksInPacket = 0;
                        packetStartTime = 0;
                        lastChunkTime = 0;
                        break;
                    }

                    // Check for next delimiter (end of packet)
                    if (byte == 0x00) {
                        // Minimum valid COBS packet size check (sensor packet should be ~25 bytes encoded)
                        // Reject obviously invalid short packets (noise, RF module responses, etc.)
                        const size_t MIN_VALID_PACKET_SIZE = 20;  // Minimum expected encoded size
                        
                        if (uartManager->rxBufferIndex < MIN_VALID_PACKET_SIZE) {
                            ESP_LOGW(TAG, "UART_PKT_SHORT: %dB in %d chunks - DISCARDING (min=%d)", 
                                     uartManager->rxBufferIndex, totalChunksInPacket, MIN_VALID_PACKET_SIZE);
                            
                            // Hex dump of rejected packet (only if verbose)
#if ENABLE_CHUNK_DETECTION_LOGGING
                            ESP_LOG_BUFFER_HEX_LEVEL(TAG, uartManager->rxBuffer, 
                                                     uartManager->rxBufferIndex, ESP_LOG_WARN);
#endif
                            
                            // CRITICAL FIX: Clear all state
                            uartManager->rxState = UartManager::RxState::SYNC;
                            uartManager->rxBufferIndex = 0;
                            totalChunksInPacket = 0;
                            packetStartTime = 0;
                            lastChunkTime = 0;
                            break;
                        }
                        
                        // Add the delimiter to the buffer - espp::Cobs::decode_packet expects it
                        if (uartManager->rxBufferIndex < MAX_ENCODED_SIZE) {
                            uartManager->rxBuffer[uartManager->rxBufferIndex++] = 0x00;
                        }
                        
                        uint8_t decodedBuffer[MAX_PACKET_SIZE];
                        size_t decodedLength = 0;

                        // Decode COBS packet (includes CRC32 verification)
                        esp_err_t decodeResult = uart_framing_decode(uartManager->rxBuffer, uartManager->rxBufferIndex, 
                                                       decodedBuffer, &decodedLength);
                        
                        if (decodeResult == ESP_OK)
                        {
                            int64_t packetDurationMs = (currentTime - packetStartTime) / 1000;
                            
                            // Extract packet type and data
                            if (decodedLength >= 1) {
                                uint8_t packetType = decodedBuffer[0];
                                ESP_LOGD(TAG, "PKT_RX: %dB→%dB, type=0x%02X, %d chunks, %lldms", 
                                         uartManager->rxBufferIndex, decodedLength - 1, 
                                         packetType, totalChunksInPacket, packetDurationMs);
                                
                                if (packetType == PACKET_TYPE_SENSOR_DATA) {
                                    // Process sensor data (skip type byte)
                                    uartManager->processReceivedPacket(&decodedBuffer[1], decodedLength - 1);
                                } else {
                                    ESP_LOGW(TAG, "Unknown packet type: 0x%02X", packetType);
                                }
                            } else {
                                ESP_LOGW(TAG, "Decoded packet too small: %d bytes", decodedLength);
                            }
                        }
                        else
                        {
                            int64_t packetDurationMs = (currentTime - packetStartTime) / 1000;
                            ESP_LOGW(TAG, "UART_PKT_FAIL: %dB in %d chunks, %lldms, err=0x%x", 
                                     uartManager->rxBufferIndex, totalChunksInPacket, 
                                     packetDurationMs, decodeResult);
                            
                            // Hex dump on decode failure
                            ESP_LOG_BUFFER_HEX_LEVEL(TAG, uartManager->rxBuffer, 
                                                     MIN(uartManager->rxBufferIndex, 32), ESP_LOG_WARN);
                            
                            if (decodeResult == ESP_ERR_INVALID_CRC) {
                                ESP_LOGW(TAG, "CRC32 verification failed");
                            }
                        }

                        // Reset FSM - go back to SYNC to wait for next packet start
                        uartManager->rxState = UartManager::RxState::SYNC;
                        uartManager->rxBufferIndex = 0;
                        totalChunksInPacket = 0;
                        packetStartTime = 0;
                        lastChunkTime = 0;
                    } else {
                        // Collect byte
                        if (uartManager->rxBufferIndex < MAX_ENCODED_SIZE - 1) {  // Leave room for delimiter
                            uartManager->rxBuffer[uartManager->rxBufferIndex++] = byte;
                            uartManager->lastByteTime = esp_timer_get_time();
                        } else {
                            ESP_LOGW(TAG, "UART_OVERFLOW: %dB - DISCARDING", uartManager->rxBufferIndex);
                            
                            // CRITICAL FIX: Clear all state on overflow
                            uartManager->rxState = UartManager::RxState::SYNC;
                            uartManager->rxBufferIndex = 0;
                            totalChunksInPacket = 0;
                            packetStartTime = 0;
                            lastChunkTime = 0;
                        }
                    }
                    }
                    break;

                case UartManager::RxState::PROCESS:
                    // This state should not be reached in normal operation
                    // Processing is now done inline in COLLECT when delimiter is received
                    ESP_LOGW(TAG, "FSM: Unexpected PROCESS state, resetting to SYNC");
                    uartManager->rxState = UartManager::RxState::SYNC;
                    uartManager->rxBufferIndex = 0;
                    totalChunksInPacket = 0;
                    packetStartTime = 0;
                    lastChunkTime = 0;
                    break;
            }
            } // End of for loop processing bytes in chunk
        }
        else if (rxBytes < 0)
        {
            consecutiveErrors++;
            ESP_LOGE(TAG, "UART read error: %d (consecutive errors: %d)", rxBytes, consecutiveErrors);
            
            if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS)
            {
                ESP_LOGE(TAG, "Too many consecutive UART errors, attempting recovery...");
                uart_flush(UART_PORT_FOR_RECEIVER);
                vTaskDelay(pdMS_TO_TICKS(1000));
                consecutiveErrors = 0;
                
                // Reset FSM
                uartManager->rxState = UartManager::RxState::SYNC;
                uartManager->rxBufferIndex = 0;
                totalChunksInPacket = 0;
                packetStartTime = 0;
                lastChunkTime = 0;
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        else
        {
            // Timeout - check for packet timeout in COLLECT state
            if (uartManager->rxState == UartManager::RxState::COLLECT)
            {
                int64_t currentTime = esp_timer_get_time();  // Use microseconds for consistency
                int64_t timeSinceLastByte = currentTime - uartManager->lastByteTime;
                
                if (timeSinceLastByte > (PACKET_TIMEOUT_MS * 1000)) {  // Convert ms to us
                    uint32_t timeoutMs = timeSinceLastByte / 1000;  // Convert us to ms for display
                    ESP_LOGW(TAG, "PKT_TIMEOUT: %ums, %dB collected, %d chunks", 
                             timeoutMs, uartManager->rxBufferIndex, totalChunksInPacket);
                    
                    // CRITICAL FIX: Clear buffer to prevent corruption of next packet
                    uartManager->rxState = UartManager::RxState::SYNC;
                    uartManager->rxBufferIndex = 0;  // Reset buffer index
                    totalChunksInPacket = 0;
                    packetStartTime = 0;  // Reset packet start time
                    lastChunkTime = 0;    // Reset chunk time
                }
            }
        }
    }
}

void UartManager::sendData(char *data, size_t length)
{
    if (this->receiverPortInitialized == false)
    {
        ESP_LOGE(TAG, "Receiver uart port needs to be initialized first");
        return;
    }

    // Check if RF module is in configuration mode
    if (rfModuleInConfigMode)
    {
        ESP_LOGW(TAG, "RF module in configuration mode - skipping data transmission to prevent interference");
        return;
    }

    if (data == NULL || length == 0)
    {
        ESP_LOGE(TAG, "Invalid data to send !");
        return;
    }

    // Take mutex before sending data
    if (xSemaphoreTake(uartPortAccessMutex, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGE(TAG, "sendData() -> Failed to take uart port access mutex");
        return;
    }

    ESP_LOGI(TAG, "sendData: Preparing to send %d bytes", length);
    
    // Prepare packet with type byte
    uint8_t packet[MAX_PACKET_SIZE];
    packet[0] = PACKET_TYPE_SENSOR_DATA;
    memcpy(&packet[1], data, length);
    size_t packetLength = length + 1;
    
    ESP_LOGI(TAG, "sendData: Packet with type byte: %d bytes (type=0x%02X)", packetLength, packet[0]);

    // Encode with COBS (includes CRC32)
    uint8_t encodedBuffer[MAX_ENCODED_SIZE];
    size_t encodedLength = 0;
    
    esp_err_t encodeResult = uart_framing_encode(packet, packetLength, encodedBuffer, &encodedLength);
    if (encodeResult != ESP_OK)
    {
        ESP_LOGE(TAG, "sendData: Failed to encode packet (error: 0x%x)", encodeResult);
        xSemaphoreGive(uartPortAccessMutex);
        return;
    }
    
    ESP_LOGI(TAG, "sendData: COBS encode + CRC32 OK, encoded: %d bytes", encodedLength);

    // Send encoded packet
    int bytesWritten = uart_write_bytes(UART_PORT_FOR_RECEIVER, encodedBuffer, encodedLength);
    
    if (bytesWritten < 0)
    {
        ESP_LOGE(TAG, "sendData: UART write error: %d", bytesWritten);
    }
    else if (bytesWritten != (int)encodedLength)
    {
        ESP_LOGW(TAG, "sendData: Partial write - expected: %d, written: %d", encodedLength, bytesWritten);
    }
    else
    {
        ESP_LOGI(TAG, "sendData: TX complete - raw: %d bytes → encoded: %d bytes", length, bytesWritten);
    }

    // Release mutex after sending data
    xSemaphoreGive(uartPortAccessMutex);
}

void dataReceptionTimerCallback(TimerHandle_t xTimer)
{
    // This is called when the uart manager did not receive any data in time.
    if (xTimer == NULL) {
        ESP_LOGE(TAG, "UART timer callback: Timer handle is NULL");
        return;
    }
    
    UartManager *uartManager = (UartManager *)pvTimerGetTimerID(xTimer);
    ESP_LOGE(TAG, "UART TIMER RESET");

    if (uartManager != NULL && uartManager->uartTaskEventGroup != NULL)
    {
        xEventGroupSetBits(uartManager->uartTaskEventGroup, UART_TIMEOUT_BIT);
    }
    else
    {
        ESP_LOGE(TAG, "UART timer callback: UartManager or event group is NULL");
    }
}

void UartManager::initReceiverPort(function<bool(char *, size_t)> onReceiverPortDataReceivedListener)
{
    // Ensure INFO level logging is enabled for UartManager
    // This is critical for chunk detection logging to work
#if ENABLE_CHUNK_DETECTION_LOGGING
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Log level set to INFO for chunk detection diagnostics");
#else
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Log level set to INFO for UartManager");
#endif

    // Initialize mutexes for UART access control
    uartInitializationMutex = xSemaphoreCreateBinary();
    uartPortAccessMutex = xSemaphoreCreateMutex();

    // Initialize WIR-1186 RF module programming mode pin
    gpio_set_direction(WIRELESS_PRG_MODE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(WIRELESS_PRG_MODE_PIN, 1);  // Start in normal mode (HIGH)

    // Uart Configuration
    const uart_config_t uartConfiguration = {
        .baud_rate = RECEIVER_PORT_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };

    xTaskCreatePinnedToCore(
        receiverPortDataReceptionTask,
        "receiver_port_data_reception_task",
        UART_MANAGER_TASK_STACK_DEPTH,
        (void *)this,
        configMAX_PRIORITIES - 1,
        NULL,
        APP_CPU_NUM);

    // Let's yield to the task created recently
    vTaskDelay(5 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_FOR_RECEIVER, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_FOR_RECEIVER, &uartConfiguration));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_FOR_RECEIVER, RECEIVER_PORT_TX, RECEIVER_PORT_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Let us now initialize the timer
    this->dataReceptionTimerHandle = xTimerCreate(
        "uart-data-timer",
        pdMS_TO_TICKS(UART_DATA_RECEPTION_TIMEOUT_SEC * 1000),
        pdTRUE,
        (void *)this, // Pass the pointer to the uart manager instance.
        dataReceptionTimerCallback);

    if (this->dataReceptionTimerHandle == NULL)
    {
        ESP_LOGE(TAG, "Failed to create timer !");
        return;
    }

    xTimerStart(this->dataReceptionTimerHandle, portMAX_DELAY);
    xSemaphoreTake(uartInitializationMutex, portMAX_DELAY);

    this->uartInitialized = true;
    this->receiverPortInitialized = true;
    this->onReceiverPortDataReceivedListener = onReceiverPortDataReceivedListener;

    // gpio_reset_pin(WIRELESS_PRG_MODE_PIN);
    // gpio_set_direction(WIRELESS_PRG_MODE_PIN, GPIO_MODE_OUTPUT);
    // vTaskDelay(20);
    ESP_LOGW(TAG, "Setting PRG pin to LOW");
    gpio_set_level(WIRELESS_PRG_MODE_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGW(TAG, "Setting PRG pin to HIGH");
    gpio_set_level(WIRELESS_PRG_MODE_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGW(TAG, "Sending Command to reset the module !");
    // sendData("X\r", sizeof("X\r"));
    uart_write_bytes(UART_PORT_FOR_RECEIVER, "X\r", sizeof("X\r"));
    ESP_LOGW(TAG, "Command Sent !");
    vTaskDelay(pdMS_TO_TICKS(100));
}

void UartManager::setDataReceptionTimeoutListener(function<void()> dataReceptionTimeoutListener)
{
    this->dataReceptionTimeoutListener = dataReceptionTimeoutListener;
}

void UartManager::resetRadioModule()
{
    gpio_set_direction(WIRELESS_PRG_MODE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(WIRELESS_PRG_MODE_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(UART_PORT_FOR_RECEIVER, "X?\n\r", (sizeof("X?\n\r") / sizeof(char)));
}

bool UartManager::isRFModuleResponding()
{
    if (!receiverPortInitialized)
    {
        ESP_LOGE(TAG, "RF Module Health Check: UART not initialized");
        return false;
    }

    const char *LOCAL_TAG = "isRFModuleResponding()";
    ESP_LOGI(LOCAL_TAG, "Checking RF module health...");

    // Try a simple query to check if RF module is responding
    char *response = sendRFModuleQueryCommandAndGetResponse("A?\r", 10);

    if (response == NULL)
    {
        ESP_LOGE(LOCAL_TAG, "RF module is NOT responding");
        return false;
    }

    ESP_LOGI(LOCAL_TAG, "RF module is responding");
    free(response);
    return true;
}

UartManager::~UartManager()
{
    if (this->dataReceptionTimerHandle != NULL)
        xTimerDelete(this->dataReceptionTimerHandle, portMAX_DELAY);
    if (this->uartTimeoutTaskHandle != NULL)
        vTaskDelete(this->uartTimeoutTaskHandle);
    if (this->uartTaskEventGroup != NULL)
        vEventGroupDelete(this->uartTaskEventGroup);
}

UartManager::UartManager()
{
    this->receiverPortInitialized = false;
    this->onReceiverPortDataReceivedListener = NULL;
    this->dataReceptionTimerHandle = NULL;
    this->uartTaskEventGroup = xEventGroupCreate();
    
    // Initialize FSM state
    this->rxState = RxState::SYNC;
    this->rxBufferIndex = 0;
    this->lastByteTime = 0;
    memset(this->rxBuffer, 0, sizeof(this->rxBuffer));
    
    xTaskCreate(
        uart_timeout_task_wrapper,
        "uart_timeout_task",
        UART_TIMEOUT_TASK_STACK_DEPTH,
        this,
        5,
        &this->uartTimeoutTaskHandle);
}

// Utils functions
const char *getAirDataRateString(RFModuleAirDataRate airDataRate)
{
    switch (airDataRate)
    {
    case DATA_RATE_19_2_KBPS:
        return "DATA_RATE_19_2_KBPS";
    case DATA_RATE_38_4_KBPS:
        return "DATA_RATE_38_4_KBPS";
    case DATA_RATE_76_8_KBPS:
        return "DATA_RATE_76_8_KBPS";
    case DATA_RATE_MAX:
        return "DATA_RATE_MAX";
    default:
        return "INVALID";
    }
    return "";
}
const char *getBaudrateString(RFModuleBaudrate baudRate)
{
    switch (baudRate)
    {
    case BAUD_9600:
        return "BAUD_9600";
    case BAUD_19200:
        return "BAUD_19200";
    case BAUD_38400:
        return "BAUD_38400";
    case BAUD_57600:
        return "BAUD_57600";
    case BAUD_115200:
        return "BAUD_115200";
    case BAUD_MAX:
        return "BAUD_MAX";
    default:
        return "INVALID";
    }
    return "";
}
const char *getCarrierFrequencyString(RFModuleCarrierFrequency frequency)
{
    switch (frequency)
    {
    case FREQ_865MHZ:
        return "FREQ_865MHZ";
    case FREQ_865_5_MHZ:
        return "FREQ_865_5_MHZ";
    case FREQ_866MHZ:
        return "FREQ_866MHZ";
    case FREQ_866_5_MHZ:
        return "FREQ_866_5_MHZ";
    case FREQ_869MHZ:
        return "FREQ_869MHZ";
    case FREQ_MAX:
        return "FREQ_MAX";
    default:
        return "INVALID";
    }
    return "";
}
const char *getPowerTransmitLevelString(RFModulePowerTransmitLevel powerLevel)
{
    switch (powerLevel)
    {
    case POWER_LEVEL_PLUS_17_DB:
        return "POWER_LEVEL_PLUS_17_DB";
    case POWER_LEVEL_PLUS_10_DB:
        return "POWER_LEVEL_PLUS_10_DB";
    case POWER_LEVEL_PLUS_4_DB:
        return "POWER_LEVEL_PLUS_4_DB";
    case POWER_LEVEL_MINUS_2_DB:
        return "POWER_LEVEL_MINUS_2_DB";
    case POWER_LEVEL_MINUS_8_DB:
        return "POWER_LEVEL_MINUS_8_DB";
    case POWER_LEVEL_MINUS_14_DB:
        return "POWER_LEVEL_MINUS_14_DB";
    case POWER_LEVEL_MINUS_20_DB:
        return "POWER_LEVEL_MINUS_20_DB";
    case POWER_LEVEL_MINUS_24_DB:
        return "POWER_LEVEL_MINUS_24_DB";
    case POWER_LEVEL_MAX:
        return "POWER_LEVEL_MAX";
    default:
        return "INVALID";
    }
    return "";
}
const char *getSignalStrengthLimitString(RFModuleSignalStrengthLimit strengthLimit)
{
    switch (strengthLimit)
    {
    case SIGNAL_STRENGTH_MINUS_100_DBM:
        return "SIGNAL_STRENGTH_MINUS_100_DBM";
    case SIGNAL_STRENGTH_MINUS_90_DBM:
        return "SIGNAL_STRENGTH_MINUS_90_DBM";
    case SIGNAL_STRENGTH_MINUS_80_DBM:
        return "SIGNAL_STRENGTH_MINUS_80_DBM";
    case SIGNAL_STRENGTH_MINUS_70_DBM:
        return "SIGNAL_STRENGTH_MINUS_70_DBM";
    case SIGNAL_STRENGTH_MINUS_60_DBM:
        return "SIGNAL_STRENGTH_MINUS_60_DBM";
    case SIGNAL_STRENGTH_MINUS_50_DBM:
        return "SIGNAL_STRENGTH_MINUS_50_DBM";
    case SIGNAL_STRENGTH_MINUS_40_DBM:
        return "SIGNAL_STRENGTH_MINUS_40_DBM";
    case SIGNAL_STRENGTH_MINUS_30_DBM:
        return "SIGNAL_STRENGTH_MINUS_30_DBM";
    case SIGNAL_STRENGTH_MAX:
        return "SIGNAL_STRENGTH_MAX";
    default:
        return "INVALID";
    }
    return "";
}

char *readStringUntil(char terminator, char *buffer, int bufferSize)
{

    int index = 0;
    uint8_t data;

    while (true)
    {
        // Use 2-second timeout instead of blocking forever
        int len = uart_read_bytes(UART_PORT_FOR_RECEIVER, &data, 1, pdMS_TO_TICKS(2000));
        if (len > 0)
        {

            if (data == terminator)
            {
                buffer[index] = '\0';
                ESP_LOGI("readStringUntil", "Encountered terminating char %c | read %d bytes", terminator, index);
                return buffer;
            }
            else
            {
                buffer[index++] = data;
                if (index >= bufferSize - 1)
                {
                    buffer[index] = '\0';
                    ESP_LOGW("readStringUntil", "Buffer Overflow ! String recorded : %s", buffer);
                    return buffer;
                }
            }
        }
        else if (len == 0)
        {
            // Timeout - no data received within 2 seconds
            ESP_LOGE("readStringUntil", "UART read timeout - RF module not responding (no data received within 2 seconds)");
            return NULL;
        }
        else
        {
            // Error (len < 0)
            ESP_LOGE("readStringUntil", "UART read error (code: %d)", len);
            return NULL;
        }
    }
}

// Helper function to validate channel frequency values
bool isValidChannelFrequency(int freq)
{
    return (freq >= FREQ_865MHZ && freq < FREQ_MAX);
}

// Helper function to enter RF module configuration mode
void enterRFConfigMode()
{
    const char *LOCAL_TAG = "enterRFConfigMode()";
    
    // Flush UART input buffer first
    ESP_ERROR_CHECK(uart_flush_input(UART_PORT_FOR_RECEIVER));
    vTaskDelay(pdMS_TO_TICKS(2));
    
    ESP_LOGI(LOCAL_TAG, "Setting RF Module to Configuration mode...");
    
    // Create a LOW pulse of 10ms to enable configuration mode in the WIR-1186 module
    // First set HIGH, then LOW, then back to HIGH
    ESP_ERROR_CHECK(gpio_set_level(WIRELESS_PRG_MODE_PIN, 1));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(gpio_set_level(WIRELESS_PRG_MODE_PIN, 0));
    vTaskDelay(pdMS_TO_TICKS(50));
}

// Helper function to exit RF module configuration mode
void exitRFConfigMode()
{
    const char *LOCAL_TAG = "exitRFConfigMode()";
    
    ESP_LOGI(LOCAL_TAG, "Exiting from RF Module Configuration mode...");
    
    // Exit configuration mode
    ESP_ERROR_CHECK(gpio_set_level(WIRELESS_PRG_MODE_PIN, 1));
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_write_bytes(UART_PORT_FOR_RECEIVER, "X\r", strlen("X\r"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Flush all the data in the output buffer
    ESP_ERROR_CHECK(uart_flush(UART_PORT_FOR_RECEIVER));
}

// Helper function to send RF module query command and get response
char *sendRFModuleQueryCommandAndGetResponse(const char *command, int expectedResponseSize)
{
    const char *LOCAL_TAG = "sendRFModuleQueryCommandAndGetResponse()";

    // Take mutex to prevent data transmission interference
    if (xSemaphoreTake(uartPortAccessMutex, pdMS_TO_TICKS(5000)) != pdTRUE)
    {
        ESP_LOGE(LOCAL_TAG, "Failed to take uart port access mutex");
        return NULL;
    }

    // Set flag to indicate RF module is in configuration mode
    rfModuleInConfigMode = true;

    // Enter RF module configuration mode
    enterRFConfigMode();

    // Step 1: Read and discard the initial prompt (ends with ':')
    uint8_t BUFFER_SIZE = 40;
    char *promptBuffer = (char *)malloc(BUFFER_SIZE * sizeof(char));
    if (promptBuffer == NULL)
    {
        ESP_LOGE(LOCAL_TAG, "Failed to allocate memory for prompt buffer");
        exitRFConfigMode();
        rfModuleInConfigMode = false;
        xSemaphoreGive(uartPortAccessMutex);
        return NULL;
    }
    
    char *prompt = readStringUntil(':', promptBuffer, BUFFER_SIZE);
    if (prompt != NULL)
    {
        ESP_LOGI(LOCAL_TAG, "Received prompt: %s", prompt);
        free(promptBuffer);
    }
    else
    {
        ESP_LOGE(LOCAL_TAG, "No prompt received from RF Module");
        free(promptBuffer);
    }

    // Step 2: Flush input buffer and send command
    uart_flush_input(UART_PORT_FOR_RECEIVER);
    int bytesWritten = uart_write_bytes(UART_PORT_FOR_RECEIVER, command, strlen(command));
    if (bytesWritten < 0)
    {
        ESP_LOGE(LOCAL_TAG, "Failed to write command to UART");
        exitRFConfigMode();
        rfModuleInConfigMode = false;
        xSemaphoreGive(uartPortAccessMutex);
        return NULL;
    }

    ESP_LOGI(LOCAL_TAG, "Command sent: %s (bytes: %d)", command, bytesWritten);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Step 3: Read response (ends with ':')
    BUFFER_SIZE = expectedResponseSize;
    char *responseBuffer = (char *)malloc(BUFFER_SIZE * sizeof(char));
    if (responseBuffer == NULL)
    {
        ESP_LOGE(LOCAL_TAG, "Failed to allocate memory for response buffer");
        exitRFConfigMode();
        rfModuleInConfigMode = false;
        xSemaphoreGive(uartPortAccessMutex);
        return NULL;
    }

    memset(responseBuffer, 0, BUFFER_SIZE);
    char *fullResponse = readStringUntil(':', responseBuffer, BUFFER_SIZE);

    if (fullResponse == NULL)
    {
        ESP_LOGE(LOCAL_TAG, "Failed to read response from RF module");
        free(responseBuffer);
        exitRFConfigMode();
        rfModuleInConfigMode = false;
        xSemaphoreGive(uartPortAccessMutex);
        return NULL;
    }

    ESP_LOGI(LOCAL_TAG, "Full response received: %s", fullResponse);

    // Step 4: Parse the response to extract data between '?' and '\r' or '\n'
    const char *start = strchr(fullResponse, '?');
    if (start == NULL)
    {
        ESP_LOGE(LOCAL_TAG, "Invalid response from RF module: '?' not present!");
        free(responseBuffer);
        exitRFConfigMode();
        rfModuleInConfigMode = false;
        xSemaphoreGive(uartPortAccessMutex);
        return NULL;
    }
    
    // Move one character past the '?'
    start++;

    // Look for either '\r' or '\n' as line terminator
    const char *end = strchr(start, '\r');
    if (end == NULL)
    {
        end = strchr(start, '\n');
    }
    
    if (end == NULL)
    {
        ESP_LOGE(LOCAL_TAG, "Invalid response from RF module: line terminator not present!");
        free(responseBuffer);
        exitRFConfigMode();
        rfModuleInConfigMode = false;
        xSemaphoreGive(uartPortAccessMutex);
        return NULL;
    }

    int dataLength = end - start;
    if (dataLength <= 0)
    {
        ESP_LOGE(LOCAL_TAG, "Received an empty string as response!");
        free(responseBuffer);
        exitRFConfigMode();
        rfModuleInConfigMode = false;
        xSemaphoreGive(uartPortAccessMutex);
        return NULL;
    }

    // Step 5: Extract the actual data
    char *data = (char *)malloc(dataLength + 1);
    if (data == NULL)
    {
        ESP_LOGE(LOCAL_TAG, "Memory allocation (%d bytes) for storing data failed!", (dataLength + 1));
        free(responseBuffer);
        exitRFConfigMode();
        rfModuleInConfigMode = false;
        xSemaphoreGive(uartPortAccessMutex);
        return NULL;
    }

    strncpy(data, start, dataLength);
    data[dataLength] = '\0';
    
    ESP_LOGI(LOCAL_TAG, "Data extracted from response: %s", data);
    
    // Free the full response buffer
    free(responseBuffer);

    // Exit RF module configuration mode
    exitRFConfigMode();

    // Clear flag - RF module is no longer in configuration mode
    rfModuleInConfigMode = false;

    // Release mutex
    xSemaphoreGive(uartPortAccessMutex);

    return data;
}

// Helper function to send RF module configuration command
bool sendRFModuleConfigCommand(const char *command)
{
    const char *LOCAL_TAG = "sendRFModuleConfigCommand()";

    // Take mutex to prevent data transmission interference
    if (xSemaphoreTake(uartPortAccessMutex, pdMS_TO_TICKS(5000)) != pdTRUE)
    {
        ESP_LOGE(LOCAL_TAG, "Failed to take uart port access mutex");
        return false;
    }

    // Set flag to indicate RF module is in configuration mode
    rfModuleInConfigMode = true;

    // Enter RF module configuration mode
    enterRFConfigMode();

    // Step 1: Read and discard the initial prompt (ends with ':')
    uint8_t BUFFER_SIZE = 40;
    char *promptBuffer = (char *)malloc(BUFFER_SIZE * sizeof(char));
    if (promptBuffer == NULL)
    {
        ESP_LOGE(LOCAL_TAG, "Failed to allocate memory for prompt buffer");
        exitRFConfigMode();
        rfModuleInConfigMode = false;
        xSemaphoreGive(uartPortAccessMutex);
        return false;
    }
    
    char *prompt = readStringUntil(':', promptBuffer, BUFFER_SIZE);
    if (prompt != NULL)
    {
        ESP_LOGI(LOCAL_TAG, "Received prompt: %s", prompt);
        free(promptBuffer);
    }
    else
    {
        ESP_LOGE(LOCAL_TAG, "No prompt received from RF Module");
        free(promptBuffer);
    }

    // Step 2: Send command to RF module
    int bytesWritten = uart_write_bytes(UART_PORT_FOR_RECEIVER, command, strlen(command));
    if (bytesWritten < 0)
    {
        ESP_LOGE(LOCAL_TAG, "Failed to write command to UART");
        exitRFConfigMode();
        rfModuleInConfigMode = false;
        xSemaphoreGive(uartPortAccessMutex);
        return false;
    }

    ESP_LOGI(LOCAL_TAG, "Config command sent: %s (bytes: %d)", command, bytesWritten);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Step 3: Read response (ends with ':')
    BUFFER_SIZE = strlen(command) + strlen("\nOK\n:") + 2;
    char *responseBuffer = (char *)malloc(BUFFER_SIZE * sizeof(char));
    if (responseBuffer == NULL)
    {
        ESP_LOGE(LOCAL_TAG, "Failed to allocate memory for response buffer");
        exitRFConfigMode();
        rfModuleInConfigMode = false;
        xSemaphoreGive(uartPortAccessMutex);
        return false;
    }
    
    memset(responseBuffer, 0, BUFFER_SIZE);
    char *response = readStringUntil(':', responseBuffer, BUFFER_SIZE);

    bool isCommandSent = false;
    if (response != NULL)
    {
        // Check if the response contains "OK"
        isCommandSent = (strstr(response, "OK") != NULL);
        ESP_LOGI(LOCAL_TAG, "Response received: %s (Success: %d)", response, isCommandSent);
        free(responseBuffer);
    }
    else
    {
        ESP_LOGE(LOCAL_TAG, "No response received from RF Module");
        free(responseBuffer);
    }

    // Exit RF module configuration mode
    exitRFConfigMode();

    // Clear flag - RF module is no longer in configuration mode
    rfModuleInConfigMode = false;

    // Release mutex
    xSemaphoreGive(uartPortAccessMutex);

    return isCommandSent;
}


// Getters

RFModuleAirDataRate UartManager::getRFModuleAirDataRate()
{
    if (!receiverPortInitialized)
        return DATA_RATE_MAX;

    const char *LOCAL_TAG = "getRFModuleAirDataRate()";
    char *responseData = sendRFModuleQueryCommandAndGetResponse("A?\r", 10);

    if (responseData == NULL)
        return DATA_RATE_MAX;

    int num = (int)strtol(responseData, NULL, 16);  // Parse as hexadecimal
    if (num >= 0 && num < DATA_RATE_MAX)
    {
        RFModuleAirDataRate airDataRate = (RFModuleAirDataRate)num;
        ESP_LOGI(LOCAL_TAG, "Air Data Rate Configured : %s", getAirDataRateString(airDataRate));
        free(responseData);
        return airDataRate;
    }
    else
    {
        ESP_LOGE(LOCAL_TAG, "Invalid air data rate value: %d", num);
        free(responseData);
    }

    return DATA_RATE_MAX;
}
RFModuleBaudrate UartManager::getRFModuleBaudrate()
{
    if (!receiverPortInitialized)
        return BAUD_MAX;

    const char *LOCAL_TAG = "getRFModuleBaudrate()";
    char *responseData = sendRFModuleQueryCommandAndGetResponse("B?\r", 10);

    if (responseData == NULL)
        return BAUD_MAX;

    int num = (int)strtol(responseData, NULL, 16);  // Parse as hexadecimal
    if (num >= 0 && num < BAUD_MAX)
    {
        RFModuleBaudrate baudRate = (RFModuleBaudrate)num;
        ESP_LOGI(LOCAL_TAG, "Baud Rate Configured : %s", getBaudrateString(baudRate));
        free(responseData);
        return baudRate;
    }
    else
    {
        ESP_LOGE(LOCAL_TAG, "Invalid baud rate value: %d", num);
        free(responseData);
    }

    return BAUD_MAX;
}
RFModuleCarrierFrequency UartManager::getRFModuleCarrierFrequency()
{
    if (!receiverPortInitialized)
        return FREQ_MAX;

    const char *LOCAL_TAG = "getRFModuleCarrierFrequency()";
    char *responseData = sendRFModuleQueryCommandAndGetResponse("C?\r", 10);

    if (responseData == NULL)
        return FREQ_MAX;

    int num = (int)strtol(responseData, NULL, 16);  // Parse as hexadecimal
    if (isValidChannelFrequency(num))
    {
        RFModuleCarrierFrequency frequency = (RFModuleCarrierFrequency)num;
        ESP_LOGI(LOCAL_TAG, "Carrier frequency configured : %s", getCarrierFrequencyString(frequency));
        free(responseData);
        return frequency;
    }
    else
    {
        ESP_LOGE(LOCAL_TAG, "Invalid carrier frequency value: %d", num);
        free(responseData);
    }

    return FREQ_MAX;
}
RFModulePowerTransmitLevel UartManager::getRFModulePowerTransmitLevel()
{
    if (!receiverPortInitialized)
        return POWER_LEVEL_MAX;

    const char *LOCAL_TAG = "getRFModulePowerTransmitLevel()";
    char *responseData = sendRFModuleQueryCommandAndGetResponse("P?\r", 10);

    if (responseData == NULL)
        return POWER_LEVEL_MAX;

    int num = (int)strtol(responseData, NULL, 16);  // Parse as hexadecimal
    if (num >= 0 && num < POWER_LEVEL_MAX)
    {
        RFModulePowerTransmitLevel powerLevel = (RFModulePowerTransmitLevel)num;
        ESP_LOGI(LOCAL_TAG, "Power Transmit Level Configured : %s", getPowerTransmitLevelString(powerLevel));
        free(responseData);
        return powerLevel;
    }
    else
    {
        ESP_LOGE(LOCAL_TAG, "Invalid power level value: %d", num);
        free(responseData);
    }

    return POWER_LEVEL_MAX;
}
RFModuleSignalStrengthLimit UartManager::getRFModuleSignalStrengthLimit()
{
    if (!receiverPortInitialized)
        return SIGNAL_STRENGTH_MAX;

    const char *LOCAL_TAG = "getRFModuleSignalStrengthLimit()";
    char *responseData = sendRFModuleQueryCommandAndGetResponse("R?\r", 10);

    if (responseData == NULL)
        return SIGNAL_STRENGTH_MAX;

    int num = (int)strtol(responseData, NULL, 16);  // Parse as hexadecimal
    if (num >= 0 && num < SIGNAL_STRENGTH_MAX)
    {
        RFModuleSignalStrengthLimit signalStrength = (RFModuleSignalStrengthLimit)num;
        ESP_LOGI(LOCAL_TAG, "Signal Strength Limit Configured : %s", getSignalStrengthLimitString(signalStrength));
        free(responseData);
        return signalStrength;
    }
    else
    {
        ESP_LOGE(LOCAL_TAG, "Invalid signal strength value: %d", num);
        free(responseData);
    }

    return SIGNAL_STRENGTH_MAX;
}

uint16_t UartManager::getRFModuleHardwareID()
{
    if (!receiverPortInitialized)
        return 0xFFFF;

    const char *LOCAL_TAG = "getRFModuleHardwareID()";
    char *responseData = sendRFModuleQueryCommandAndGetResponse("S?\r", 10);

    if (responseData == NULL)
        return 0xFFFF;

    uint16_t num = (uint16_t)strtol(responseData, NULL, 16);
    ESP_LOGI(LOCAL_TAG, "HardwareID : 0x%04X", num);
    free(responseData);
    return num;
}
uint16_t UartManager::getRFModuleNetworkID()
{
    if (!receiverPortInitialized)
        return 0xFFFF;

    const char *LOCAL_TAG = "getRFModuleNetworkID()";
    char *responseData = sendRFModuleQueryCommandAndGetResponse("N?\r", 10);

    if (responseData == NULL)
        return 0xFFFF;

    uint16_t num = (uint16_t)strtol(responseData, NULL, 16);
    ESP_LOGI(LOCAL_TAG, "NetworkID : 0x%04X", num);
    free(responseData);
    return num;
}
uint16_t UartManager::getRFModuleDestinationID()
{
    if (!receiverPortInitialized)
        return 0xFFFF;

    const char *LOCAL_TAG = "getRFModuleDestinationID()";
    char *responseData = sendRFModuleQueryCommandAndGetResponse("D?\r", 10);

    if (responseData == NULL)
        return 0xFFFF;

    uint16_t num = (uint16_t)strtol(responseData, NULL, 16);
    ESP_LOGI(LOCAL_TAG, "DestinationID : 0x%04X", num);
    free(responseData);
    return num;
}
char *UartManager::getRFModule128bitEncryptionKey()
{
    if (!receiverPortInitialized)
        return NULL;

    const char *LOCAL_TAG = "getRFModule128bitEncryptionKey()";
    char *responseData = sendRFModuleQueryCommandAndGetResponse("K?\r", 40);

    if (responseData == NULL)
        return NULL;

    ESP_LOGI(LOCAL_TAG, "128 bit Encryption Key : %s", responseData);
    return responseData;
}


// Setters

void UartManager::setRFModuleAirDataRate(RFModuleAirDataRate airDataRate)
{
    if (!receiverPortInitialized)
        return;

    const char *LOCAL_TAG = "setRFModuleAirDataRate()";

    // Construct a command string to be sent to the RF module.
    char commandStr[8];
    snprintf(commandStr, 8, "A=%02d\r", (uint8_t)airDataRate);

    bool isCommandSent = sendRFModuleConfigCommand(commandStr);
    if (isCommandSent)
        ESP_LOGW(LOCAL_TAG, "Air Data Rate Set Successfully !");
    else
        ESP_LOGE(LOCAL_TAG, "Failed to set RF module's air data rate !");
}
void UartManager::setRFModuleBaudrate(RFModuleBaudrate baudrate)
{
    if (!receiverPortInitialized)
        return;

    const char *LOCAL_TAG = "setRFModuleBaudrate()";

    // Construct a command string to be sent to the RF module.
    char commandStr[8];
    snprintf(commandStr, 8, "B=%02d\r", (uint8_t)baudrate);

    bool isCommandSent = sendRFModuleConfigCommand(commandStr);
    if (isCommandSent)
        ESP_LOGW(LOCAL_TAG, "Baud rate Set Successfully !");
    else
        ESP_LOGE(LOCAL_TAG, "Failed to set RF module's Baud rate !");
}
void UartManager::setRFModuleCarrierFrequency(RFModuleCarrierFrequency carrierFrequency)
{
    if (!receiverPortInitialized)
        return;

    const char *LOCAL_TAG = "setRFModuleCarrierFrequency()";

    // Construct a command string to be sent to the RF module.
    char commandStr[8];
    snprintf(commandStr, 8, "C=%02d\r", (uint8_t)carrierFrequency);

    bool isCommandSent = sendRFModuleConfigCommand(commandStr);
    if (isCommandSent)
        ESP_LOGW(LOCAL_TAG, "Carrier Frequency Set Successfully !");
    else
        ESP_LOGE(LOCAL_TAG, "Failed to set RF module's Carrier Frequency !");
}
void UartManager::setRFModulePowerTransmitLevel(RFModulePowerTransmitLevel powerTransmitLevel)
{
    if (!receiverPortInitialized)
        return;

    const char *LOCAL_TAG = "setRFModulePowerTransmitLevel()";

    // Construct a command string to be sent to the RF module.
    char commandStr[8];
    snprintf(commandStr, 8, "P=%02d\r", (uint8_t)powerTransmitLevel);

    bool isCommandSent = sendRFModuleConfigCommand(commandStr);
    if (isCommandSent)
        ESP_LOGW(LOCAL_TAG, "Transmit Power Level Set Successfully !");
    else
        ESP_LOGE(LOCAL_TAG, "Failed to set RF module's Transmit Power Level !");
}
void UartManager::setRFModuleSignalStrengthLimit(RFModuleSignalStrengthLimit signalStrengthLimit)
{
    if (!receiverPortInitialized)
        return;

    const char *LOCAL_TAG = "setRFModuleSignalStrengthLimit()";

    // Construct a command string to be sent to the RF module.
    char commandStr[8];
    snprintf(commandStr, 8, "R=%02d\r", (uint8_t)signalStrengthLimit);

    bool isCommandSent = sendRFModuleConfigCommand(commandStr);
    if (isCommandSent)
        ESP_LOGW(LOCAL_TAG, "Signal Strength Level Set Successfully !");
    else
        ESP_LOGE(LOCAL_TAG, "Failed to set RF module's Signal Strength Level !");
}

void UartManager::setRFModuleHardwareID(uint16_t hardwareID)
{
    if (!receiverPortInitialized)
        return;

    const char *LOCAL_TAG = "setRFModuleHardwareID()";

    // TODO - Add validity checks for the hardwareID
    if (hardwareID == 0xFFFF)
        return;

    // Construct a command string to be sent to the RF module.
    char commandStr[10];
    snprintf(commandStr, 10, "S=%04X\r", hardwareID);

    bool isCommandSent = sendRFModuleConfigCommand(commandStr);
    if (isCommandSent)
        ESP_LOGW(LOCAL_TAG, "HardwareID Set Successfully !");
    else
        ESP_LOGE(LOCAL_TAG, "Failed to set RF module's HardwareID !");
}
void UartManager::setRFModuleNetworkID(uint16_t networkID)
{
    if (!receiverPortInitialized)
        return;

    const char *LOCAL_TAG = "setRFModuleNetworkID()";

    // TODO - Add validity checks for the NetworkID
    if (networkID == 0xFFFF)
        return;

    // Construct a command string to be sent to the RF module.
    char commandStr[10];
    snprintf(commandStr, 10, "N=%04X\r", networkID);

    bool isCommandSent = sendRFModuleConfigCommand(commandStr);
    if (isCommandSent)
        ESP_LOGW(LOCAL_TAG, "NetworkID Set Successfully !");
    else
        ESP_LOGE(LOCAL_TAG, "Failed to set RF module's NetworkID !");
}
void UartManager::setRFModuleDestinationID(uint16_t destinationID)
{
    if (!receiverPortInitialized)
        return;

    const char *LOCAL_TAG = "setRFModuleNetworkID()";

    // TODO - Add validity checks for the DestinationID
    if (destinationID == 0xFFFF)
        return;

    // Construct a command string to be sent to the RF module.
    char commandStr[10];
    snprintf(commandStr, 10, "D=%04X\r", destinationID);

    bool isCommandSent = sendRFModuleConfigCommand(commandStr);
    if (isCommandSent)
        ESP_LOGW(LOCAL_TAG, "DestinationID Set Successfully !");
    else
        ESP_LOGE(LOCAL_TAG, "Failed to set RF module's DestinationID !");
}
void UartManager::setRFModule128bitEncryptionKey(char *encryptionKey)
{
    if (!receiverPortInitialized)
        return;

    const char *LOCAL_TAG = "setRFModule128bitEncryptionKey()";

    // TODO - Add validity checks for the encryption Key
    int keyLength = strlen(encryptionKey);
    if (keyLength != 32)
    {
        ESP_LOGE(LOCAL_TAG, "Encryption key should be at least 32 characters long. Key length : %d", keyLength);
        return;
    }

    for (int i = 0; i < keyLength; i++)
    {
        if (!isxdigit(encryptionKey[i]))
        {
            ESP_LOGE(LOCAL_TAG, "Encryption key should contain valid hex string. Invalid character found at index %d", i);
            return;
        }
    }

    // Construct a command string to be sent to the RF module.
    char commandStr[40];
    snprintf(commandStr, 40, "K=%s\r", encryptionKey);

    bool isCommandSent = sendRFModuleConfigCommand(commandStr);
    if (isCommandSent)
        ESP_LOGW(LOCAL_TAG, "128-bit Encryption Key set Successfully !");
    else
        ESP_LOGE(LOCAL_TAG, "128-bit Encryption set RF module's DestinationID !");
}