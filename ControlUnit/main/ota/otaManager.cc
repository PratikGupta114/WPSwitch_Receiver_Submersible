#include "otaManager.h"
#include <inttypes.h>

#define START_UPDATE_CHECK_BIT (1 << 0)
#include "esp_app_format.h"
#include "string.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include <regex>
#include "esp_task_wdt.h"
#include "math.h"
#include <iostream>

#define TAG "OTAManager.cc"

using namespace std;

static char *get_sha256_string(const uint8_t *image_hash)
{
    char *hash_print = (char *)malloc(HASH_LEN * 2 + 1);
    hash_print[HASH_LEN * 2] = '\0';
    for (int i = 0; i < HASH_LEN; ++i)
    {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    return hash_print;
    // ESP_LOGW(TAG, "%s", hash_print);
}

static void printImageHeader(esp_image_header_t *imageHeader)
{
    ESP_LOGW(TAG,
             "\n------------------------------------ Image Header ------------------------------------\n"
             "{\n"
             "\t\"magic\" : \"0x%02x\",\n"
             "\t\"segment_count\" : %" PRIu8 ",\n"
             "\t\"spi_mode\" : %" PRIu8 ",\n"
             "\t\"spi_speed\" : %" PRIu8 ",\n"
             "\t\"spi_size\" : %" PRIu8 ",\n"
             "\t\"entry_addr\" : \"0x%08lx\",\n"
             "\t\"wp_pin\" : %" PRIu8 ",\n"
             "\t\"spi_pin_drv\" : [\"0x%02x\", \"0x%02x\", \"0x%02x\"],\n"
             "\t\"chip_id\" : \"0x%04x\",\n"
             "\t\"min_chip_rev\" : \"0x%02x\",\n"
             "\t\"reserved\" : [\"0x%02x\", \"0x%02x\", \"0x%02x\", \"0x%02x\", \"0x%02x\", \"0x%02x\", \"0x%02x\", \"0x%02x\"],\n"
             "\t\"hash_appended\" : %" PRIu8 "\n"
             "}"
             "\n--------------------------------------------------------------------------------------\n",
             imageHeader->magic,
             imageHeader->segment_count,
             imageHeader->spi_mode,
             imageHeader->spi_speed,
             imageHeader->spi_size,
             imageHeader->entry_addr,
             imageHeader->wp_pin,
             (imageHeader->spi_pin_drv[0]), (imageHeader->spi_pin_drv[1]), (imageHeader->spi_pin_drv[2]),
             imageHeader->chip_id,
             imageHeader->min_chip_rev,
             (imageHeader->reserved[0]), (imageHeader->reserved[1]), (imageHeader->reserved[2]), (imageHeader->reserved[3]),
             (imageHeader->reserved[4]), (imageHeader->reserved[5]), (imageHeader->reserved[6]), (imageHeader->reserved[7]),
             imageHeader->hash_appended);
}

static void printImageSegmentHeader(esp_image_segment_header_t *imageSegmentHeader)
{
    ESP_LOGW(TAG,
             "\n-------------------------------- Image Segment Header --------------------------------\n"
             "{\n"
             "\t\"load_addr\" : \"0x%08lx\",\n"
             "\t\"data_len\" : %" PRIu32 "\n"
             "}"
             "\n--------------------------------------------------------------------------------------\n",
             imageSegmentHeader->load_addr,
             imageSegmentHeader->data_len);
}

static void printAppDescription(esp_app_desc_t *appDescription)
{
    char *hashString = get_sha256_string(appDescription->app_elf_sha256);
    ESP_LOGW(TAG,
             "\n------------------------------------ App Description ------------------------------------\n"
             "{\n"
             "\t\"magic_word\" : \"0x%08" PRIX32 "\",\n"
             "\t\"secure_version\" : \"0x%08" PRIX32 "\",\n"
             "\t\"reserv1\" : [\"0x%08" PRIX32 "\", \"0x%08" PRIX32 "\"],\n"
             "\t\"version\" : \"%s\",\n"
             "\t\"project_name\" : \"%s\",\n"
             "\t\"time\" : \"%s\",\n"
             "\t\"date\" : \"%s\",\n"
             "\t\"idf_ver\" : \"%s\",\n"
             "\t\"app_elf_sha256\" : \"%s\",\n"
             "\t\"reserv2\" : [see below]\n"
             "}\n"
             "\n--------------------------------------------------------------------------------------\n",
             (uint32_t)appDescription->magic_word,
             (uint32_t)appDescription->secure_version,
             (uint32_t)appDescription->reserv1[0], (uint32_t)appDescription->reserv1[1],
             appDescription->version,
             appDescription->project_name,
             appDescription->time,
             appDescription->date,
             appDescription->idf_ver,
             hashString);
    for (int i = 0; i < 20; ++i)
    {
        ESP_LOGW(TAG, "reserv2[%d] = 0x%08" PRIX32, i, (uint32_t)appDescription->reserv2[i]);
    }
    free(hashString);
}

// COURTESY - https://www.youtube.com/watch?v=t_vM_8TLjFE
// COURTESY - https://stackoverflow.com/questions/1716296/why-does-printf-not-flush-after-the-call-unless-a-newline-is-in-the-format-strin
void printOTAProgress(uint8_t progress)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    cout << "\rOTA in progress : | ";
    int numChars = (progress * 70) / 100;
    // Print the completed progress.
    for (int i = 1; i <= numChars; i++)
    {
        cout << "█";
    }
    // Print the in complete progress.
    for (int i = 70 - numChars; i > 0; i--)
    {
        cout << " ";
    }
    cout << " | " << progress << " Done !";
    // setvbuf(stdout, NULL, _IONBF, 0);
    fflush(stdout);
}

// TODO - update the task comments and complete the code.
void otaTask(void *params)
{
    OTAManager *otaManager = (OTAManager *)params;
    if (otaManager == NULL) {
        ESP_LOGE(TAG, "OTA task: OTAManager pointer is NULL");
        vTaskDelete(NULL);
        return;
    }
    
    int hours = (int)(otaManager->updateCheckIntervalMillis / 3600000);
    ESP_LOGI(TAG, "Starting OTA task | Will run every %d hour(s)", hours);
    for (;;)
    {
        EventBits_t bits = xEventGroupWaitBits(otaManager->otaEventGroup, START_UPDATE_CHECK_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

        if ((bits & START_UPDATE_CHECK_BIT) == 0)
        {
            ESP_LOGE(TAG, "OTA event group wait failed");
            continue;
        }
        
        if (otaManager->httpsClient == NULL) {
            ESP_LOGE(TAG, "OTA task: HttpsClient pointer is NULL");
            otaManager->abortOTA("HTTPS client not initialized");
            continue;
        }
        
        HttpsClient *httpsClient = (HttpsClient *)otaManager->httpsClient;
        // Step 1. update the bool variable
        otaManager->isUpdateCheckRunning = true;

        size_t freeHeapSize = xPortGetFreeHeapSize();
        size_t minFreeHeap = esp_get_minimum_free_heap_size();
        ESP_LOGI(TAG, "Heap before OTA version check: free=%zu bytes, min_free=%zu bytes", freeHeapSize, minFreeHeap);

        // Heap integrity check before HTTPS request (helps debug heap corruption)
        if (!heap_caps_check_integrity_all(true)) {
            ESP_LOGE(TAG, "Heap corruption detected BEFORE version check request!");
            otaManager->abortOTA("Heap corruption before version check");
            continue;
        }

        // Note: MQTT disconnect before OTA was removed - the configuration changes
        // (increased WiFi RX buffers, disabled peer cert caching, increased stack)
        // should be sufficient to prevent heap corruption during concurrent TLS.
        // If crashes return, re-enable the MQTT disconnect by uncommenting below:
        // if (otaManager->onBeforeHttpsRequestListener != NULL) {
        //     ESP_LOGW(TAG, "Stopping MQTT before OTA HTTPS request to free TLS resources...");
        //     otaManager->onBeforeHttpsRequestListener();
        //     vTaskDelay(pdMS_TO_TICKS(500));
        // }

        // Log stack high water mark before TLS operation (helps diagnose stack issues)
        UBaseType_t stackBefore = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG, "Stack high water mark before HTTPS request: %u bytes", (unsigned int)stackBefore);

        // Step 2. check for a new version by making a call to the rest api
        const esp_app_desc_t *appDesc = esp_app_get_description();
        const char *projectName = (appDesc != NULL && appDesc->project_name[0] != '\0') ? appDesc->project_name : "wpswitch-submersible";
        char versionCheckUrl[256];
        if (strchr(VERSION_CHECK_API_URL, '?') != NULL)
        {
            snprintf(versionCheckUrl, sizeof(versionCheckUrl), "%s&projectName=%s", VERSION_CHECK_API_URL, projectName);
        }
        else
        {
            snprintf(versionCheckUrl, sizeof(versionCheckUrl), "%s?projectName=%s", VERSION_CHECK_API_URL, projectName);
        }

        // Use device ID for authentication if available
        HttpsClientResponse *response = NULL;
        if (otaManager->deviceID != NULL && strlen(otaManager->deviceID) > 0)
        {
            ESP_LOGW(TAG, "Making authenticated OTA version check request");
            ESP_LOGW(TAG, "Version check URL: %s", versionCheckUrl);
            ESP_LOGW(TAG, "Device ID: %s", otaManager->deviceID);
            response = httpsClient->makeGetRequestTo(versionCheckUrl, otaManager->deviceID);
        }
        else
        {
            ESP_LOGW(TAG, "No device ID set for OTA authentication, request may fail");
            ESP_LOGW(TAG, "Version check URL: %s", versionCheckUrl);
            response = httpsClient->makeGetRequestTo(versionCheckUrl);
        }

        // Log stack high water mark after TLS operation (critical for diagnosing stack overflow)
        UBaseType_t stackAfter = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG, "Stack high water mark after HTTPS request: %u bytes (used ~%u bytes for TLS)", 
                 (unsigned int)stackAfter, (unsigned int)(stackBefore - stackAfter));

        // Heap integrity check after HTTPS request
        if (!heap_caps_check_integrity_all(true)) {
            ESP_LOGE(TAG, "Heap corruption detected AFTER version check request!");
            if (response != NULL) {
                freeHttpsClientResponse(response);
            }
            otaManager->abortOTA("Heap corruption after version check");
            continue;
        }

        uint32_t latestVersionNumber = 0;
        char *latestVersionName = NULL;
        if (response != NULL)
        {
            ESP_LOGW(TAG, "Version check response received - Size: %" PRIu32 " bytes, Status: %d", 
                     response->bufferSize, response->statusCode);
            ESP_LOGW(TAG, "Response received : \n\n%.*s", response->bufferSize, response->buffer);

            /**
             *  {
             *      "number": 65536,
             *      "name": "1.0.0",
             *  }
             */
            // Step 3. Parse the response from the http request based on the schema provided by server
            cJSON *root = cJSON_Parse(response->buffer);
            if (root == NULL)
            {
                ESP_LOGE(TAG, "Failed to parse version data from the reponse received !");
                freeHttpsClientResponse(response);
                response = NULL;
                otaManager->abortOTA("Failed to parse version response JSON");
                continue;
            }
            if (!cJSON_HasObjectItem(root, "number") || !cJSON_HasObjectItem(root, "name"))
            {
                ESP_LOGE(TAG, "Response does not contain version number or version name");
                freeHttpsClientResponse(response);
                response = NULL;
                cJSON_Delete(root);
                otaManager->abortOTA("Version response missing required fields");
                continue;
            }

            // copy the contents of the version name inside the json to another allocated string.
            latestVersionNumber = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "number"));
            cJSON *nameItem = cJSON_GetObjectItem(root, "name");
            const char *name = (nameItem && cJSON_IsString(nameItem)) ? nameItem->valuestring : NULL;
            if (name == NULL)
            {
                ESP_LOGE(TAG, "Version name missing or invalid in server response");
                freeHttpsClientResponse(response);
                response = NULL;
                cJSON_Delete(root);
                otaManager->abortOTA("Version name missing in server response");
                continue;
            }
            latestVersionName = (char *)malloc(strlen(name) + 1);
            strcpy(latestVersionName, name);

            // Free the memory for the response buffer and the json object
            freeHttpsClientResponse(response);
            response = NULL;
            cJSON_Delete(root);
        }
        else
        {
            // Since the response was null, so memory must be allocated for the repsonse.
            ESP_LOGE(TAG, "Failed to check recent firmware version | No Response Received ");
            otaManager->abortOTA("No response from version check server");
            continue; // Skip the rest of this iteration and try again later
        }

        freeHeapSize = xPortGetFreeHeapSize();
        minFreeHeap = esp_get_minimum_free_heap_size();
        ESP_LOGI(TAG, "Heap after version check: free=%zu bytes, min_free=%zu bytes", freeHeapSize, minFreeHeap);

        // Step 4. Use the response to check for the change in the version.
        uint32_t currentVersionNumber = otaManager->getCurrentFirmwareVersionNumber();
        char *currentVersionName = NULL;
        bool gotCurrentVer = otaManager->getCurrentFirmwareVersion(currentVersionName);
        if (!gotCurrentVer)
        {
            ESP_LOGE(TAG, "Failed to obtain current firmware version!");
            if (latestVersionName != NULL)
                free(latestVersionName);
            otaManager->abortOTA("Failed to read current firmware version");
            continue;
        }
        if (currentVersionNumber >= latestVersionNumber)
        {
            ESP_LOGI(TAG, "Firmware up to date ! Current version : (%s) = latest version : (%s)",
                     currentVersionName ? currentVersionName : "UNKNOWN",
                     latestVersionName ? latestVersionName : "UNKNOWN");
            if (latestVersionName != NULL)
                free(latestVersionName);
            if (currentVersionName != NULL)
                free(currentVersionName);
            otaManager->abortOTA("Firmware already up to date");
            continue;
        }
        else
        {
            ESP_LOGW(TAG, "Update Required ! Current version : (%s) = latest version : (%s)",
                     currentVersionName ? currentVersionName : "UNKNOWN",
                     latestVersionName ? latestVersionName : "UNKNOWN");
            // Store the new version name in the manager for event publishing
            if (otaManager->newVersionName != NULL)
            {
                free(otaManager->newVersionName);
            }
            otaManager->newVersionName = latestVersionName;
            latestVersionName = NULL; // Transfer ownership to manager
        }

        // TODO - Get the clearance from the main function.
        // Step 5. If a new version is available, make a call to the callback if it's not null
        // the callback shall return true / false whether to make a call or not.
        if (otaManager->onNewUpdateAvailableListener != NULL && otaManager->onNewUpdateAvailableListener() == false)
        {
            ESP_LOGE(TAG, "Did not receive clearance from the main function for conducting an OTA update.");
            if (currentVersionName != NULL)
                free(currentVersionName);
            otaManager->abortOTA("OTA update not authorized");
            continue;
        }

        // If this section of the code is reached, this shows that the ota update should run
        // Update the state variable accordingly.
        otaManager->isUpdateCheckRunning = false;
        otaManager->otaUpdateRunning = true;

        if (otaManager->onOTAUpdateStartedListener != NULL)
            otaManager->onOTAUpdateStartedListener(
                currentVersionName ? currentVersionName : "UNKNOWN",
                otaManager->newVersionName ? otaManager->newVersionName : "UNKNOWN");

        // Free current version name after passing to listener
        if (currentVersionName != NULL)
        {
            free(currentVersionName);
            currentVersionName = NULL;
        }

        // Begin ota update.
        esp_err_t err;
        esp_ota_handle_t updateHandle = 0;
        const esp_partition_t *updatePartition = NULL;

        // Step 6. Check if the running partition is the one configured.
        const esp_partition_t *configured = esp_ota_get_boot_partition();
        const esp_partition_t *running = esp_ota_get_running_partition();

        if (configured != running)
        {
            ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08lx, but running from offset 0x%08lx",
                     configured->address, running->address);
            ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
        }
        ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08lx)",
                 (int)running->type, (int)running->subtype, running->address);
        // free((void *)configured);

        // Configure the http client for the ota update
        esp_http_client_config_t config = {};
        
        // Build authenticated URL with deviceID and projectName for firmware download
        char *firmwareUrl = NULL;
        const esp_app_desc_t *runningAppDesc = esp_app_get_description();
        projectName = (runningAppDesc != NULL && runningAppDesc->project_name[0] != '\0') ? runningAppDesc->project_name : "wpswitch-submersible";
        size_t urlLen = strlen(FIRMWARE_OTA_UPDATE_URL);
        size_t deviceIDLen = (otaManager->deviceID != NULL) ? strlen(otaManager->deviceID) : 0;
        size_t projLen = strlen(projectName);
        size_t authUrlLen = urlLen + deviceIDLen + projLen + 64;
        
        firmwareUrl = (char *)malloc(authUrlLen);
        if (firmwareUrl != NULL)
        {
            char sep = (strchr(FIRMWARE_OTA_UPDATE_URL, '?') != NULL) ? '&' : '?';
            if (otaManager->deviceID != NULL && strlen(otaManager->deviceID) > 0)
            {
                snprintf(firmwareUrl, authUrlLen, "%s%cprojectName=%s&deviceID=%s",
                         FIRMWARE_OTA_UPDATE_URL, sep, projectName, otaManager->deviceID);
            }
            else
            {
                snprintf(firmwareUrl, authUrlLen, "%s%cprojectName=%s",
                         FIRMWARE_OTA_UPDATE_URL, sep, projectName);
            }
            ESP_LOGW(TAG, "OTA firmware download URL: %s", firmwareUrl);
            config.url = firmwareUrl;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to allocate memory for authenticated firmware URL");
            ESP_LOGW(TAG, "OTA firmware download URL (fallback): %s", FIRMWARE_OTA_UPDATE_URL);
            config.url = FIRMWARE_OTA_UPDATE_URL;
        }
        
#if USE_DEVELOPMENT_SERVER != 1
        config.cert_pem = (char *)server_pem_start;
#endif
        // Timeout and buffer configuration - increased for reliability
        config.timeout_ms = CONFIG_OTA_RECV_TIMEOUT;
        config.buffer_size = OTA_HTTP_BUFFER_SIZE;
        config.buffer_size_tx = 1024;
        
        // TCP Keep-Alive configuration - CRITICAL for long OTA downloads
        // Prevents connection drops due to idle timeouts on routers/servers
        config.keep_alive_enable = true;
        config.keep_alive_idle = OTA_KEEPALIVE_IDLE;
        config.keep_alive_interval = OTA_KEEPALIVE_INTERVAL;
        config.keep_alive_count = OTA_KEEPALIVE_COUNT;
        
        ESP_LOGI(TAG, "OTA HTTP config: timeout=%dms, buffer=%d, keepalive(idle=%ds, interval=%ds, count=%d)",
                 CONFIG_OTA_RECV_TIMEOUT, OTA_HTTP_BUFFER_SIZE, 
                 OTA_KEEPALIVE_IDLE, OTA_KEEPALIVE_INTERVAL, OTA_KEEPALIVE_COUNT);

#if USE_DEVELOPMENT_SERVER == 1
        config.skip_cert_common_name_check = true;
#else
        config.skip_cert_common_name_check = false;
#endif
        // Initialize the http client
        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client == NULL)
        {
            ESP_LOGE(TAG, "Failed to initialize http client for OTA. Aborting !!");
            if (firmwareUrl != NULL)
            {
                free(firmwareUrl);
                firmwareUrl = NULL;
            }
            otaManager->abortOTA("Failed to initialize HTTP client");
            continue;
        }
        
        ESP_LOGW(TAG, "Opening HTTP connection to: %s", config.url);
        err = esp_http_client_open(client, 0);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to open HTTP connection to %s: %s. Aborting !!", config.url, esp_err_to_name(err));
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            client = NULL;
            if (firmwareUrl != NULL)
            {
                free(firmwareUrl);
                firmwareUrl = NULL;
            }
            otaManager->abortOTA("Failed to open HTTP connection");
            continue;
        }
        ESP_LOGW(TAG, "HTTP connection opened successfully");

        // Now let's fetch the http headers.
        int contentLength = esp_http_client_fetch_headers(client);
        int statusCode = esp_http_client_get_status_code(client);
        
        ESP_LOGW(TAG, "HTTP Response - Status Code: %d, Content-Length: %d bytes", statusCode, contentLength);
        
        // Log response headers for debugging
        char *contentType = NULL;
        esp_http_client_get_header(client, "Content-Type", &contentType);
        if (contentType != NULL)
        {
            ESP_LOGW(TAG, "HTTP Response - Content-Type: %s", contentType);
        }
        
        // Check for error status codes
        if (statusCode < 200 || statusCode >= 300)
        {
            ESP_LOGE(TAG, "HTTP request failed with status code: %d", statusCode);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            client = NULL;
            if (firmwareUrl != NULL)
            {
                free(firmwareUrl);
                firmwareUrl = NULL;
            }
            otaManager->abortOTA("HTTP request returned error status code");
            continue;
        }
        
        if (contentLength <= 0)
        {
            ESP_LOGE(TAG, "Invalid content length: %d", contentLength);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            client = NULL;
            if (firmwareUrl != NULL)
            {
                free(firmwareUrl);
                firmwareUrl = NULL;
            }
            otaManager->abortOTA("Invalid or missing content length in response");
            continue;
        }

        ESP_LOGI(TAG, "Free task stack size left %" PRIu16 "", uxTaskGetStackHighWaterMark(NULL));

        // esp_task_wdt_init(15, 0);
        // Get the information of the next partition to be updated
        updatePartition = esp_ota_get_next_update_partition(running);
        // esp_task_wdt_init(5, 0);

        assert(updatePartition != NULL);
        if (updatePartition == NULL)
        {
            ESP_LOGE(TAG, "FATAL ERROR - the next partition for the update was not found. Aborting !!");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            otaManager->abortOTA("Update partition not found");
        }
        ESP_LOGI(TAG, "Update patition is not null ");
        ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%lx",
                 updatePartition->subtype, updatePartition->address);

        // Initialize variables required for the OTA update
        int firmwareFileLength = 0;
        bool imageHeaderChecked = false;

        // allocate memory for the ota data write buffer
        otaManager->otaWriteDataBuffer = (char *)malloc(OTA_WRITE_DATA_BUFFER_LENGTH + 1);

        uint8_t lastReportedProgress = 0;

        while (true)
        {
            // Defensive check: ensure client is still valid
            if (client == NULL)
            {
                ESP_LOGE(TAG, "HTTP client is NULL, exiting download loop");
                break;
            }

            // Now read data packets of the firmware
            int dataRead = esp_http_client_read(client, otaManager->otaWriteDataBuffer, OTA_WRITE_DATA_BUFFER_LENGTH);
            if (dataRead < 0)
            {
                // If no data was read in bytes - SSL/TLS read failure
                int ssl_errno = errno;
                ESP_LOGE(TAG, "Error: SSL data read error (errno: %d)", ssl_errno);
                ESP_LOGE(TAG, "Download failed at %d/%d bytes (%.1f%%)", 
                         firmwareFileLength, contentLength, 
                         contentLength > 0 ? (float)firmwareFileLength * 100.0f / contentLength : 0.0f);

                // Clean up resources for the http client - CRITICAL: close before cleanup
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                client = NULL;

                // Abort current task
                otaManager->abortOTA("SSL data read error during download");
                break;  // Exit the while loop immediately
            }
            else if (dataRead == 0)
            {
                /*
                 * As esp_http_client_read never returns negative error code, we rely on
                 * `errno` to check for underlying transport connectivity closure if any
                 */
                if (errno == ECONNRESET || errno == ENOTCONN)
                {
                    ESP_LOGE(TAG, "Connection closed, errno = %d", errno);
                    break;
                }
                if (esp_http_client_is_complete_data_received(client) == true)
                {
                    ESP_LOGI(TAG, "Connection closed");
                    break;
                }
            }
            else
            {
                // Some data has been received !
                // check if the headers have been checked !
                if (imageHeaderChecked == false)
                {
                    // The headers were not checked, this is likely to be the first
                    // data chunk received
                    esp_app_desc_t newAppDescription;
                    esp_image_header_t newImageHeader;
                    esp_image_segment_header_t newImageSegmentHeader;

                    size_t totalHeaderSize = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t);
                    if (dataRead > totalHeaderSize)
                    {
                        // This is rechecking the version of the firmware image
                        // Extract the image header, image segment header and the app description from the image
                        memcpy(
                            &newImageHeader,
                            &(otaManager->otaWriteDataBuffer[0]),
                            sizeof(esp_image_header_t));

                        memcpy(
                            &newImageSegmentHeader,
                            &(otaManager->otaWriteDataBuffer[sizeof(esp_image_header_t)]),
                            sizeof(esp_image_segment_header_t));

                        memcpy(
                            &newAppDescription,
                            &(otaManager->otaWriteDataBuffer[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)]),
                            sizeof(esp_app_desc_t));

                        printImageHeader(&newImageHeader);
                        printImageSegmentHeader(&newImageSegmentHeader);
                        printAppDescription(&newAppDescription);

                        esp_app_desc_t runningAppInfo;
                        if (esp_ota_get_partition_description(running, &runningAppInfo) == ESP_OK)
                            ESP_LOGI(TAG, "Running firmware version : %s", runningAppInfo.version);

                        // Extract the running partition's info
                        const esp_partition_t *lastInvalidApp = esp_ota_get_last_invalid_partition();
                        esp_app_desc_t invalidAppInfo;
                        if (esp_ota_get_partition_description(lastInvalidApp, &invalidAppInfo) == ESP_OK)
                            ESP_LOGI(TAG, "Last invalid firmware version: %s", invalidAppInfo.version);

                        if (lastInvalidApp != NULL)
                        {
                            // Check if the new firmware is same as that of last invalid app version
                            if (memcmp(invalidAppInfo.version, newAppDescription.version, sizeof(newAppDescription.version)) == 0)
                            {
                                ESP_LOGW(TAG, "New version is the same as invalid version.");
                                ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalidAppInfo.version);
                                ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                                esp_http_client_close(client);
                                esp_http_client_cleanup(client);
                                client = NULL;
                                otaManager->abortOTA("New version matches previously failed version");
                                break;
                            }
                        }

                        // Check if the latest version is not same as the current one
                        if (memcmp(newAppDescription.version, runningAppInfo.version, sizeof(runningAppInfo.version)) == 0)
                        {
                            ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                            esp_http_client_close(client);
                            esp_http_client_cleanup(client);
                            client = NULL;
                            otaManager->abortOTA("Downloaded version same as current version");
                            break;
                        }

                        imageHeaderChecked = true;

                        // begin the ota update
                        err = esp_ota_begin(updatePartition, OTA_WITH_SEQUENTIAL_WRITES, &updateHandle);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "esp_ota_begin failed | %s", esp_err_to_name(err));
                            esp_http_client_close(client);
                            esp_http_client_cleanup(client);
                            client = NULL;
                            otaManager->abortOTA("esp_ota_begin failed");
                            break;
                        }
                        ESP_LOGI(TAG, "esp_ota_begin succeeded");
                    }
                    else
                    {
                        // Check if the response looks like JSON error instead of firmware
                        if (dataRead > 0 && otaManager->otaWriteDataBuffer[0] == '{') {
                            // Null-terminate for safe string operations
                            char tempBuffer[256];
                            int copyLen = (dataRead < 255) ? dataRead : 255;
                            memcpy(tempBuffer, otaManager->otaWriteDataBuffer, copyLen);
                            tempBuffer[copyLen] = '\0';
                            ESP_LOGE(TAG, "Received JSON error response instead of firmware: %s", tempBuffer);
                        } else {
                            ESP_LOGE(TAG, "Received response is not likely to be firmware image (size: %d bytes)", dataRead);
                        }
                        
                        if (client != NULL)
                        {
                            esp_http_client_close(client);
                            esp_http_client_cleanup(client);
                            client = NULL;
                        }
                        if (updateHandle != 0)
                        {
                            esp_ota_abort(updateHandle);
                            updateHandle = 0;
                        }
                        otaManager->abortOTA("Invalid firmware image received");
                        break;
                    }
                }

                err = esp_ota_write(updateHandle, (const void *)otaManager->otaWriteDataBuffer, dataRead);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "FATAL- ERROR occured while writing ota data | %s ", esp_err_to_name(err));
                    if (client != NULL)
                    {
                        esp_http_client_close(client);
                        esp_http_client_cleanup(client);
                        client = NULL;
                    }
                    esp_ota_abort(updateHandle);
                    otaManager->abortOTA("Failed to write OTA data to flash");
                    break;  // Exit the while loop immediately
                }
                firmwareFileLength += dataRead;

                int progress = round(((float)firmwareFileLength / (float)contentLength) * 100.0f);
                if (otaManager->onOTAUpdateProgressListener != NULL && progress > lastReportedProgress && progress % 10 == 0)
                {
                    otaManager->onOTAUpdateProgressListener(progress);
                    lastReportedProgress = progress;
                }
            }
        }

        // Free the authenticated firmware URL if allocated
        if (firmwareUrl != NULL)
        {
            free(firmwareUrl);
            firmwareUrl = NULL;
        }

        // Check if client was cleaned up due to error (prevents NULL pointer crash)
        if (client == NULL)
        {
            ESP_LOGE(TAG, "HTTP client was cleaned up due to error, skipping post-download processing");
            continue;
        }

        // Ensure final progress is reported
        if (otaManager->onOTAUpdateProgressListener != NULL && lastReportedProgress < 100)
            otaManager->onOTAUpdateProgressListener(100);

        ESP_LOGI(TAG, "Total binary length written %d", firmwareFileLength);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        if (client != NULL && esp_http_client_is_complete_data_received(client) != true)
        {
            ESP_LOGE(TAG, "Error in receiving the complete file");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            client = NULL;
            esp_ota_abort(updateHandle);
            otaManager->abortOTA("Incomplete firmware file received");
            continue;
        }

        err = esp_ota_end(updateHandle);
        if (err != ESP_OK)
        {
            if (err == ESP_ERR_OTA_VALIDATE_FAILED)
                ESP_LOGE(TAG, "Image validation failed, image is corrupted");
            else
                ESP_LOGE(TAG, "esp_ota_end() failed | %s", esp_err_to_name(err));
            if (client != NULL)
            {
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                client = NULL;
            }
            otaManager->abortOTA(err == ESP_ERR_OTA_VALIDATE_FAILED ? "Image validation failed - corrupted" : "esp_ota_end failed");
            continue;
        }

        err = esp_ota_set_boot_partition(updatePartition);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed | %s", esp_err_to_name(err));
            if (client != NULL)
            {
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                client = NULL;
            }
            otaManager->abortOTA("Failed to set boot partition");
            continue;
        }

        // Clean up HTTP client before restart
        if (client != NULL)
        {
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            client = NULL;
        }

        ESP_LOGI(TAG, "Preparing to restart the system !");

        if (otaManager->onOTAUpdateCompletedListener != NULL)
            otaManager->onOTAUpdateCompletedListener();

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
        return;

        // Step 7. Once the esp_ota update is successful, Kill the task, and programmatically restart the esp32.
        // Add a closing brace for the for loop
    }
    otaManager->otaUpdateRunning = false;
    otaManager->isUpdateCheckRunning = false;
}

void otaTimerCallback(TimerHandle_t xTimer)
{
    if (xTimer == NULL) {
        ESP_LOGE(TAG, "OTA timer callback: Timer handle is NULL");
        return;
    }
    
    OTAManager *otaManager = (OTAManager *)pvTimerGetTimerID(xTimer);
    if (otaManager == NULL) {
        ESP_LOGE(TAG, "OTA timer callback: OTAManager pointer is NULL");
        return;
    }
    
    if (otaManager->isOTAUpdateRunning())
    {
        ESP_LOGE(TAG, "OTA Update check already running, skipping trigger.");
        return;
    }
    
    if (otaManager->otaEventGroup == NULL) {
        ESP_LOGE(TAG, "OTA timer callback: Event group is NULL");
        return;
    }
    
    xEventGroupSetBits(otaManager->otaEventGroup, START_UPDATE_CHECK_BIT);
}

void OTAManager::abortOTA(const char* errorReason)
{
    this->isUpdateCheckRunning = false;
    this->otaUpdateRunning = false;
    if (this->otaWriteDataBuffer != NULL)
    {
        free(this->otaWriteDataBuffer);
        this->otaWriteDataBuffer = NULL;
    }
    // Free stored new version name
    if (this->newVersionName != NULL)
    {
        free(this->newVersionName);
        this->newVersionName = NULL;
    }
    
    // Note: MQTT restart removed - we no longer disconnect MQTT before OTA requests
    // The configuration changes should be sufficient to prevent heap corruption.
    
    if (this->onOTAUpdateAbortedListener != NULL)
        this->onOTAUpdateAbortedListener(errorReason != NULL ? errorReason : "Unknown error");
}

void OTAManager::setOnNewUpdateAvailableListener(std::function<bool()> onNewUpdateAvailableListener)
{
    this->onNewUpdateAvailableListener = onNewUpdateAvailableListener;
}
void OTAManager::setOnOTAUpdateStartedListener(function<void(const char* currentVersion, const char* newVersion)> onOTAUpdateStartedListener)
{
    this->onOTAUpdateStartedListener = onOTAUpdateStartedListener;
}
void OTAManager::setOnOTAUpdateAbortedListener(function<void(const char* errorReason)> onOTAUpdateAbortedListener)
{
    this->onOTAUpdateAbortedListener = onOTAUpdateAbortedListener;
}
void OTAManager::setOnOTAUpdateProgressListener(function<void(uint8_t)> onOTAUpdateProgressListener)
{
    this->onOTAUpdateProgressListener = onOTAUpdateProgressListener;
}
void OTAManager::setOnOTAUpdateCompletedListener(function<void()> onOTAUpdateCompletedListener)
{
    this->onOTAUpdateCompletedListener = onOTAUpdateCompletedListener;
}

void OTAManager::setOnBeforeHttpsRequestListener(function<void()> listener)
{
    this->onBeforeHttpsRequestListener = listener;
}

void OTAManager::setOnAfterHttpsRequestListener(function<void()> listener)
{
    this->onAfterHttpsRequestListener = listener;
}

bool OTAManager::isOTAUpdateRunning()
{
    return this->otaUpdateRunning || this->isUpdateCheckRunning;
}
void OTAManager::triggerUpdateCheck()
{
    if (isOTAUpdateRunning())
    {
        ESP_LOGW(TAG, "OTA update is already in progress, skipping trigger.");
        return;
    }
    xEventGroupSetBits(otaEventGroup, START_UPDATE_CHECK_BIT);
}

void OTAManager::startPeriodicUpdatesCheck()
{
    if (this->timerHandle != NULL)
    {
        ESP_LOGW(TAG, "Timer for periodic update check is already running.");
        return;
    }

    ESP_LOGI(TAG, "Creating timer for periodic update check..");
    this->timerHandle = xTimerCreate(
        "Version Check Timer",
        (this->updateCheckIntervalMillis / portTICK_PERIOD_MS),
        pdTRUE,       // Auto-reload timer
        (void *)this, // Pass the pointer to the OTAManager instance.
        otaTimerCallback);

    if (this->timerHandle == NULL)
    {
        ESP_LOGE(TAG, "Failed to create timer for ota update check");
        return;
    }

    xTimerStart(this->timerHandle, portMAX_DELAY);
    ESP_LOGI(TAG, "Timer for ota update check started successfully");
}
void OTAManager::stopPeriodicUpdatesCheck()
{
    if (this->timerHandle != NULL)
    {
        xTimerStop(this->timerHandle, portMAX_DELAY);
        xTimerDelete(this->timerHandle, portMAX_DELAY);
        this->timerHandle = NULL;
        ESP_LOGI(TAG, "Timer for ota update check stopped and deleted.");
    }
    else
    {
        ESP_LOGW(TAG, "Timer for ota update check was not running.");
    }
}
uint32_t OTAManager::getVersionNumberFromString(char *versionStringPtr)
{
    if (versionStringPtr == NULL)
        return 0;
    char versionString[10];
    memcpy(versionString, versionStringPtr, strlen(versionStringPtr) + 1);
    uint32_t versionNumber = 0;
    char *token = strtok(versionString, ".");
    versionNumber |= ((uint32_t)atoi(token) << 16);

    for (int i = 1; i >= 0; i--)
    {
        char *part = strtok(NULL, ".");
        uint32_t partInt = (uint32_t)atoi(part);
        versionNumber |= (partInt << (8 * i));
    }
    return versionNumber;
}
bool OTAManager::getCurrentFirmwareVersion(char *&versionStringHolder)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t runningPartitionDescription;
    esp_err_t ret = esp_ota_get_partition_description(running, &runningPartitionDescription);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read partition info | %s", esp_err_to_name(ret));
        versionStringHolder = NULL;
        return false;
    }
    size_t len = strlen(runningPartitionDescription.version) + 1;
    versionStringHolder = (char *)malloc(sizeof(char) * len);
    memcpy(versionStringHolder, runningPartitionDescription.version, len);
    return true;
}
bool OTAManager::getCurrentAppDescription(cJSON *&jsonHolder)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t runningPartitionDescription;
    esp_err_t ret = esp_ota_get_partition_description(running, &runningPartitionDescription);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read partition info | %s", esp_err_to_name(ret));
        jsonHolder = NULL;
        return false;
    }

    // Prepare the json string
    jsonHolder = cJSON_CreateObject();
    char *appElfString = get_sha256_string(runningPartitionDescription.app_elf_sha256);

    cJSON_AddStringToObject(jsonHolder, "appVersion", runningPartitionDescription.version);
    cJSON_AddStringToObject(jsonHolder, "idfVersion", runningPartitionDescription.idf_ver);
    cJSON_AddStringToObject(jsonHolder, "date", runningPartitionDescription.date);
    cJSON_AddStringToObject(jsonHolder, "time", runningPartitionDescription.time);
    cJSON_AddStringToObject(jsonHolder, "projectName", runningPartitionDescription.project_name);
    cJSON_AddStringToObject(jsonHolder, "appElf", appElfString);
    free(appElfString);

    return true;
}
uint32_t OTAManager::getCurrentFirmwareVersionNumber()
{
    char *versionString = NULL;
    this->getCurrentFirmwareVersion(versionString);

    if (versionString == NULL)
        return 0;

    uint32_t versionNumber = 0;

    bool match = regex_match(versionString, regex(VERSION_NAME_REGEX));
    versionNumber = (match == 1) ? getVersionNumberFromString(versionString) : 0;

    if (versionString != NULL)
        free(versionString);

    return versionNumber;
}

OTAManager::OTAManager(HttpsClient *httpsClient) : OTAManager(httpsClient, DEFAULT_UPDATE_CHECK_INTERVAL_MILLIS)
{
    // Delegated to the other constructor
}

OTAManager::OTAManager(HttpsClient *httpsClient, uint64_t updateCheckIntervalMillis)
{
    this->httpsClient = httpsClient;
    this->updateCheckIntervalMillis = updateCheckIntervalMillis;
    this->timerHandle = NULL;
    this->otaTaskHandle = NULL;

    // If the value passed is less than the minimum possible value
    // set the value to the minimum
    if (this->updateCheckIntervalMillis < MINIMUM_UPDATE_CHECK_INTERVAL_MILLIS)
        this->updateCheckIntervalMillis = MINIMUM_UPDATE_CHECK_INTERVAL_MILLIS;

    // If the value passed is more than the maximum possible value
    // set the value to the maximum
    if (this->updateCheckIntervalMillis > MAXIMUM_UPDATE_CHECK_INTERVAL_MILLIS)
        this->updateCheckIntervalMillis = MAXIMUM_UPDATE_CHECK_INTERVAL_MILLIS;

    this->otaEventGroup = xEventGroupCreate();
    if (this->otaEventGroup == NULL)
    {
        ESP_LOGE(TAG, "Failed to create OTA event group");
    }

    xTaskCreatePinnedToCore(
        otaTask,
        "ota_task",
        OTA_UPDATE_CHECK_TASK_STACK_DEPTH,
        (void *)this,
        OTA_UPDATE_CHECK_TASK_PRIORITY,
        &this->otaTaskHandle,
        PRO_CPU_NUM);
}

OTAManager::~OTAManager()
{
    if (this->timerHandle != NULL)
    {
        xTimerDelete(this->timerHandle, portMAX_DELAY);
    }
    if (this->otaTaskHandle != NULL)
    {
        vTaskDelete(this->otaTaskHandle);
    }
    if (this->otaEventGroup != NULL)
    {
        vEventGroupDelete(this->otaEventGroup);
    }
    if (this->otaWriteDataBuffer != NULL)
    {
        free(this->otaWriteDataBuffer);
        this->otaWriteDataBuffer = NULL;
    }
    if (this->deviceID != NULL)
    {
        free(this->deviceID);
        this->deviceID = NULL;
    }
    if (this->newVersionName != NULL)
    {
        free(this->newVersionName);
        this->newVersionName = NULL;
    }
}

void OTAManager::setDeviceID(const char *deviceID)
{
    // Free existing deviceID if any
    if (this->deviceID != NULL)
    {
        free(this->deviceID);
        this->deviceID = NULL;
    }
    
    if (deviceID != NULL && strlen(deviceID) > 0)
    {
        size_t len = strlen(deviceID) + 1;
        this->deviceID = (char *)malloc(len);
        if (this->deviceID != NULL)
        {
            memcpy(this->deviceID, deviceID, len);
            ESP_LOGI(TAG, "Device ID set for OTA authentication: %s", this->deviceID);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to allocate memory for device ID");
        }
    }
}