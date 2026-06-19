
#include "HttpsClient.h"
#include "secrets.h"
#include <string.h>

#define TAG "HttpsClient"

#define MIN(x, y) ((x < y) ? x : y)

void freeHttpsClientResponse(HttpsClientResponse *response)
{
    if (response == NULL)
    {
        return;
    }
    
    // Free the buffer first, then the response structure
    if (response->buffer != NULL)
    {
        free(response->buffer);
        response->buffer = NULL;
    }
    
    free(response);
}

size_t parseSizeTFromString(char *st)
{
    char *x;
    for (x = st; *x; x++)
    {
        if (!isdigit(*x))
            return 0;
    }
    return (strtoul(st, 0, 10));
}

esp_err_t httpClientEventHandler(esp_http_client_event_t *event)
{
    HttpsClient *httpsClient = (HttpsClient *)event->user_data;

    switch (event->event_id)
    {
    case HTTP_EVENT_ERROR:
    {
        // DEBUG: Log more details about the HTTP error
        if (event->client != NULL)
        {
            int statusCode = esp_http_client_get_status_code(event->client);
            ESP_LOGE(TAG, "%s() -> Error Occurred | HTTP status_code=%d", __func__, statusCode);
        }
        else
        {
            ESP_LOGE(TAG, "%s() -> Error Occurred | client handle is NULL", __func__);
        }
    }
    break;
    case HTTP_EVENT_ON_CONNECTED:
    {
        // Since the connection with the server is established, mark the client as connected.
        httpsClient->httpClientClosed = false;
        ESP_LOGI(TAG, "%s() -> Connected to the server", __func__);
    }
    break;
    case HTTP_EVENT_HEADER_SENT:
    {
        ESP_LOGI(TAG, "%s() -> Http Event Header sent ", __func__);
    }
    break;
    case HTTP_EVENT_ON_HEADER:
    {
        if (httpsClient->validResponse != true)
            return ESP_FAIL;

        if (strcmp(event->header_key, "Content-Length") == 0)
        {
            size_t contentLength = parseSizeTFromString(event->header_value);
            // ESP_LOGW(TAG, "%s() -> Content Length : %d", __func__, contentLength);
            if (contentLength >= RECV_BUFFER_LENGTH)
            {
                // Mark this response as an invalid one and clean the http client handle
                ESP_LOGW(TAG, "%s() -> Cannot process response greater than length %d", __func__, RECV_BUFFER_LENGTH);
                httpsClient->validResponse = false;
                return ESP_FAIL;
            }
        }
        else if (strcmp(event->header_key, "Content-Type") == 0 && strcmp(event->header_value, "application/json; charset=utf-8") != 0)
        {
            // Mark this response as an invalid one and clean the http client handle
            ESP_LOGE(TAG, "%s() -> response Content-Type : \"%s\". Currently response types of \"application/json; charset=utf-8\" are only accepted", __func__, event->header_value);
            httpsClient->validResponse = false;
            return ESP_FAIL;
        }
        else
        {
            // Mark the response a valid one
            httpsClient->validResponse = true;
        }
    }
    break;
    case HTTP_EVENT_ON_DATA:
    {
        if (httpsClient->validResponse == false)
        {
            ESP_LOGE(TAG, "%s() -> HTTP_EVENT_ON_DATA: not a valid response", __func__);
            return ESP_FAIL;
        }
        
        // Safety check: ensure recvBuffer is allocated
        if (httpsClient->recvBuffer == NULL)
        {
            ESP_LOGE(TAG, "%s() -> HTTP_EVENT_ON_DATA: recvBuffer is NULL", __func__);
            httpsClient->validResponse = false;
            return ESP_FAIL;
        }
        
        if (!esp_http_client_is_chunked_response(event->client))
        {
            // Non-chunked response - APPEND data to buffer (may arrive in multiple segments)
            ESP_LOGD(TAG, "Non-chunked data segment: %d bytes (total so far: %zu)", 
                     event->data_len, httpsClient->recvDataLen);
            
            // Calculate remaining space in buffer (leave room for null terminator)
            size_t remainingSpace = (RECV_BUFFER_LENGTH - 1) - httpsClient->recvDataLen;
            
            if (remainingSpace == 0)
            {
                ESP_LOGW(TAG, "%s() -> Buffer full, cannot append %d more bytes", 
                         __func__, event->data_len);
                // Don't mark as invalid - we have partial data that may be usable
                return ESP_OK;
            }
            
            // Calculate how much we can copy
            size_t copyLen = (event->data_len <= remainingSpace) ? event->data_len : remainingSpace;
            
            if (copyLen < (size_t)event->data_len)
            {
                ESP_LOGW(TAG, "%s() -> Truncating: only %zu of %d bytes fit", 
                         __func__, copyLen, event->data_len);
            }
            
            // APPEND to buffer at current position (not overwrite from start)
            memcpy(httpsClient->recvBuffer + httpsClient->recvDataLen, event->data, copyLen);
            httpsClient->recvDataLen += copyLen;
            httpsClient->recvBuffer[httpsClient->recvDataLen] = '\0';  // Always null-terminate
            
            ESP_LOGD(TAG, "Buffer now contains %zu bytes", httpsClient->recvDataLen);
        }
        else
        {
            // Chunked response - not supported
            int chunkLength = 0, contentLength = 0;
            esp_http_client_get_chunk_length(httpsClient->httpClientHandle, &chunkLength);
            contentLength = esp_http_client_get_content_length(httpsClient->httpClientHandle);

            ESP_LOGE(TAG, "Chunked response not supported: chunk=%d, total=%d", chunkLength, contentLength);
            httpsClient->validResponse = false;
        }
    }
    break;
    case HTTP_EVENT_ON_FINISH:
    {
        ESP_LOGI(TAG, "Http finished ");
    }
    break;
    case HTTP_EVENT_DISCONNECTED:
    {
        // Mark the connection to be disconnected.
        httpsClient->httpClientClosed = false;
        ESP_LOGI(TAG, "Http Disconnected ");
    }
    break;
    case HTTP_EVENT_REDIRECT:
    {
        ESP_LOGI(TAG, "Http redirect event");
    }
    break;
    default:
        break;
    }

    return ESP_OK;
}

HttpsClientResponse *HttpsClient::makeGetRequestTo(const char *url)
{
    // DEBUG: Log the URL being requested
    ESP_LOGI(TAG, "%s() -> Request URL: %s", __func__, url ? url : "NULL");

    // CRITICAL: Reset config to clean state for each request to prevent stale pointers
    memset(&this->httpClientConfig, 0, sizeof(esp_http_client_config_t));
    
    // Set all required fields fresh for this request
    this->httpClientConfig.url = url;
    this->httpClientConfig.event_handler = (http_event_handle_cb)httpClientEventHandler;
    this->httpClientConfig.user_data = this;
    this->httpClientConfig.buffer_size = RECV_BUFFER_LENGTH;
    this->httpClientConfig.timeout_ms = 30000;  // 30 second timeout
    
#if USE_DEVELOPMENT_SERVER == 1
    this->httpClientConfig.transport_type = HTTP_TRANSPORT_OVER_TCP;
#else
    this->httpClientConfig.cert_pem = server_pem_start;
    this->httpClientConfig.cert_len = server_pem_end - server_pem_start;
#endif

    HttpsClientResponse *response = NULL;

    (this->httpClientHandle) = esp_http_client_init(&(this->httpClientConfig));

    if (this->httpClientHandle == NULL)
    {
        // Mark both the flags that denote the client handle isn't initialized and the
        // connection was not established.
        this->httpClientClean = true;
        this->httpClientClosed = true;
        ESP_LOGE(TAG, "%s() -> failed to initialize http client", __func__);
    }
    else
    {
        // Mark that the client was initialized but connection was not established
        this->httpClientClean = false;
        this->httpClientClosed = true;
        // Let's initially mark the response as a valid on. This will be later updated
        this->validResponse = true;
        this->recvDataLen = 0;

        // allocate memory for the buffer and zero it
        recvBuffer = (char *)malloc(RECV_BUFFER_LENGTH * sizeof(char));
        if (recvBuffer == NULL)
        {
            ESP_LOGE(TAG, "%s() -> Failed to allocate recvBuffer", __func__);
            this->cleanHttpClient();
            return NULL;
        }
        memset(this->recvBuffer, 0x00, RECV_BUFFER_LENGTH);

        // Now let's attempt to connect to the server and perform the http request
        esp_err_t result = esp_http_client_perform(this->httpClientHandle);

        // DEBUG: Always log the HTTP status code and content length after perform
        int statusCode = esp_http_client_get_status_code(this->httpClientHandle);
        int64_t contentLength = esp_http_client_get_content_length(this->httpClientHandle);
        ESP_LOGI(TAG, "%s() -> HTTP Response: status_code=%d, content_length=%lld", __func__, statusCode, contentLength);

        if (result != ESP_OK)
        {
            // Log the error statement, the client shall be cleaned up in the next step
            ESP_LOGE(TAG, "%s() -> failed to perform the http request | %s (status_code=%d)", __func__, esp_err_to_name(result), statusCode);
            
            // DEBUG: Log any partial response body received for debugging auth/server errors
            if (this->recvBuffer != NULL && contentLength > 0)
            {
                // Null-terminate for safe logging (limit to reasonable size)
                size_t logLen = (contentLength < 512) ? (size_t)contentLength : 512;
                char *debugBuffer = (char *)malloc(logLen + 1);
                if (debugBuffer != NULL)
                {
                    memcpy(debugBuffer, this->recvBuffer, logLen);
                    debugBuffer[logLen] = '\0';
                    ESP_LOGW(TAG, "%s() -> Response body (partial): %s", __func__, debugBuffer);
                    free(debugBuffer);
                }
            }
        }
        else if (this->validResponse == true)
        {
            // Use actual received data length, not contentLength from header
            size_t actualLen = this->recvDataLen;
            if (actualLen == 0)
            {
                // Fallback to contentLength if recvDataLen wasn't set
                actualLen = (contentLength > 0 && contentLength < RECV_BUFFER_LENGTH) ? (size_t)contentLength : 0;
            }
            
            if (actualLen == 0)
            {
                ESP_LOGW(TAG, "%s() -> No data received", __func__);
            }
            else
            {
                response = (HttpsClientResponse *)malloc(sizeof(HttpsClientResponse));
                if (response != NULL)
                {
                    response->buffer = (char *)malloc(sizeof(char) * (actualLen + 1));
                    if (response->buffer != NULL)
                    {
                        memcpy(response->buffer, this->recvBuffer, actualLen);
                        response->buffer[actualLen] = '\0';  // Ensure null-termination
                        response->statusCode = getStatusCodeForNumber(statusCode);
                        response->bufferSize = actualLen;
                        
                        ESP_LOGI(TAG, "%s() -> Request successful, response size=%zu bytes", __func__, actualLen);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "%s() -> Failed to allocate response buffer", __func__);
                        free(response);
                        response = NULL;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "%s() -> Failed to allocate response struct", __func__);
                }
            }
        }
        else
        {
            // DEBUG: Log when validResponse is false (e.g., content-type mismatch, chunked response)
            ESP_LOGW(TAG, "%s() -> Response marked invalid (validResponse=false), status_code=%d", __func__, statusCode);
        }
    }
    this->cleanHttpClient();
    return response;
}

HttpCommonStatusCode HttpsClient::getStatusCodeForNumber(int code)
{
    switch (code)
    {
    case 200:
        return STATUS_OK;
    case 400:
        return STATUS_BAD_REQUEST;
    case 401:
        return STATUS_UNAUTHORIZED;
    case 403:
        return STATUS_FORBIDDEN;
    case 404:
        return STATUS_NOT_FOUND;
    case 500:
        return STATUS_INTERNAL_SERVER_ERROR;
    default:
        return STATUS_CODE_UNUSUAL;
    }
}

void HttpsClient::cleanHttpClient()
{
    if (this->httpClientHandle != NULL && this->httpClientClean == false)
    {
        ESP_LOGI(TAG, "%s() -> closing connection ", __func__);
        esp_http_client_cleanup(this->httpClientHandle);
        this->httpClientClean = true;
        this->httpClientClosed = true;
        this->httpClientHandle = NULL;

        if (this->recvBuffer != NULL)
            free(this->recvBuffer);
        this->recvBuffer = NULL;
    }
}
void HttpsClient::closeHttpConnection()
{
    if (this->httpClientHandle != NULL && this->httpClientClosed == false)
    {
        ESP_LOGI(TAG, "%s() -> closing connection ", __func__);
        esp_http_client_close(this->httpClientHandle);
        this->httpClientClosed = true;
    }
}

HttpsClient::HttpsClient()
{
    this->httpClientConfig = {};
    (this->httpClientConfig).event_handler = (http_event_handle_cb)httpClientEventHandler;
    (this->httpClientConfig).user_data = this;
    (this->httpClientConfig).buffer_size = RECV_BUFFER_LENGTH;
}

HttpsClient::~HttpsClient()
{
    // Clean up HTTP client first (this also frees recvBuffer if httpClientClean is false)
    if (httpClientHandle != NULL)
    {
        cleanHttpClient();
    }
    
    // Safety: free recvBuffer if it wasn't freed by cleanHttpClient
    // (e.g., if httpClientClean was already true)
    if (recvBuffer != NULL)
    {
        free(recvBuffer);
        recvBuffer = NULL;
    }
}

HttpsClientResponse *HttpsClient::makeGetRequestTo(const char *url, const char *deviceID)
{
    if (url == NULL)
    {
        ESP_LOGE(TAG, "%s() -> URL is NULL", __func__);
        return NULL;
    }

    // Build URL with deviceID query parameter for authentication
    char *authenticatedUrl = NULL;
    if (deviceID != NULL && strlen(deviceID) > 0)
    {
        // Calculate required buffer size: url + "?deviceID=" + deviceID + null terminator
        size_t urlLen = strlen(url);
        size_t deviceIDLen = strlen(deviceID);
        size_t authUrlLen = urlLen + 10 + deviceIDLen + 1; // 10 = strlen("?deviceID=")
        
        authenticatedUrl = (char *)malloc(authUrlLen);
        if (authenticatedUrl == NULL)
        {
            ESP_LOGE(TAG, "%s() -> Failed to allocate memory for authenticated URL", __func__);
            return NULL;
        }
        
        // Check if URL already has query parameters
        if (strchr(url, '?') != NULL)
        {
            // URL already has query params, append with &
            snprintf(authenticatedUrl, authUrlLen, "%s&deviceID=%s", url, deviceID);
        }
        else
        {
            // No existing query params, start with ?
            snprintf(authenticatedUrl, authUrlLen, "%s?deviceID=%s", url, deviceID);
        }
        
        ESP_LOGI(TAG, "%s() -> Authenticated URL: %s", __func__, authenticatedUrl);
    }
    else
    {
        // No deviceID provided, use original URL
        authenticatedUrl = strdup(url);
        if (authenticatedUrl == NULL)
        {
            ESP_LOGE(TAG, "%s() -> Failed to duplicate URL", __func__);
            return NULL;
        }
    }

    // Make the request using the authenticated URL
    HttpsClientResponse *response = makeGetRequestTo(authenticatedUrl);
    
    // Free the allocated URL
    free(authenticatedUrl);
    
    return response;
}