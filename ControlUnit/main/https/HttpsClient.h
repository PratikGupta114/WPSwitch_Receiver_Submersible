
#ifndef HTTPSCLIENT_H_
#define HTTPSCLIENT_H_

#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/event_groups.h"
#include <functional>

#define RECV_BUFFER_LENGTH 2048

// default server side certificates
// extern const char server_pem_start[] asm("_binary_server_certificate_pem_start");
// extern const char server_pem_end[] asm("_binary_server_certificate_pem_end");

// firebase function server side certificates
extern const char server_pem_start[] asm("_binary_firebase_function_pem_start");
extern const char server_pem_end[] asm("_binary_firebase_function_pem_end");

typedef enum http_common_status_codes
{
    STATUS_BAD_REQUEST = 400,
    STATUS_UNAUTHORIZED = 401,
    STATUS_FORBIDDEN = 403,
    STATUS_NOT_FOUND = 404,
    STATUS_INTERNAL_SERVER_ERROR = 500,
    STATUS_OK = 200,
    STATUS_CODE_UNUSUAL = -1
} HttpCommonStatusCode;

typedef struct https_response_t
{
    char *buffer;
    size_t bufferSize;
    HttpCommonStatusCode statusCode;
} HttpsClientResponse;

/**
 * @brief Free an HttpsClientResponse and its buffer
 * 
 * This function properly frees the response buffer before freeing the response
 * structure itself. Always use this function to free HttpsClientResponse objects
 * returned by makeGetRequestTo() to prevent memory leaks.
 * 
 * @param response Pointer to the HttpsClientResponse to free (can be NULL)
 */
void freeHttpsClientResponse(HttpsClientResponse *response);

class HttpsClient
{
private:
    bool httpClientClean = true;
    bool httpClientClosed = true;
    bool validResponse = false;
    size_t recvDataLen = 0;  // Track actual bytes received in recvBuffer

    EventGroupHandle_t httpRequestEventGroup;
    esp_http_client_handle_t httpClientHandle;
    esp_http_client_config_t httpClientConfig;
    char *recvBuffer = NULL;
    friend esp_err_t httpClientEventHandler(esp_http_client_event_t *event);
    HttpCommonStatusCode getStatusCodeForNumber(int code);

    void cleanHttpClient();
    void closeHttpConnection();

public:
    HttpsClient();
    ~HttpsClient();
    HttpsClientResponse *makeGetRequestTo(const char *url);
    // Overload with device ID for authenticated OTA requests
    HttpsClientResponse *makeGetRequestTo(const char *url, const char *deviceID);
};

#endif