

// #include "esp_tls.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portable.h"
#include "HttpsServer.h"
#include <algorithm>
#include "cJSON.h"
#include "storage/nvsManager.h"
#include "esp_timer.h"

#define TAG "HttpsServer"

#define RATE_LIMIT_INTERVAL_MS 1000 // 1 request per second
static int64_t last_request_timestamp_ms = 0;

#define HEALTH_CHECK_RATE_LIMIT_INTERVAL_MS 500 // 2 requests per second for health checks
static int64_t last_health_check_timestamp_ms = 0;

#define CONTENT_TYPE_HEADER_KEY_STRING "Content-Type"
#define CONTENT_TYPE_HEADER_VALUE_STRING "application/json"

#define MIN(x, y) ((x < y) ? x : y)

#define HTTPS_SERVER_TASK_STACK_SIZE 6144
#define HTTPS_SERVER_TASK_PRIORITY 10
#define HTTPS_SERVER_TASK_CORE_ID PRO_CPU_NUM

#define MAX_CONTENT_BUFFER_SIZE 512  // Increased to accommodate RF config requests with encryption keys

using namespace std;

// https://github.com/swarajsatvaya/UrlDecoder-C/blob/master/urlDecoder.c
char *urlDecode(const char *str)
{
    // int d = 0; /* whether or not the string is decoded */

    char *dStr = (char *)malloc(strlen(str) + 1);
    char eStr[] = "00"; /* for a hex code */

    strcpy(dStr, str);

    // while(!d) {
    // d = 1;
    int i; /* the counter for the string */

    for (i = 0; i < strlen(dStr); ++i)
    {

        if (dStr[i] == '%')
        {
            if (dStr[i + 1] == 0)
                return dStr;

            if (isxdigit(dStr[i + 1]) && isxdigit(dStr[i + 2]))
            {

                // d = 0;

                /* combine the next to numbers into one */
                eStr[0] = dStr[i + 1];
                eStr[1] = dStr[i + 2];

                /* convert it to decimal */
                long int x = strtol(eStr, NULL, 16);

                /* remove the hex */
                memmove(&dStr[i + 1], &dStr[i + 3], strlen(&dStr[i + 3]) + 1);

                dStr[i] = x;
            }
        }
        else if (dStr[i] == '+')
        {
            dStr[i] = ' ';
        }
    }
    //}
    return dStr;
}

esp_err_t invalidUriHandler(httpd_req_t *req)
{
    const char resp[] = "{ \"message\" : \"Invalid Request\"}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t check_rate_limit(httpd_req_t *req)
{
    int64_t current_timestamp_ms = esp_timer_get_time() / 1000;
    if (current_timestamp_ms - last_request_timestamp_ms < RATE_LIMIT_INTERVAL_MS)
    {
        httpd_resp_set_status(req, "429 Too Many Requests");
        httpd_resp_send(req, "Too Many Requests", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
    last_request_timestamp_ms = current_timestamp_ms;
    return ESP_OK;
}

// Lightweight health check endpoint - returns 200 OK with no body
static esp_err_t healthCheckHandler(httpd_req_t *req)
{
    int64_t current_timestamp_ms = esp_timer_get_time() / 1000;
    if (current_timestamp_ms - last_health_check_timestamp_ms < HEALTH_CHECK_RATE_LIMIT_INTERVAL_MS)
    {
        httpd_resp_set_status(req, "429 Too Many Requests");
        httpd_resp_send(req, NULL, 0);
        return ESP_FAIL;
    }
    last_health_check_timestamp_ms = current_timestamp_ms;
    
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

esp_err_t credentialsExchangeRequestHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    // Now let's read the request

    HttpsServer httpsServer = *((HttpsServer *)req->user_ctx);

    char content[120];

    /* Truncate if content length larger than the buffer */
    size_t recv_size = std::min(req->content_len, sizeof(content));
    int ret = httpd_req_recv(req, content, recv_size);

    if (ret <= 0)
    {
        /* 0 return value indicates connection closed */
        /* Check if timeout occurred */

        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }

        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }
    // Let's add a null terminator at the end of the string.
    content[recv_size] = '\0';

    cJSON *root = cJSON_Parse(content);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to parse credentials exchange JSON");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"message\":\"Invalid JSON\"}");
        return ESP_FAIL;
    }

    // Validate required fields exist before accessing
    cJSON *ssidItem = cJSON_GetObjectItem(root, "ssid");
    cJSON *passwordItem = cJSON_GetObjectItem(root, "password");
    cJSON *tokenItem = cJSON_GetObjectItem(root, "token");
    
    if (ssidItem == NULL || passwordItem == NULL || tokenItem == NULL ||
        !cJSON_IsString(ssidItem) || !cJSON_IsString(passwordItem) || !cJSON_IsString(tokenItem))
    {
        ESP_LOGE(TAG, "Credentials exchange missing required fields");
        cJSON_Delete(root);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"message\":\"Missing required fields\"}");
        return ESP_FAIL;
    }

    std::string ssid = ssidItem->valuestring;
    std::string password = passwordItem->valuestring;
    std::string token = tokenItem->valuestring;

    // Clean up cJSON object before sending response (Requirement 6.1)
    cJSON_Delete(root);

    const char resp[] = "{\"message\":\"Credentials shared successfully\"}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    // ESP_LOGI(TAG, "SSID : %s | Password : %s | Token : %s ", ssid, password, token);
    if (httpsServer.onCredentialsSharedListener != NULL)
        httpsServer.onCredentialsSharedListener(ssid, password, token);

    return ESP_OK;
}

esp_err_t credentialsPostHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;
    ESP_LOGI(TAG, "HTTP: POST /wifi-credential - ContentLength=%d", req->content_len);

    char contentBuffer[MAX_CONTENT_BUFFER_SIZE];
    size_t receivngSize = MIN(MAX_CONTENT_BUFFER_SIZE - 1, req->content_len);  // Leave room for null terminator
    if (receivngSize < req->content_len)
    {
        ESP_LOGE(TAG, "Request content length size more than the buffer size");
    }
    int ret = httpd_req_recv(req, contentBuffer, receivngSize);

    if (ret <= 0)
    {
        /* 0 return value indicates connection closed */
        /* Check if timeout occurred */

        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }

        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }

    // Let's add a null terminator at the end of the response.
    contentBuffer[receivngSize] = '\0';

    cJSON *root = cJSON_Parse(contentBuffer);

    if (root == NULL)
    {
        // Error while parsing the json object.
        // hence send an error response
        ESP_LOGE(TAG, "Invalid Request Body; Does not have a json body");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"message\":\"Invalid Request Body\"}");
        return ESP_FAIL;
    }

    if (!cJSON_HasObjectItem(root, "ssid") || !cJSON_HasObjectItem(root, "password") || !cJSON_HasObjectItem(root, "connect"))
    {
        // Error while parsing the json object.
        // hence send an error response
        ESP_LOGE(TAG, "Invalid Request Body; Request body does not contain ssid or password or connect bool flag");
        cJSON_Delete(root);  // Clean up cJSON object (Requirement 6.1)
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"message\":\"Invalid Request Body\"}");
        return ESP_FAIL;
    }

    string ssid = cJSON_GetObjectItem(root, "ssid")->valuestring;
    string password = cJSON_GetObjectItem(root, "password")->valuestring;
    string connectString = cJSON_GetObjectItem(root, "connect")->valuestring;

    bool connect = connectString.compare("true") == 0 ? true : false;

    // Clean up cJSON object after extracting values (Requirement 6.1)
    cJSON_Delete(root);

    ESP_LOGI(TAG, "HTTP: WiFi credentials - SSID=%s, Connect=%s", 
             ssid.c_str(), (connect ? "true" : "false"));

    bool result = false;
    char *jsonResultStringHolder = NULL;
    // ESP_LOGI(TAG, "SSID : %s | Password : %s | Token : %s ", ssid, password, token);
    if (httpsServer->credentialsPostCallback == NULL)
    {
        ESP_LOGE(TAG, "credentialsPostCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }
    else
        result = httpsServer->credentialsPostCallback(ssid, password, connect, jsonResultStringHolder);

    if (result == false || jsonResultStringHolder == NULL)
    {
        ESP_LOGE(TAG, "credentialsPostCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }
    else
    {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, jsonResultStringHolder, HTTPD_RESP_USE_STRLEN);
        cJSON_free(jsonResultStringHolder);
        if (httpsServer->httpEventCallback != NULL)
        {
            httpsServer->httpEventCallback();
        }
    }
    return ESP_OK;
}

esp_err_t wiFiApScanGetHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;

    char *jsonResultStringHolder = NULL;

    ESP_LOGI(TAG, "HTTP: GET /nearby-ap-scan-list - Initiating WiFi scan");

    if (httpsServer->wiFiApScanCallback == NULL)
    {
        ESP_LOGE(TAG, "wiFiApScanCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    bool scanSuccess = httpsServer->wiFiApScanCallback(jsonResultStringHolder);

    if (!scanSuccess || jsonResultStringHolder == NULL)
    {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Failed to scan Nearby AP list\"}");
        if (jsonResultStringHolder != NULL)
        {
            cJSON_free(jsonResultStringHolder);
        }
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, jsonResultStringHolder, HTTPD_RESP_USE_STRLEN);
    cJSON_free(jsonResultStringHolder);

    if (httpsServer->httpEventCallback != NULL)
    {
        httpsServer->httpEventCallback();
    }

    return ESP_OK;
}

esp_err_t credentialsGetHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;

    char *jsonResultStringHolder = NULL;

    ESP_LOGI(TAG, "HTTP: GET /wifi-credentials - Retrieving saved credentials");

    if (httpsServer->credentialsGetCallback == NULL)
    {
        ESP_LOGE(TAG, "credentialsGetCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    bool success = httpsServer->credentialsGetCallback(jsonResultStringHolder);

    if (!success || jsonResultStringHolder == NULL)
    {
        const char resp[] = "{\"success\" : \"false\", \"message\":\"Failed to get WiFi Credentials\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        if (jsonResultStringHolder != NULL)
        {
            cJSON_free(jsonResultStringHolder);
        }
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, jsonResultStringHolder, HTTPD_RESP_USE_STRLEN);
    cJSON_free(jsonResultStringHolder);

    if (httpsServer->httpEventCallback != NULL)
    {
        httpsServer->httpEventCallback();
    }

    return ESP_OK;
}

esp_err_t credentialDeleteHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;

    // Reading request query params shall be implemented in the future

    // ESP_LOGW(TAG, "Received GET request for saved wifi-credentials");

    // now let's collect the query string from the request received
    size_t queryStringLength = httpd_req_get_url_query_len(req);

    // Check if the request contains valid query
    if (queryStringLength <= 0)
    {
        ESP_LOGE(TAG, "Request does not contain a valid query string !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"message\":\"valid query missing from the request\"}");
        return ESP_FAIL;
    }

    char *queryString = (char *)malloc((queryStringLength + 1) * sizeof(char));
    // queryString[queryStringLength] = '\0';

    esp_err_t result = httpd_req_get_url_query_str(req, queryString, queryStringLength + 1);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Error while getting request query string | %s", esp_err_to_name(result));
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        free(queryString);
        return ESP_FAIL;
    }

    char ssidBuffer[34];
    if (httpd_query_key_value(queryString, "ssid", ssidBuffer, 34) != ESP_OK)
    {
        ESP_LOGE(TAG, "Invalid SSID !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Invalid SSID\"}");
        free(queryString);
        return ESP_FAIL;
    }

    char *decodedSsid = urlDecode(ssidBuffer);
    string ssid = decodedSsid;

    if (httpsServer->credentialDeleteCallback == NULL)
    {
        ESP_LOGE(TAG, "credentialDeleteCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        free(queryString);
        free(decodedSsid);
        return ESP_FAIL;
    }

    bool success = httpsServer->credentialDeleteCallback(ssid);

    if (!success)
    {
        const char resp[] = "{ \"success\" : \"false\", \"message\":\"Failed to delete credential\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        const char resp[] = "{ \"success\" : \"true\", \"message\":\"Credential deleted successfully\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        if (httpsServer->httpEventCallback != NULL)
        {
            httpsServer->httpEventCallback();
        }
    }
    free(queryString);
    free(decodedSsid);
    return ESP_OK;
}

esp_err_t deviceInfoGetHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;

    char *jsonResultStringHolder = NULL;

    ESP_LOGW(TAG, "Received GET request for device info");

    if (httpsServer->deviceInfoCallback == NULL)
    {
        ESP_LOGE(TAG, "deviceInfoCallback not set!");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    bool success = httpsServer->deviceInfoCallback(jsonResultStringHolder);

    if (!success || jsonResultStringHolder == NULL)
    {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Failed to obtain device info\"}");
        if (jsonResultStringHolder != NULL)
        {
            cJSON_free(jsonResultStringHolder);
        }
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, jsonResultStringHolder, HTTPD_RESP_USE_STRLEN);
    cJSON_free(jsonResultStringHolder);

    if (httpsServer->httpEventCallback != NULL)
    {
        httpsServer->httpEventCallback();
    }

    return ESP_OK;
}

esp_err_t wifiStatusGetHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;

    char *jsonResultStringHolder = NULL;

    ESP_LOGW(TAG, "Received GET request for getting wifi connection status");

    if (httpsServer->wiFiStatusCallback == NULL)
    {
        ESP_LOGE(TAG, "wiFiStatusCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    bool success = httpsServer->wiFiStatusCallback(jsonResultStringHolder);

    if (!success || jsonResultStringHolder == NULL)
    {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Failed to obtain wifi status\"}");
        if (jsonResultStringHolder != NULL)
        {
            cJSON_free(jsonResultStringHolder);
        }
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, jsonResultStringHolder, HTTPD_RESP_USE_STRLEN);
    cJSON_free(jsonResultStringHolder);

    if (httpsServer->httpEventCallback != NULL)
    {
        httpsServer->httpEventCallback();
    }

    return ESP_OK;
}

esp_err_t wifiApDisconnectHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;

    if (httpsServer->wifiApDisconnectCallback == NULL)
    {
        ESP_LOGE(TAG, "wifiApDisconnectCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    bool success = httpsServer->wifiApDisconnectCallback();

    if (!success)
    {
        const char resp[] = "{ \"success\" : \"false\", \"message\":\"Failed to disconnect Access Point\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        const char resp[] = "{ \"success\" : \"true\", \"message\":\"Current connected Access Point disconnected\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        if (httpsServer->httpEventCallback != NULL)
        {
            httpsServer->httpEventCallback();
        }
    }

    return ESP_OK;
}

esp_err_t exitConfigModeHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;
    ESP_LOGI(TAG, "HTTP: POST /exitConfigMode - Exiting configuration mode, device will restart");

    if (httpsServer->exitConfigModeCallback == NULL)
    {
        ESP_LOGE(TAG, "exitConfigModeCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    const char resp[] = "{\"message\":\"Device will now restart\"}";
    httpd_resp_set_hdr(req, CONTENT_TYPE_HEADER_KEY_STRING, CONTENT_TYPE_HEADER_VALUE_STRING);
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    httpsServer->exitConfigModeCallback();
    if (httpsServer->httpEventCallback != NULL)
    {
        httpsServer->httpEventCallback();
    }
    return ESP_OK;
}

esp_err_t systemConfigGetHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;

    if (httpsServer->systemConfigGetCallback == NULL)
    {
        ESP_LOGE(TAG, "systemConfigGetCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    char *jsonResultStringHolder = NULL;
    bool success = httpsServer->systemConfigGetCallback(jsonResultStringHolder);

    if (!success || jsonResultStringHolder == NULL)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Failed to get system configuration\"}");
        if (jsonResultStringHolder != NULL)
        {
            free(jsonResultStringHolder);
        }
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, jsonResultStringHolder, HTTPD_RESP_USE_STRLEN);
    free(jsonResultStringHolder);

    if (httpsServer->httpEventCallback != NULL)
    {
        httpsServer->httpEventCallback();
    }

    return ESP_OK;
}

esp_err_t systemConfigPostHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;
    ESP_LOGI(TAG, "HTTP: POST /system-config - Updating system configuration");
    char content[MAX_CONTENT_BUFFER_SIZE];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);  // Leave room for null terminator
    int ret = httpd_req_recv(req, content, recv_size);

    if (ret <= 0)
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[recv_size] = '\0';

    if (httpsServer->systemConfigPostCallback == NULL)
    {
        ESP_LOGE(TAG, "systemConfigPostCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    char *jsonResultStringHolder = NULL;
    bool success = httpsServer->systemConfigPostCallback(content, jsonResultStringHolder);

    if (!success || jsonResultStringHolder == NULL)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Failed to update system configuration\"}");
        if (jsonResultStringHolder != NULL)
        {
            free(jsonResultStringHolder);
        }
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, jsonResultStringHolder, HTTPD_RESP_USE_STRLEN);
    free(jsonResultStringHolder);

    if (httpsServer->httpEventCallback != NULL)
    {
        httpsServer->httpEventCallback();
    }

    return ESP_OK;
}

esp_err_t pumpConfigGetHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;

    if (httpsServer->pumpConfigGetCallback == NULL)
    {
        ESP_LOGE(TAG, "pumpConfigGetCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    char *jsonResultStringHolder = NULL;
    bool success = httpsServer->pumpConfigGetCallback(jsonResultStringHolder);

    if (!success || jsonResultStringHolder == NULL)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Failed to get pump configuration\"}");
        if (jsonResultStringHolder != NULL)
        {
            free(jsonResultStringHolder);
        }
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, jsonResultStringHolder, HTTPD_RESP_USE_STRLEN);
    free(jsonResultStringHolder);

    if (httpsServer->httpEventCallback != NULL)
    {
        httpsServer->httpEventCallback();
    }

    return ESP_OK;
}

esp_err_t pumpConfigPostHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;
    ESP_LOGI(TAG, "HTTP: POST /pump-config - Updating pump configuration");
    char content[MAX_CONTENT_BUFFER_SIZE];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);  // Leave room for null terminator
    int ret = httpd_req_recv(req, content, recv_size);

    if (ret <= 0)
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[recv_size] = '\0';

    if (httpsServer->pumpConfigPostCallback == NULL)
    {
        ESP_LOGE(TAG, "pumpConfigPostCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    char *jsonResultStringHolder = NULL;
    bool success = httpsServer->pumpConfigPostCallback(content, jsonResultStringHolder);

    if (!success || jsonResultStringHolder == NULL)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Failed to update pump configuration\"}");
        if (jsonResultStringHolder != NULL)
        {
            free(jsonResultStringHolder);
        }
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, jsonResultStringHolder, HTTPD_RESP_USE_STRLEN);
    free(jsonResultStringHolder);

    if (httpsServer->httpEventCallback != NULL)
    {
        httpsServer->httpEventCallback();
    }

    return ESP_OK;
}

esp_err_t rfConfigGetHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;

    if (httpsServer->rfConfigGetCallback == NULL)
    {
        ESP_LOGE(TAG, "rfConfigGetCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    char *jsonResultStringHolder = NULL;
    bool success = httpsServer->rfConfigGetCallback(jsonResultStringHolder);

    if (!success || jsonResultStringHolder == NULL)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Failed to get RF configuration\"}");
        if (jsonResultStringHolder != NULL)
        {
            free(jsonResultStringHolder);
        }
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, jsonResultStringHolder, HTTPD_RESP_USE_STRLEN);
    free(jsonResultStringHolder);

    if (httpsServer->httpEventCallback != NULL)
    {
        httpsServer->httpEventCallback();
    }

    return ESP_OK;
}

esp_err_t rfConfigPostHandler(httpd_req_t *req)
{
    if (check_rate_limit(req) != ESP_OK)
    {
        return ESP_FAIL;
    }
    HttpsServer *httpsServer = (HttpsServer *)req->user_ctx;
    ESP_LOGI(TAG, "HTTP: POST /rf_config - Updating RF module configuration");
    char content[MAX_CONTENT_BUFFER_SIZE];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);  // Leave room for null terminator
    int ret = httpd_req_recv(req, content, recv_size);

    if (ret <= 0)
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[recv_size] = '\0';

    if (httpsServer->rfConfigPostCallback == NULL)
    {
        ESP_LOGE(TAG, "rfConfigPostCallback not set !");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Unexpected error occurred\"}");
        return ESP_FAIL;
    }

    char *jsonResultStringHolder = NULL;
    bool success = httpsServer->rfConfigPostCallback(content, jsonResultStringHolder);

    if (!success || jsonResultStringHolder == NULL)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"message\":\"Failed to update RF configuration\"}");
        if (jsonResultStringHolder != NULL)
        {
            free(jsonResultStringHolder);
        }
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, jsonResultStringHolder, HTTPD_RESP_USE_STRLEN);
    free(jsonResultStringHolder);

    if (httpsServer->httpEventCallback != NULL)
    {
        httpsServer->httpEventCallback();
    }

    return ESP_OK;
}

httpd_handle_t HttpsServer::configureHttpsServer(void)
{
#if USE_HTTP_SECURE
    httpd_ssl_config_t httpsConfiguration = HTTPD_SSL_CONFIG_DEFAULT();

    extern const unsigned char server_cert_start[] asm("_binary_https_server_certificate_pem_start");
    extern const unsigned char server_cert_end[] asm("_binary_https_server_certificate_pem_end");
    extern const unsigned char private_key_pem_start[] asm("_binary_https_server_key_pem_start");
    extern const unsigned char private_key_pem_end[] asm("_binary_https_server_key_pem_end");

    httpsConfiguration.servercert = server_cert_start;
    httpsConfiguration.servercert_len = server_cert_end - server_cert_start;
    httpsConfiguration.prvtkey_pem = private_key_pem_start;
    httpsConfiguration.prvtkey_len = private_key_pem_end - private_key_pem_start;
    httpsConfiguration.httpd.core_id = HTTPS_SERVER_TASK_CORE_ID;
    httpsConfiguration.httpd.task_priority = HTTPS_SERVER_TASK_PRIORITY;
    httpsConfiguration.httpd.stack_size = HTTPS_SERVER_TASK_STACK_SIZE;
    httpsConfiguration.httpd.max_uri_handlers = 20;
    httpsConfiguration.httpd.max_open_sockets = 3;  // Allow 3 concurrent connections for config mode
    httpsConfiguration.httpd.recv_wait_timeout = 20;
    httpsConfiguration.httpd.send_wait_timeout = 20;
    httpsConfiguration.httpd.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port : '%d' with priority : '%d'",
             httpsConfiguration.httpd.server_port,
             httpsConfiguration.httpd.task_priority);

    ESP_LOGI(TAG, "Server certificate length: %d", httpsConfiguration.servercert_len);
    ESP_LOGI(TAG, "Server key length: %d", httpsConfiguration.prvtkey_len);

    if (httpd_ssl_start(&(this->httpsServerHandle), &httpsConfiguration) == ESP_OK)
#else
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = HTTPS_SERVER_TASK_CORE_ID;
    config.task_priority = HTTPS_SERVER_TASK_PRIORITY;
    config.stack_size = HTTPS_SERVER_TASK_STACK_SIZE;
    config.max_uri_handlers = 20;
    config.max_open_sockets = 3;  // Allow 3 concurrent connections for config mode
    config.recv_wait_timeout = 20;
    config.send_wait_timeout = 20;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port : '%d' with priority : '%d'",
             config.server_port,
             config.task_priority);

    if (httpd_start(&(this->httpsServerHandle), &config) == ESP_OK)
#endif
    {
        ESP_LOGI(TAG, "Registering invalid URI Handlers");
        httpd_uri_t invalidGetUri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = invalidUriHandler,
            .user_ctx = NULL};

        httpd_uri_t invalidPostUri = {
            .uri = "/",
            .method = HTTP_POST,
            .handler = invalidUriHandler,
            .user_ctx = NULL};

        httpd_uri_t healthCheckGetUri = {
            .uri = "/health",
            .method = HTTP_GET,
            .handler = healthCheckHandler,
            .user_ctx = NULL};

        httpd_uri_t credentialsPostUri = {
            .uri = "/wifi-credential",
            .method = HTTP_POST,
            .handler = credentialsPostHandler,
            .user_ctx = this};

        httpd_uri_t credentialDeleteUri = {
            .uri = "/wifi-credential",
            .method = HTTP_DELETE,
            .handler = credentialDeleteHandler,
            .user_ctx = this};

        httpd_uri_t credentialGetUri = {
            .uri = "/wifi-credentials",
            .method = HTTP_GET,
            .handler = credentialsGetHandler,
            .user_ctx = this};

        httpd_uri_t wiFiApScanGetUri = {
            .uri = "/nearby-ap-scan-list",
            .method = HTTP_GET,
            .handler = wiFiApScanGetHandler,
            .user_ctx = this,
        };

        httpd_uri_t wifiStatusGetUri = {
            .uri = "/wifi-status",
            .method = HTTP_GET,
            .handler = wifiStatusGetHandler,
            .user_ctx = this,
        };

        httpd_uri_t deviceInfoGetUri = {
            .uri = "/device-info",
            .method = HTTP_GET,
            .handler = deviceInfoGetHandler,
            .user_ctx = this,
        };

        httpd_uri_t wifiApDisconnectPostUri = {
            .uri = "/disconnect-ap",
            .method = HTTP_POST,
            .handler = wifiApDisconnectHandler,
            .user_ctx = this,
        };

        httpd_uri_t exitConfigModePostUri = {
            .uri = "/exitConfigMode",
            .method = HTTP_POST,
            .handler = exitConfigModeHandler,
            .user_ctx = this,
        };

        httpd_uri_t systemConfigGetUri = {
            .uri = "/system-config",
            .method = HTTP_GET,
            .handler = systemConfigGetHandler,
            .user_ctx = this};

        httpd_uri_t systemConfigPostUri = {
            .uri = "/system-config",
            .method = HTTP_POST,
            .handler = systemConfigPostHandler,
            .user_ctx = this};

        httpd_uri_t pumpConfigGetUri = {
            .uri = "/pump-config",
            .method = HTTP_GET,
            .handler = pumpConfigGetHandler,
            .user_ctx = this};

        httpd_uri_t pumpConfigPostUri = {
            .uri = "/pump-config",
            .method = HTTP_POST,
            .handler = pumpConfigPostHandler,
            .user_ctx = this};

        httpd_uri_t rfConfigGetUri = {
            .uri = "/rf_config",
            .method = HTTP_GET,
            .handler = rfConfigGetHandler,
            .user_ctx = this};

        httpd_uri_t rfConfigPostUri = {
            .uri = "/rf_config",
            .method = HTTP_POST,
            .handler = rfConfigPostHandler,
            .user_ctx = this};

        httpd_register_uri_handler(this->httpsServerHandle, &invalidGetUri);
        httpd_register_uri_handler(this->httpsServerHandle, &invalidPostUri);
        httpd_register_uri_handler(this->httpsServerHandle, &healthCheckGetUri);
        httpd_register_uri_handler(this->httpsServerHandle, &credentialsPostUri);
        httpd_register_uri_handler(this->httpsServerHandle, &credentialDeleteUri);
        httpd_register_uri_handler(this->httpsServerHandle, &credentialGetUri);
        httpd_register_uri_handler(this->httpsServerHandle, &wiFiApScanGetUri);
        httpd_register_uri_handler(this->httpsServerHandle, &wifiStatusGetUri);
        httpd_register_uri_handler(this->httpsServerHandle, &deviceInfoGetUri);
        httpd_register_uri_handler(this->httpsServerHandle, &wifiApDisconnectPostUri);
        httpd_register_uri_handler(this->httpsServerHandle, &exitConfigModePostUri);
        httpd_register_uri_handler(this->httpsServerHandle, &systemConfigGetUri);
        httpd_register_uri_handler(this->httpsServerHandle, &systemConfigPostUri);
        httpd_register_uri_handler(this->httpsServerHandle, &pumpConfigGetUri);
        httpd_register_uri_handler(this->httpsServerHandle, &pumpConfigPostUri);
        httpd_register_uri_handler(this->httpsServerHandle, &rfConfigGetUri);
        httpd_register_uri_handler(this->httpsServerHandle, &rfConfigPostUri);
    }

    uint32_t freeHeapSize = xPortGetFreeHeapSize();
    ESP_LOGI(TAG, "free heap size after starting https server : %" PRIu32, freeHeapSize);

    return this->httpsServerHandle;
}

void HttpsServer::start(void)
{
    if (this->httpsServerHandle == NULL)
    {
        this->httpsServerHandle = this->configureHttpsServer();
    }
}

void HttpsServer::stop(void)
{
    if (this->httpsServerHandle)
    {
#if USE_HTTP_SECURE
        httpd_ssl_stop(this->httpsServerHandle);
#else
        httpd_stop(this->httpsServerHandle);
#endif
        ESP_LOGD(TAG, "Stopping http server ");
        httpsServerHandle = NULL;

        uint32_t freeHeapSize = xPortGetFreeHeapSize();
        ESP_LOGI(TAG, "free heap size after stopping http server : %" PRIu32, freeHeapSize);
    }
}

bool HttpsServer::isRunning()
{
    return httpsServerHandle != NULL;
}
void HttpsServer::setOnCredentialsSharedListener(
    std::function<void(std::string, std::string, std::string)> onCredentialsSharedListener)
{
    this->onCredentialsSharedListener = onCredentialsSharedListener;
}

void HttpsServer::setCredentialsPostCallback(function<bool(string, string, bool, char *&)> credentialsPostCallback)
{
    this->credentialsPostCallback = credentialsPostCallback;
}
void HttpsServer::setCredentialsGetCallback(function<bool(char *&)> credentialsGetCallback)
{
    this->credentialsGetCallback = credentialsGetCallback;
}
void HttpsServer::setCredentialDeleteCallback(function<bool(string)> credentialDeleteCallback)
{
    this->credentialDeleteCallback = credentialDeleteCallback;
}
void HttpsServer::setWiFiAPScanCallback(function<bool(char *&)> wiFiApScanCallback)
{
    this->wiFiApScanCallback = wiFiApScanCallback;
}
void HttpsServer::setWiFiStatusCallback(function<bool(char *&)> wiFiStatusCallback)
{
    this->wiFiStatusCallback = wiFiStatusCallback;
}
void HttpsServer::setDeviceInfoCallback(function<bool(char *&)> deviceInfoCallback)
{
    this->deviceInfoCallback = deviceInfoCallback;
}
void HttpsServer::setWifiApDisconnectCallback(function<bool()> wifiApDisconnectCallback)
{
    this->wifiApDisconnectCallback = wifiApDisconnectCallback;
}

void HttpsServer::setExitConfigModeCallback(function<void()> exitConfigModeCallback)
{
    this->exitConfigModeCallback = exitConfigModeCallback;
}

void HttpsServer::setHttpEventCallback(function<void()> httpEventCallback)
{
    this->httpEventCallback = httpEventCallback;
}

void HttpsServer::setSystemConfigGetCallback(function<bool(char *&)> systemConfigGetCallback)
{
    this->systemConfigGetCallback = systemConfigGetCallback;
}

void HttpsServer::setSystemConfigPostCallback(function<bool(const char *, char *&)> systemConfigPostCallback)
{
    this->systemConfigPostCallback = systemConfigPostCallback;
}

void HttpsServer::setPumpConfigGetCallback(function<bool(char *&)> pumpConfigGetCallback)
{
    this->pumpConfigGetCallback = pumpConfigGetCallback;
}

void HttpsServer::setPumpConfigPostCallback(function<bool(const char *, char *&)> pumpConfigPostCallback)
{
    this->pumpConfigPostCallback = pumpConfigPostCallback;
}

void HttpsServer::setRfConfigGetCallback(function<bool(char *&)> rfConfigGetCallback)
{
    this->rfConfigGetCallback = rfConfigGetCallback;
}

void HttpsServer::setRfConfigPostCallback(function<bool(const char *, char *&)> rfConfigPostCallback)
{
    this->rfConfigPostCallback = rfConfigPostCallback;
}