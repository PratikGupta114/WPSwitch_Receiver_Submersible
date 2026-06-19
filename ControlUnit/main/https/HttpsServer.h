#ifndef HTTPSSERVER_H_
#define HTTPSSERVER_H_

#define USE_HTTP_SECURE 0

#include "esp_https_server.h"
#include <functional>
#include <string>

using namespace std;

typedef enum httpsMessage
{
    HTTPS_MSG_WIFI_CONNECT_INIT = 0,
    HTTPS_MSG_WIFI_CONNECT_SUCCESS,
    HTTPS_MSG_WIFI_CONNECT_FAIL,
    HTTPS_MSG_OTA_UPDATE_SUCCESSFUL,
    HTTPS_MSG_OTA_UPDATE_FAILED,
    HTTPS_MSG_OTA_UPDATE_INITIALIZED,
} HttpsServerMessages;

typedef struct httpsQueueMessage
{
    HttpsServerMessages messageID;
} HttpsServerMessage;

class HttpsServer
{
private:
    httpd_handle_t httpsServerHandle = NULL;
    friend esp_err_t invalidUriHandler(httpd_req_t *req);
    friend esp_err_t credentialsExchangeRequestHandler(httpd_req_t *req);
    friend esp_err_t credentialsPostHandler(httpd_req_t *req);
    friend esp_err_t credentialsGetHandler(httpd_req_t *req);
    friend esp_err_t credentialDeleteHandler(httpd_req_t *req);
    friend esp_err_t wiFiApScanGetHandler(httpd_req_t *req);
    friend esp_err_t wifiApDisconnectHandler(httpd_req_t *req);
    friend esp_err_t exitConfigModeHandler(httpd_req_t *req);
    friend esp_err_t systemConfigGetHandler(httpd_req_t *req);
    friend esp_err_t systemConfigPostHandler(httpd_req_t *req);
    friend esp_err_t pumpConfigGetHandler(httpd_req_t *req);
    friend esp_err_t pumpConfigPostHandler(httpd_req_t *req);
    friend esp_err_t rfConfigGetHandler(httpd_req_t *req);
    friend esp_err_t rfConfigPostHandler(httpd_req_t *req);

    // https://github.com/swarajsatvaya/UrlDecoder-C/blob/master/urldecode.h
    friend char *urlDecode(const char *str);

public:
    bool sendMessageToHttpsServerMonitor(HttpsServerMessages messageID);
    httpd_handle_t configureHttpsServer(void);
    void start(void);
    void stop(void);
    bool isRunning();

    function<void(string, string, string)> onCredentialsSharedListener;
    function<bool(string, string, bool, char *&)> credentialsPostCallback;
    function<bool(char *&)> credentialsGetCallback;
    function<bool(string)> credentialDeleteCallback;
    function<bool(char *&)> wiFiApScanCallback;
    function<bool(char *&)> wiFiStatusCallback;
    function<bool(char *&)> deviceInfoCallback;
    function<bool()> wifiApDisconnectCallback;
    function<void()> exitConfigModeCallback;
    function<void()> httpEventCallback;

    // New configuration callbacks
    function<bool(char *&)> systemConfigGetCallback;
    function<bool(const char *, char *&)> systemConfigPostCallback;
    function<bool(char *&)> pumpConfigGetCallback;
    function<bool(const char *, char *&)> pumpConfigPostCallback;
    function<bool(char *&)> rfConfigGetCallback;
    function<bool(const char *, char *&)> rfConfigPostCallback;

    void setOnCredentialsSharedListener(function<void(string, string, string)> onCredentialsSharedListener);
    void setCredentialsPostCallback(function<bool(string, string, bool, char *&)> credentialsPostCallback);
    void setCredentialsGetCallback(function<bool(char *&)> credentialsGetCallback);
    void setCredentialDeleteCallback(function<bool(string)> credentialDeleteCallback);
    void setWiFiAPScanCallback(function<bool(char *&)> wiFiApScanCallback);
    void setWiFiStatusCallback(function<bool(char *&)> wiFiStatusCallback);
    void setDeviceInfoCallback(function<bool(char *&)> deviceInfoCallback);
    void setWifiApDisconnectCallback(function<bool()> wifiApDisconnectCallback);
    void setExitConfigModeCallback(function<void()> exitConfigModeCallback);
    void setHttpEventCallback(function<void()> httpEventCallback);

    // New configuration callback setters
    void setSystemConfigGetCallback(function<bool(char *&)> systemConfigGetCallback);
    void setSystemConfigPostCallback(function<bool(const char *, char *&)> systemConfigPostCallback);
    void setPumpConfigGetCallback(function<bool(char *&)> pumpConfigGetCallback);
    void setPumpConfigPostCallback(function<bool(const char *, char *&)> pumpConfigPostCallback);
    void setRfConfigGetCallback(function<bool(char *&)> rfConfigGetCallback);
    void setRfConfigPostCallback(function<bool(const char *, char *&)> rfConfigPostCallback);

    HttpsServer()
    {
    }
};

#endif