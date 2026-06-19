#ifndef SECRETS_H_
#define SECRETS_H_

#include "sdkconfig.h"

// Development/Production Server Switch
// Set to 1 for development servers, 0 for production servers
#define USE_DEVELOPMENT_SERVER 0

// MQTT Credentials
// #define MQTT_USER_ID "prototypeEditionDevice"
// #define MQTT_PASSWORD "*fluctoPrototype1254#"

// MQTT Broker Credentials (Production)
#define MQTT_BROKER_USERNAME_SECRET "WPSwitchFirebaseBackendAdmin"
#define MQTT_BROKER_PASSWORD_SECRET "*MakerPratik1148#"

// MQTT Broker Credentials (Development)
#define MQTT_BROKER_USERNAME_SECRET_DEV "dummyUser"
#define MQTT_BROKER_PASSWORD_SECRET_DEV "#000000#"

// WiFi Access Point Mode Credentials (for creating hotspot)
#define WIFI_AP_SSID_SECRET "WPSwitch_9fa8e5"
#define WIFI_AP_PASSWORD_SECRET "6$11DMvAjV"

// Compile-time check to prevent insecure OTA configuration in production
#ifndef CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP
#if USE_DEVELOPMENT_SERVER == 1
#error "Insecure OTA Configuration: OTA over HTTP should be enabled in development builds. Please enable OTA over HTTP in menuconfig."
#endif
#endif

#ifdef CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP
#if USE_DEVELOPMENT_SERVER == 0
#error "Insecure OTA Configuration: OTA over HTTP must be disabled in production builds. Please disable CONFIG_OTA_ALLOW_HTTP in menuconfig."
#endif
#endif

#endif /* SECRETS_H_ */
