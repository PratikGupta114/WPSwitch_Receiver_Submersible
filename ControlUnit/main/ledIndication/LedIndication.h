#ifndef MAIN_LEDINDICATION_H_
#define MAIN_LEDINDICATION_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
// #include "led_strip.h" // Main include for led_strip component - REMOVED
#include "ws2812/WS2812.h" // Include for your custom WS2812 driver

#include "../peripherals/PeripheralManager.h" // For NEOPIXEL_DATA_OUTPUT_PIN and NUM_LEDS

// Define for RMT resolution (e.g., 10MHz) - REMOVED, WS2812 handles RMT config
// #define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

// Using NUM_LEDS from PeripheralManager or define locally if preferred
#define NUM_LEDS 4

// Global LED brightness control (0-100%)
#define LED_BRIGHTNESS_PERCENT 10
#define LED_TASK_STACK_SIZE 1792  // +256 for OTA safety (HWM was 736 bytes)
#define LED_TASK_PRIORITY 5

// Structure to represent RGB color (compatible with pixel_t from WS2812.h)
struct RgbColor
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

// Namespace for predefined colors
namespace LedColors
{
    const RgbColor BLACK = {0, 0, 0};
    const RgbColor RED = {255, 0, 0};
    const RgbColor GREEN = {0, 255, 0};
    const RgbColor BLUE = {0, 0, 255};
    const RgbColor YELLOW = {255, 255, 0};
    const RgbColor ORANGE = {255, 165, 0};
    const RgbColor CYAN = {0, 255, 255};
    const RgbColor MAGENTA = {255, 0, 255};
    const RgbColor PURPLE = {128, 0, 128};
    const RgbColor WHITE = {255, 255, 255};
    const RgbColor PALE_BLUE = {173, 216, 230};   // Light Blue
    const RgbColor PALE_YELLOW = {255, 255, 224}; // Light Yellow

    // Purple-bluish colors for config mode
    const RgbColor CONFIG_PURPLE = {128, 0, 128};     // Standard purple (0x800080)
    const RgbColor CONFIG_INDIGO = {75, 0, 130};      // Deep indigo (0x4B0082)
    const RgbColor CONFIG_VIOLET = {138, 43, 226};    // Blue-violet (0x8A2BE2)
    const RgbColor CONFIG_DEEP_PURPLE = {75, 0, 130}; // Deep purple for solid states

    // Light greenish-Cyan colors for HTTPS server states
    const RgbColor LIGHT_CYAN = {0, 255, 255};          // Pure cyan (0x00FFFF)
    const RgbColor AQUAMARINE = {127, 255, 212};        // Light greenish-cyan (0x7FFFD4)
    const RgbColor TURQUOISE = {64, 224, 208};          // Turquoise (0x40E0D0)
    const RgbColor MEDIUM_AQUAMARINE = {102, 205, 170}; // Medium aquamarine (0x66CDAA)
}

// Enum for LED indexing (optional, but good for clarity)
enum class LedIndex : uint8_t
{
    LED0 = 0, // Example: Data Reception Status
    LED1 = 1, // Example: Pump Manual Restart
    LED2 = 2, // Example: MQTT Status
    LED3 = 3, // Example: WiFi/HTTP Status
};

// Enum for LED3 Application Mode
enum class Led3AppMode : uint8_t
{
    WIFI_STATION_MODE = 0,
    WIFI_AP_HTTP_SERVER_MODE = 1
};

// Enum for WiFi Station Application Status
enum class WifiStaAppStatus : uint8_t
{
    WIFI_STA_INITIALIZING = 0, // NEW: Status before first connection attempt
    WIFI_STA_DISCONNECTED = 1,
    WIFI_STA_CONNECTING = 2,
    WIFI_STA_CONNECTED = 3,
    DEVICE_STATE_NO_WIFI_CREDS = 4
};

// Enum for HTTP Server Application Status
enum class HttpServerAppStatus : uint8_t
{
    HTTP_SERVER_INACTIVE = 0,
    HTTP_SERVER_INITIATING = 1,
    HTTP_AP_ACTIVE_READY_FOR_CONN = 2,
    HTTP_AP_ACTIVE_NO_CLIENT = 3,                 // AP mode active, no client connected
    HTTP_AP_ACTIVE_CLIENT_CONNECTED = 4,          // AP mode active, client connected
    HTTPS_SERVER_REMOTE_STATION_CONNECTING = 5,   // Remote station connecting to HTTPS server
    HTTPS_SERVER_REMOTE_STATION_CONNECTED = 6,    // Remote station connected to HTTPS server
    HTTPS_SERVER_REMOTE_STATION_DISCONNECTING = 7 // Remote station disconnecting from HTTPS server
};

// Enum for MQTT Application Status
enum class MqttAppStatus : uint8_t
{
    MQTT_INITIALIZING = 0,      // NEW: Status before first connection attempt
    MQTT_WIFI_DISCONNECTED = 1, // Higher level: WiFi not connected, so MQTT cannot operate
    MQTT_DISCONNECTED = 2,      // WiFi connected, but MQTT broker is not
    MQTT_CONNECTING = 3,        // Attempting to connect to MQTT broker
    MQTT_CONNECTED = 4          // Connected to MQTT broker
};

// Enum for Pump Application Status
enum class PumpAppStatus : uint8_t
{
    PUMP_AUTO_MODE_OK = 0,
    PUMP_MANUAL_RESTART_NEEDED = 1,
    PUMP_RECALIBRATION_MODE = 2
};

// Enum for Data Reception Application Status
enum class DataReceptionAppStatus : uint8_t
{
    DATA_RECEPTION_OK = 0,
    DATA_RECEPTION_TIMEOUT = 1
    // Add other data reception statuses if needed
};

class LedIndication
{
public:
    LedIndication();
    ~LedIndication();
    esp_err_t init();
    void startLedTask();

    // Methods to update status from other parts of the application
    void setLed3OverallMode(Led3AppMode mode);
    void updateWifiStaStatus(WifiStaAppStatus status);
    void updateHttpServerStatus(HttpServerAppStatus status);
    void updateMqttStatus(MqttAppStatus status);
    void updatePumpStatus(PumpAppStatus status);
    void updateDataReceptionStatus(DataReceptionAppStatus status);
    void triggerHttpEventIndication();
    void clearIndications();

private:
    static const char *TAG; // For logging

    TaskHandle_t led_task_handle;
    SemaphoreHandle_t data_mutex; // Mutex for protecting shared data
    // led_strip_handle_t led_strip_handle; // Handle for the led_strip object - REMOVED
    WS2812 *ws2812_strip; // Pointer to your custom WS2812 object

    // Current states for LED indications
    Led3AppMode current_led3_app_mode;
    WifiStaAppStatus current_wifi_sta_status;
    HttpServerAppStatus current_http_server_status;
    MqttAppStatus current_mqtt_status;
    PumpAppStatus current_pump_status;
    DataReceptionAppStatus current_data_reception_status;
    uint32_t blink_counter;         // Counter for blink patterns
    uint8_t http_event_blink_count; // Counter for the HTTP event blink sequence

    esp_err_t initialize_led_strip();                                // Will be adapted to initialize WS2812
    void set_pixel_color_internal(LedIndex led_idx, RgbColor color); // Will use WS2812 methods
    void refresh_led_strip();                                        // Will use WS2812->show()

    static void led_task_entry(void *arg);
    void led_task_loop();
};

#endif /* MAIN_LEDINDICATION_H_ */