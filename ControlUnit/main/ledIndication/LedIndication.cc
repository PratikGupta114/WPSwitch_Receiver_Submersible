#include "LedIndication.h"
#include "peripherals/PeripheralManager.h" // For NEOPIXEL_DATA_OUTPUT_PIN and NUM_LEDS
// #include "led_strip.h" // No longer needed
// #include "led_strip_spi.h" // No longer needed
// #include "driver/spi_master.h" // No longer needed

// Initialize static const char* TAG
const char *LedIndication::TAG = "LedIndication";

LedIndication::LedIndication() : led_task_handle(NULL),
                                 ws2812_strip(nullptr), // Initialize WS2812 pointer to nullptr
                                 current_led3_app_mode(Led3AppMode::WIFI_STATION_MODE),
                                 current_wifi_sta_status(WifiStaAppStatus::WIFI_STA_INITIALIZING),
                                 current_http_server_status(HttpServerAppStatus::HTTP_SERVER_INACTIVE),
                                 current_mqtt_status(MqttAppStatus::MQTT_INITIALIZING),
                                 current_pump_status(PumpAppStatus::PUMP_AUTO_MODE_OK),
                                 current_data_reception_status(DataReceptionAppStatus::DATA_RECEPTION_OK),
                                 blink_counter(0),
                                 http_event_blink_count(0)
{
    // Comment out constructor logs to reduce UART traffic
    // ESP_LOGI(TAG, "LedIndication constructor START.");
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create data_mutex in LedIndication constructor!");
    }
    // ESP_LOGI(TAG, "LedIndication constructor END.");
}

LedIndication::~LedIndication()
{
    if (led_task_handle != NULL)
    {
        vTaskDelete(led_task_handle);
    }
    if (data_mutex != NULL)
    {
        vSemaphoreDelete(data_mutex);
    }
    if (ws2812_strip != nullptr)
    {
        ESP_LOGI(TAG, "Clearing and deleting WS2812 strip object.");
        ws2812_strip->clear(); // Clear LEDs before deleting
        ws2812_strip->show();  // Ensure clear is sent to strip
        delete ws2812_strip;
        ws2812_strip = nullptr;
    }
}

esp_err_t LedIndication::init()
{
    // Comment out init logs to reduce UART traffic
    // ESP_LOGI(TAG, "LedIndication init() START.");
    if (data_mutex == NULL)
    {
        data_mutex = xSemaphoreCreateMutex();
        if (data_mutex == NULL)
        {
            ESP_LOGE(TAG, "Failed to create data_mutex in LedIndication init()!");
            return ESP_FAIL;
        }
    }

    esp_err_t err = initialize_led_strip();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize LED strip: %s", esp_err_to_name(err));
        return err;
    }

    // Clear the strip after initialization
    if (ws2812_strip != nullptr)
    {
        ws2812_strip->clear();
        ws2812_strip->show();
        // Comment out frequent LED logs to reduce UART traffic
        // ESP_LOGI(TAG, "WS2812 strip cleared after initialization.");
    }
    else
    {
        ESP_LOGE(TAG, "ws2812_strip is NULL after initialize_led_strip in init()!");
        // This case should ideally not happen if initialize_led_strip returns ESP_OK
        return ESP_FAIL;
    }

    // Comment out init success log to reduce UART traffic
    // ESP_LOGI(TAG, "LedIndication initialized.");
    return ESP_OK;
}

esp_err_t LedIndication::initialize_led_strip()
{
    ESP_LOGI(TAG, "initialize_led_strip() START - Using custom WS2812 driver.");

    if (ws2812_strip != nullptr)
    {
        ESP_LOGW(TAG, "WS2812 strip already initialized. Deleting existing instance.");
        delete ws2812_strip;
        ws2812_strip = nullptr;
    }

    ws2812_strip = new WS2812(NEOPIXEL_DATA_OUTPUT_PIN, NUM_LEDS);
    if (ws2812_strip == nullptr)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for WS2812 object!");
        return ESP_ERR_NO_MEM;
    }

    // Your WS2812 driver might require setting a color order if it doesn't default to GRB for WS2812s
    // Example: ws2812_strip->setColorOrder("GRB");
    // Check WS2812.cpp or its documentation. Assuming GRB is default or handled internally for now.

    ESP_LOGI(TAG, "Created WS2812 strip object. GPIO: %d, LEDs: %d", (int)NEOPIXEL_DATA_OUTPUT_PIN, (int)NUM_LEDS);
    return ESP_OK;
}

void LedIndication::set_pixel_color_internal(LedIndex led_idx, RgbColor color)
{
    uint16_t index = static_cast<uint16_t>(led_idx);
    if (ws2812_strip != nullptr && index < NUM_LEDS)
    {
        // Apply global brightness scaling
        uint8_t r = (uint8_t)(color.r * (LED_BRIGHTNESS_PERCENT / 100.0f));
        uint8_t g = (uint8_t)(color.g * (LED_BRIGHTNESS_PERCENT / 100.0f));
        uint8_t b = (uint8_t)(color.b * (LED_BRIGHTNESS_PERCENT / 100.0f));

        ws2812_strip->setPixel(index, r, g, b);
    }
    else
    {
        if (ws2812_strip == nullptr)
        {
            ESP_LOGE(TAG, "set_pixel_color_internal: ws2812_strip is NULL!");
        }
        // Optionally log if index is out of bounds, though NUM_LEDS should be correct from PeripheralManager
    }
}

void LedIndication::refresh_led_strip()
{
    if (ws2812_strip != nullptr)
    {
        ws2812_strip->show();
    }
    else
    {
        ESP_LOGE(TAG, "refresh_led_strip: ws2812_strip is NULL!");
    }
}

// --- State Update Methods (called from other tasks/modules) ---
void LedIndication::setLed3OverallMode(Led3AppMode mode)
{
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE)
    {
        current_led3_app_mode = mode;
        xSemaphoreGive(data_mutex);
    }
}

void LedIndication::updateWifiStaStatus(WifiStaAppStatus status)
{
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE)
    {
        current_wifi_sta_status = status;
        xSemaphoreGive(data_mutex);
    }
}

void LedIndication::updateHttpServerStatus(HttpServerAppStatus status)
{
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE)
    {
        current_http_server_status = status;
        xSemaphoreGive(data_mutex);
    }
}

void LedIndication::updateMqttStatus(MqttAppStatus status)
{
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE)
    {
        current_mqtt_status = status;
        xSemaphoreGive(data_mutex);
    }
}

void LedIndication::updatePumpStatus(PumpAppStatus status)
{
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE)
    {
        current_pump_status = status;
        xSemaphoreGive(data_mutex);
    }
}

void LedIndication::updateDataReceptionStatus(DataReceptionAppStatus status)
{
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE)
    {
        current_data_reception_status = status;
        xSemaphoreGive(data_mutex);
    }
}

void LedIndication::triggerHttpEventIndication()
{
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE)
    {
        // Set the counter to 8 for double blink pattern (4 ticks per blink cycle)
        // Each tick is 100ms, so this gives: ON(200ms), OFF(100ms), ON(200ms), OFF(100ms) = 600ms total
        http_event_blink_count = 8;
        xSemaphoreGive(data_mutex);
    }
}

void LedIndication::clearIndications()
{
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGI(TAG, "Clearing all LED indications to #000000 (off)");

        // Set all LEDs to black (off)
        set_pixel_color_internal(LedIndex::LED0, LedColors::BLACK);
        set_pixel_color_internal(LedIndex::LED1, LedColors::BLACK);
        set_pixel_color_internal(LedIndex::LED2, LedColors::BLACK);
        set_pixel_color_internal(LedIndex::LED3, LedColors::BLACK);

        refresh_led_strip();

        xSemaphoreGive(data_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take mutex for clearIndications()");
    }
}

// --- LED Task ---
void LedIndication::startLedTask()
{
    // Comment out task creation logs to reduce UART traffic
    // ESP_LOGI(TAG, "startLedTask() called.");
    BaseType_t task_created = xTaskCreatePinnedToCore(
        led_task_entry,
        "led_task",
        LED_TASK_STACK_SIZE,
        this,
        LED_TASK_PRIORITY,
        &led_task_handle,
        APP_CPU_NUM);

    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create led_task. Error code: %d", (int)task_created);
    }
    // Comment out success log to reduce UART traffic
    // else { ESP_LOGI(TAG, "led_task created successfully."); }
}

void LedIndication::led_task_entry(void *arg)
{
    // Comment out task entry logs to reduce UART traffic
    // ESP_LOGI("LedIndicationTaskEntry", "led_task_entry called. arg: %p", arg);
    if (!arg)
    {
        ESP_LOGE("LedIndicationTaskEntry", "Argument is NULL! Deleting task.");
        vTaskDelete(NULL);
        return;
    }
    LedIndication *self = static_cast<LedIndication *>(arg);
    // ESP_LOGI("LedIndicationTaskEntry", "'self' pointer: %p. Calling led_task_loop.", (void *)self);
    self->led_task_loop();
}

void LedIndication::led_task_loop()
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_frequency = pdMS_TO_TICKS(100);

    // Comment out task loop start log to reduce UART traffic
    // ESP_LOGI(TAG, "led_task_loop started.");

    if (ws2812_strip == nullptr)
    {
        ESP_LOGE(TAG, "ws2812_strip is NULL at start of led_task_loop! Deleting task.");
        vTaskDelete(NULL);
        return;
    }

    const float BRIGHTNESS_BLINK = 0.75f;
    const float BRIGHTNESS_CONTINUOUS = 0.40f;
    const float BRIGHTNESS_FADE_MAX = 0.75f;
    const float FADE_MIN_BRIGHTNESS_FACTOR = 0.15f; // Min brightness for pulsing LEDs (15% of base color)

    auto scale_color = [](RgbColor color, float factor) -> RgbColor
    {
        return {
            static_cast<uint8_t>(color.r * factor),
            static_cast<uint8_t>(color.g * factor),
            static_cast<uint8_t>(color.b * factor)};
    };

    auto blend_colors = [](RgbColor color1, RgbColor color2, float factor) -> RgbColor
    {
        factor = (factor < 0.0f) ? 0.0f : (factor > 1.0f) ? 1.0f
                                                          : factor;
        return {
            static_cast<uint8_t>(color1.r * (1.0f - factor) + color2.r * factor),
            static_cast<uint8_t>(color1.g * (1.0f - factor) + color2.g * factor),
            static_cast<uint8_t>(color1.b * (1.0f - factor) + color2.b * factor)};
    };

    while (true)
    {
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            blink_counter++;

            // --- Fade Animation Logic ---
            const uint32_t FADE_PERIOD_TICKS = 20; // Number of blink_counter ticks for one full fade cycle (up and down)
            float normalized_progress = static_cast<float>(blink_counter % FADE_PERIOD_TICKS) / static_cast<float>(FADE_PERIOD_TICKS - 1);
            float triangle_progress; // Will be 0.0 -> 1.0 -> 0.0

            if (normalized_progress < 0.5f)
            {
                triangle_progress = normalized_progress * 2.0f;
            }
            else
            {
                triangle_progress = (1.0f - normalized_progress) * 2.0f;
            }
            // current_fade_brightness_factor will range from FADE_MIN_BRIGHTNESS_FACTOR to BRIGHTNESS_FADE_MAX
            float current_fade_brightness_factor = FADE_MIN_BRIGHTNESS_FACTOR + (BRIGHTNESS_FADE_MAX - FADE_MIN_BRIGHTNESS_FACTOR) * triangle_progress;

            // --- HTTP Event Blink Override ---
            if (http_event_blink_count > 0)
            {
                // Double blink pattern: ON(200ms), OFF(100ms), ON(200ms), OFF(100ms) = 600ms total
                // Counter values: 8-7=ON, 6=OFF, 5-4=ON, 3=OFF, 2-1=end
                bool is_on = (http_event_blink_count >= 7) || (http_event_blink_count >= 4 && http_event_blink_count <= 5);

                // Use AQUAMARINE (greenish-cyan) color for the blinks
                RgbColor blink_color = scale_color(LedColors::AQUAMARINE, BRIGHTNESS_BLINK);
                set_pixel_color_internal(LedIndex::LED3, is_on ? blink_color : LedColors::BLACK);
                http_event_blink_count--;
            }
            else
            {
                // Determine color for LED3 (WiFi STA or AP/HTTP)
                if (current_led3_app_mode == Led3AppMode::WIFI_STATION_MODE)
                {
                    switch (current_wifi_sta_status)
                    {
                    case WifiStaAppStatus::WIFI_STA_INITIALIZING:
                        set_pixel_color_internal(LedIndex::LED3, LedColors::BLACK);
                        break;
                    case WifiStaAppStatus::WIFI_STA_DISCONNECTED:
                        set_pixel_color_internal(LedIndex::LED3, (blink_counter % 10 < 5) ? scale_color(LedColors::RED, BRIGHTNESS_BLINK) : LedColors::BLACK);
                        break;
                    case WifiStaAppStatus::WIFI_STA_CONNECTING:
                    {
                        RgbColor base_color = LedColors::CONFIG_VIOLET;
                        RgbColor faded_color = scale_color(base_color, current_fade_brightness_factor);
                        set_pixel_color_internal(LedIndex::LED3, faded_color);
                    }
                    break;
                    case WifiStaAppStatus::WIFI_STA_CONNECTED:
                        set_pixel_color_internal(LedIndex::LED3, scale_color(LedColors::GREEN, BRIGHTNESS_CONTINUOUS));
                        break;
                    case WifiStaAppStatus::DEVICE_STATE_NO_WIFI_CREDS:
                        set_pixel_color_internal(LedIndex::LED3, scale_color(LedColors::YELLOW, BRIGHTNESS_CONTINUOUS));
                        break;
                    }
                }
                else
                { // WIFI_AP_HTTP_SERVER_MODE
                    switch (current_http_server_status)
                    {
                    case HttpServerAppStatus::HTTP_SERVER_INACTIVE:
                        set_pixel_color_internal(LedIndex::LED3, LedColors::BLACK);
                        break;
                    case HttpServerAppStatus::HTTP_SERVER_INITIATING:
                    {
                        // Config Mode Entry & WiFi APSTA Initialization: LED3 should blink fade with purple-bluish color
                        RgbColor base_color = LedColors::CONFIG_INDIGO; // Deep indigo for config mode
                        RgbColor faded_color = scale_color(base_color, current_fade_brightness_factor);
                        set_pixel_color_internal(LedIndex::LED3, faded_color);
                    }
                    break;
                    case HttpServerAppStatus::HTTP_AP_ACTIVE_READY_FOR_CONN:
                        // HTTPS Server Initialization: LED3 should show solid purple-bluish color (no blink fade)
                        set_pixel_color_internal(LedIndex::LED3, LedColors::CONFIG_DEEP_PURPLE);
                        break;
                    case HttpServerAppStatus::HTTP_AP_ACTIVE_NO_CLIENT:
                        // Solid purple-bluish, waiting for client connection
                        set_pixel_color_internal(LedIndex::LED3, LedColors::CONFIG_DEEP_PURPLE);
                        break;
                    case HttpServerAppStatus::HTTP_AP_ACTIVE_CLIENT_CONNECTED:
                        // Solid cyan when a client is connected (different from config mode colors)
                        set_pixel_color_internal(LedIndex::LED3, LedColors::CYAN);
                        break;
                    case HttpServerAppStatus::HTTPS_SERVER_REMOTE_STATION_CONNECTING:
                        // Transition state: fade between purple-bluish and greenish-cyan
                        {
                            float transition_factor = (blink_counter % 20) / 20.0f;
                            RgbColor transition_color = blend_colors(LedColors::CONFIG_DEEP_PURPLE, LedColors::AQUAMARINE, transition_factor);
                            set_pixel_color_internal(LedIndex::LED3, scale_color(transition_color, BRIGHTNESS_CONTINUOUS));
                        }
                        break;
                    case HttpServerAppStatus::HTTPS_SERVER_REMOTE_STATION_CONNECTED:
                        // Solid light greenish-cyan when remote station is connected
                        set_pixel_color_internal(LedIndex::LED3, scale_color(LedColors::AQUAMARINE, BRIGHTNESS_CONTINUOUS));
                        break;
                    case HttpServerAppStatus::HTTPS_SERVER_REMOTE_STATION_DISCONNECTING:
                        // Transition state: fade from greenish-cyan back to purple-bluish
                        {
                            float transition_factor = (blink_counter % 20) / 20.0f;
                            RgbColor transition_color = blend_colors(LedColors::AQUAMARINE, LedColors::CONFIG_DEEP_PURPLE, transition_factor);
                            set_pixel_color_internal(LedIndex::LED3, scale_color(transition_color, BRIGHTNESS_CONTINUOUS));
                        }
                        break;
                    }
                }
            }

            // Determine color for LED2 (MQTT)
            if (current_led3_app_mode == Led3AppMode::WIFI_AP_HTTP_SERVER_MODE)
            {
                set_pixel_color_internal(LedIndex::LED2, LedColors::BLACK); // MQTT not relevant in config mode
            }
            else if (current_mqtt_status == MqttAppStatus::MQTT_WIFI_DISCONNECTED)
            {
                set_pixel_color_internal(LedIndex::LED2, LedColors::BLACK); // Off if WiFi is down
            }
            else
            {
                switch (current_mqtt_status)
                {
                case MqttAppStatus::MQTT_INITIALIZING:
                    set_pixel_color_internal(LedIndex::LED2, LedColors::BLACK);
                    break;
                case MqttAppStatus::MQTT_DISCONNECTED:
                    set_pixel_color_internal(LedIndex::LED2, (blink_counter % 10 < 5) ? scale_color(LedColors::RED, BRIGHTNESS_BLINK) : LedColors::BLACK);
                    break;
                case MqttAppStatus::MQTT_CONNECTING:
                {
                    RgbColor base_color = LedColors::GREEN;
                    RgbColor faded_color = {
                        static_cast<uint8_t>(base_color.r * current_fade_brightness_factor),
                        static_cast<uint8_t>(base_color.g * current_fade_brightness_factor),
                        static_cast<uint8_t>(base_color.b * current_fade_brightness_factor)};
                    set_pixel_color_internal(LedIndex::LED2, faded_color);
                }
                break;
                case MqttAppStatus::MQTT_CONNECTED:
                    set_pixel_color_internal(LedIndex::LED2, scale_color(LedColors::GREEN, BRIGHTNESS_CONTINUOUS));
                    break;
                default:
                    set_pixel_color_internal(LedIndex::LED2, LedColors::BLACK);
                    break;
                }
            }

            // Determine color for LED1 (Manual Pump Restart)
            switch (current_pump_status)
            {
            case PumpAppStatus::PUMP_AUTO_MODE_OK:
                set_pixel_color_internal(LedIndex::LED1, LedColors::BLACK);
                break;
            case PumpAppStatus::PUMP_MANUAL_RESTART_NEEDED:
                set_pixel_color_internal(LedIndex::LED1, scale_color(LedColors::YELLOW, BRIGHTNESS_CONTINUOUS));
                break;
            case PumpAppStatus::PUMP_RECALIBRATION_MODE:
                set_pixel_color_internal(LedIndex::LED1, scale_color(LedColors::BLUE, BRIGHTNESS_CONTINUOUS));
                break;
            }

            // Determine color for LED0 (Data Reception Timeout)
            switch (current_data_reception_status)
            {
            case DataReceptionAppStatus::DATA_RECEPTION_OK:
                set_pixel_color_internal(LedIndex::LED0, LedColors::BLACK);
                break;
            case DataReceptionAppStatus::DATA_RECEPTION_TIMEOUT:
                set_pixel_color_internal(LedIndex::LED0, (blink_counter % 10 < 5) ? scale_color(LedColors::RED, BRIGHTNESS_BLINK) : LedColors::BLACK);
                break;
            }

            refresh_led_strip();

            xSemaphoreGive(data_mutex);
        }
        else
        {
            ESP_LOGW(TAG, "led_task_loop: Failed to take mutex within 50ms!");
        }
        vTaskDelayUntil(&last_wake_time, task_frequency);
    }
}