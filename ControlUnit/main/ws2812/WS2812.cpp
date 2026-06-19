#include <esp_log.h>
#include <driver/gpio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdexcept>
#include "sdkconfig.h"
#include "WS2812.h"

static const char *TAG = "WS2812";

WS2812::WS2812(gpio_num_t dinPin, uint16_t pixelCount, int channel)
{
    this->pixelCount = pixelCount;
    this->colorOrder = (char *)"GRB";
    this->led_strip = NULL;

    // Configure the LED strip
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = dinPin;
    strip_config.max_leds = pixelCount;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;

    // Configure the RMT backend
    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz
    rmt_config.flags.with_dma = false;

    // Initialize the device
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &this->led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create led_strip RMT device: %s", esp_err_to_name(err));
    }
}

WS2812::~WS2812()
{
    if (this->led_strip) {
        led_strip_del(this->led_strip);
        this->led_strip = NULL;
    }
}

void WS2812::show()
{
    if (this->led_strip) {
        esp_err_t err = led_strip_refresh(this->led_strip);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to refresh led_strip: %s", esp_err_to_name(err));
        }
    }
}

void WS2812::setColorOrder(char *colorOrder)
{
    if (colorOrder != nullptr && strlen(colorOrder) == 3)
    {
        this->colorOrder = colorOrder;
    }
}

void WS2812::setPixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    assert(index < pixelCount);
    if (this->led_strip) {
        esp_err_t err = led_strip_set_pixel(this->led_strip, index, red, green, blue);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set pixel %d: %s", index, esp_err_to_name(err));
        }
    }
}

void WS2812::setPixel(uint16_t index, pixel_t pixel)
{
    setPixel(index, pixel.red, pixel.green, pixel.blue);
}

void WS2812::setPixel(uint16_t index, uint32_t pixel)
{
    // Extract red, green, blue from uint32_t (R, G, B order is unpacked here)
    uint8_t red = pixel & 0xff;
    uint8_t green = (pixel & 0xff00) >> 8;
    uint8_t blue = (pixel & 0xff0000) >> 16;
    setPixel(index, red, green, blue);
}

void WS2812::setHSBPixel(uint16_t index, uint16_t hue, uint8_t saturation, uint8_t brightnes)
{
    double sat_red;
    double sat_green;
    double sat_blue;
    double ctmp_red;
    double ctmp_green;
    double ctmp_blue;
    double new_red;
    double new_green;
    double new_blue;
    double dSaturation = (double)saturation / 255;
    double dBrightnes = (double)brightnes / 255;

    assert(index < pixelCount);

    if (hue < 120)
    {
        sat_red = (120 - hue) / 60.0;
        sat_green = hue / 60.0;
        sat_blue = 0;
    }
    else if (hue < 240)
    {
        sat_red = 0;
        sat_green = (240 - hue) / 60.0;
        sat_blue = (hue - 120) / 60.0;
    }
    else
    {
        sat_red = (hue - 240) / 60.0;
        sat_green = 0;
        sat_blue = (360 - hue) / 60.0;
    }

    if (sat_red > 1.0)
    {
        sat_red = 1.0;
    }
    if (sat_green > 1.0)
    {
        sat_green = 1.0;
    }
    if (sat_blue > 1.0)
    {
        sat_blue = 1.0;
    }

    ctmp_red = 2 * dSaturation * sat_red + (1 - dSaturation);
    ctmp_green = 2 * dSaturation * sat_green + (1 - dSaturation);
    ctmp_blue = 2 * dSaturation * sat_blue + (1 - dSaturation);

    if (dBrightnes < 0.5)
    {
        new_red = dBrightnes * ctmp_red;
        new_green = dBrightnes * ctmp_green;
        new_blue = dBrightnes * ctmp_blue;
    }
    else
    {
        new_red = (1 - dBrightnes) * ctmp_red + 2 * dBrightnes - 1;
        new_green = (1 - dBrightnes) * ctmp_green + 2 * dBrightnes - 1;
        new_blue = (1 - dBrightnes) * ctmp_blue + 2 * dBrightnes - 1;
    }

    setPixel(index, (uint8_t)(new_red * 255), (uint8_t)(new_green * 255), (uint8_t)(new_blue * 255));
}

void WS2812::clear()
{
    if (this->led_strip) {
        esp_err_t err = led_strip_clear(this->led_strip);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear led_strip: %s", esp_err_to_name(err));
        }
    }
}