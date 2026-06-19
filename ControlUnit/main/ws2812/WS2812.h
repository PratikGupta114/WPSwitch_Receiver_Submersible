#ifndef MAIN_WS2812_H_
#define MAIN_WS2812_H_

#include <stdint.h>
#include <driver/gpio.h>
#include "led_strip.h"

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} pixel_t;

class WS2812
{
public:
    WS2812(gpio_num_t gpioNum, uint16_t pixelCount, int channel = 0);
    void show();
    void setColorOrder(char *order);
    void setPixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
    void setPixel(uint16_t index, pixel_t pixel);
    void setPixel(uint16_t index, uint32_t pixel);
    void setHSBPixel(uint16_t index, uint16_t hue, uint8_t saturation, uint8_t brightnes);
    void clear();
    virtual ~WS2812();

private:
    char *colorOrder;
    uint16_t pixelCount;
    led_strip_handle_t led_strip;
};

#endif
