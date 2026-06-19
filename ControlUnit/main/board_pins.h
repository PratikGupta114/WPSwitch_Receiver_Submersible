#ifndef BOARD_PINS_H_
#define BOARD_PINS_H_

#include "driver/gpio.h"

// ============================================================================
// GPIO Pin Configuration (Strictly matching the Submersible Control Unit board)
// ============================================================================

// 1. Switching Unit (CH32V003) Interface
#define CONTROL_UNIT_UART_PORT      UART_NUM_1
#define CONTROL_UNIT_UART_TX_PIN    GPIO_NUM_26  // Pin 11, IO26
#define CONTROL_UNIT_UART_RX_PIN    GPIO_NUM_25  // Pin 10, IO25
#define CONTROL_UNIT_PRESENCE_PIN   GPIO_NUM_35  // Pin 7, IO35 (Input-Only)

// 2. RF Wireless Module (WIR-1186) Interface
#define WIRELESS_UART_PORT          UART_NUM_2
#define WIRELESS_DATA_TX_PIN        GPIO_NUM_4   // Pin 26, IO4
#define WIRELESS_DATA_RX_PIN        GPIO_NUM_16  // Pin 27, IO16
#define WIRELESS_PRG_PIN            GPIO_NUM_17  // Pin 28, IO17

// 3. I2C Bus (RTC DS1307 & Display)
#define I2C_SDA_PIN                 GPIO_NUM_32  // Pin 8, IO32
#define I2C_SCL_PIN                 GPIO_NUM_27  // Pin 12, IO27

// 4. Status Indication (WS2812B NeoPixel)
#define NEOPIXEL_DATA_PIN           GPIO_NUM_23  // Pin 37, IO23

// 5. Buzzer Output (PWM/GPIO)
#define BUZZER_OUTPUT_PIN           GPIO_NUM_33  // Pin 9, IO33

// 6. User Inputs
#define CONFIG_BUTTON_INPUT_PIN     GPIO_NUM_34  // Pin 6, IO34 (Input-Only)
#define WS_SWITCH_BUTTON_INPUT_PIN  GPIO_NUM_21  // Pin 33, IO21
#define BUTTON_LED_OUTPUT_PIN       GPIO_NUM_22  // Pin 36, IO22

// 7. 7-Segment Shift Register Interface (74HC595)
#define CLOCK_PIN_SHCP              GPIO_NUM_5   // Pin 29, IO5
#define DATA_PIN_DS                 GPIO_NUM_18  // Pin 30, IO18
#define LATCH_PIN_STCP              GPIO_NUM_19  // Pin 31, IO19

#endif // BOARD_PINS_H_
