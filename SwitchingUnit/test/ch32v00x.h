#ifndef MOCK_CH32V00X_H
#define MOCK_CH32V00X_H

#include <stdint.h>

/* Mock GPIO registers and types */
typedef void* GPIO_TypeDef;
#define GPIOC ((GPIO_TypeDef)0x1)
#define GPIOD ((GPIO_TypeDef)0x2)

#define GPIO_Pin_0 0x0001
#define GPIO_Pin_1 0x0002
#define GPIO_Pin_2 0x0004
#define GPIO_Pin_3 0x0008
#define GPIO_Pin_4 0x0010
#define GPIO_Pin_5 0x0020
#define GPIO_Pin_6 0x0040
#define GPIO_Pin_7 0x0080

typedef enum {
    Bit_RESET = 0,
    Bit_SET
} BitAction;

/* Dummy function stubs to satisfy board_pins.h inline helpers */
static inline void GPIO_WriteBit(GPIO_TypeDef port, uint16_t pin, BitAction action) {
    (void)port; (void)pin; (void)action;
}

static inline uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef port, uint16_t pin) {
    (void)port; (void)pin;
    return 1;
}

#endif
