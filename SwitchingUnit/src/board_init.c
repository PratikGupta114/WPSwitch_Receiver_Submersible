#include "board_init.h"
#include "board_pins.h"

void board_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    NVIC_InitTypeDef nvic = {0};

    /* Enable GPIO clocks for Port C, Port D, and AFIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_AFIO,
                           ENABLE);

    /* -----------------------------------------------------------------
     * GPIOC: Relay outputs (PC0, PC1, PC2) — Push-Pull, 50 MHz
     * ---------------------------------------------------------------*/
    gpio.GPIO_Pin   = RELAY_LARGE_NO_PIN | RELAY_SMALL_NO_PIN | RELAY_NC_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    /* Drive all relays to safe OFF state immediately */
    GPIO_WriteBit(RELAY_LARGE_NO_PORT, RELAY_LARGE_NO_PIN, Bit_RESET);
    GPIO_WriteBit(RELAY_SMALL_NO_PORT, RELAY_SMALL_NO_PIN, Bit_RESET);
    GPIO_WriteBit(RELAY_NC_PORT,       RELAY_NC_PIN,       Bit_RESET);

    /* -----------------------------------------------------------------
     * GPIOC: Presence pin (PC3) — Open-Drain, released HIGH + EXTI interrupt
     * ---------------------------------------------------------------*/
    gpio.GPIO_Pin   = PRESENCE_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);
    GPIO_WriteBit(PRESENCE_PORT, PRESENCE_PIN, Bit_SET); /* Release (HIGH) */

    /* Route PC3 to EXTI Line 3 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);

    EXTI_InitTypeDef exti = {0};
    exti.EXTI_Line = EXTI_Line3;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel = EXTI7_0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* -----------------------------------------------------------------
     * GPIOC: HLW8032 RX (PC6) — Input with Pull-Up
     * ---------------------------------------------------------------*/
    gpio.GPIO_Pin  = HLW8032_RX_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &gpio);

    /* -----------------------------------------------------------------
     * GPIOC: HLW8032 Pulse / CF (PC7) — Input with Pull-Up
     * ---------------------------------------------------------------*/
    gpio.GPIO_Pin  = HLW8032_PF_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &gpio);

    /* -----------------------------------------------------------------
     * GPIOD: Motor detection (PD4) — Input with Pull-Up
     * ---------------------------------------------------------------*/
    gpio.GPIO_Pin  = MOTOR_DETECT_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &gpio);

    /* PD5 (TX) and PD6 (RX) are configured by hw_uart_init(). */
    /* PD1 (SWIO) is reserved for the WCH-LinkE programmer.     */
}
