#include "hw_uart.h"
#include "board_pins.h"
#include "ch32v00x.h"

/* ---------------------------------------------------------------------------
 * RX Ring Buffer (interrupt-filled, main-loop drained)
 * -------------------------------------------------------------------------*/
static volatile uint8_t  rx_buf[HW_UART_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;  /* Written by ISR  */
static volatile uint16_t rx_tail = 0;  /* Read by main    */

void hw_uart_init(uint32_t baud)
{
    GPIO_InitTypeDef  gpio  = {0};
    USART_InitTypeDef usart = {0};
    NVIC_InitTypeDef  nvic  = {0};

    /* Enable peripheral clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1,
                           ENABLE);

    /* PD5 = USART1 TX — Alternate Function Push-Pull */
    gpio.GPIO_Pin   = UART_TX_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(UART_TX_PORT, &gpio);

    /* PD6 = USART1 RX — Floating Input */
    gpio.GPIO_Pin  = UART_RX_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(UART_RX_PORT, &gpio);

    /* USART1 configuration */
    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);

    /* Enable RXNE interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0; /* Higher than TIM2 */
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART1, ENABLE);
}

void hw_uart_send_byte(uint8_t b)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
        /* wait */
    }
    USART_SendData(USART1, b);
}

void hw_uart_send(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        hw_uart_send_byte(data[i]);
    }
}

int16_t hw_uart_read_byte(void)
{
    if (rx_head == rx_tail) {
        return -1;  /* Buffer empty */
    }
    uint8_t b = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) & (HW_UART_RX_BUF_SIZE - 1);
    return (int16_t)b;
}

uint16_t hw_uart_rx_available(void)
{
    return (uint16_t)((rx_head - rx_tail) & (HW_UART_RX_BUF_SIZE - 1));
}

/* ---------------------------------------------------------------------------
 * USART1 RX Interrupt Handler
 *
 * Pushes received bytes into the ring buffer. On overflow, the oldest
 * unread data is preserved and the newest byte is dropped silently.
 * -------------------------------------------------------------------------*/
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t  data      = (uint8_t)USART_ReceiveData(USART1);
        uint16_t next_head = (rx_head + 1) & (HW_UART_RX_BUF_SIZE - 1);
        if (next_head != rx_tail) {
            rx_buf[rx_head] = data;
            rx_head = next_head;
        }
        /* Overflow: drop the incoming byte silently */
    }

    /* Check for error flags (Overrun, Noise, Framing, Parity) */
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET ||
        USART_GetFlagStatus(USART1, USART_FLAG_NE)  != RESET ||
        USART_GetFlagStatus(USART1, USART_FLAG_FE)  != RESET ||
        USART_GetFlagStatus(USART1, USART_FLAG_PE)  != RESET)
    {
        /* Clear error flags by reading STATR and then DATAR */
        volatile uint32_t temp_statr = USART1->STATR;
        volatile uint32_t temp_datar = USART1->DATAR;
        (void)temp_statr;
        (void)temp_datar;
    }
}
