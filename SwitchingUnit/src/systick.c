#include "systick.h"
#include "ch32v00x.h"

/* ---------------------------------------------------------------------------
 * Millisecond counter (incremented by TIM2 ISR)
 *
 * NOTE: This counter may drift by up to ~50 ms when the software UART
 * receives an HLW8032 frame (interrupts disabled during bit-bang RX).
 * For relay timing on 3 s / 170 s scales this is acceptable.
 * -------------------------------------------------------------------------*/
static volatile uint32_t ms_counter = 0;

void systick_init(void)
{
    /* =================================================================
     * SysTick: Free-running up-counter at HCLK (48 MHz)
     * Used by sw_uart.c for precise bit-timing.
     * Do NOT call Delay_Init() / Delay_Ms() after this.
     * ===============================================================*/
    SysTick->CTLR = 0;             /* Disable                          */
    SysTick->SR   = 0;             /* Clear status flags                */
    SysTick->CNT  = 0;             /* Reset counter                     */
    SysTick->CMP  = 0xFFFFFFFF;    /* Max compare (no auto-reload)      */
    /* STCLK = HCLK (bit 2), STE = enable (bit 0) */
    SysTick->CTLR = (1u << 2) | (1u << 0);

    /* =================================================================
     * TIM2: 1 ms update interrupt for millis() counter
     * 48 MHz / (47+1) = 1 MHz tick, period = 999+1 = 1000 µs = 1 ms
     * ===============================================================*/
    TIM_TimeBaseInitTypeDef tim  = {0};
    NVIC_InitTypeDef        nvic = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    tim.TIM_Period        = 999;
    tim.TIM_Prescaler     = 47;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel                   = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1; /* Lower than USART1 */
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM2, ENABLE);
}

uint32_t millis(void)
{
    return ms_counter;
}

uint8_t elapsed(uint32_t *last, uint32_t period_ms)
{
    uint32_t now = ms_counter;
    if ((now - *last) >= period_ms) {
        *last = now;
        return 1;
    }
    return 0;
}

void delay_ms_blocking(uint32_t ms)
{
    uint32_t start = ms_counter;
    while ((ms_counter - start) < ms) {
        /* spin — interrupts must be enabled */
    }
}

/* ---------------------------------------------------------------------------
 * TIM2 Update Interrupt Handler
 * -------------------------------------------------------------------------*/
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        ms_counter++;
    }
}
