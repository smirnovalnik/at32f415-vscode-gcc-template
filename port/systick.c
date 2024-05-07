/**
  ******************************************************************************
  * @file    systick.c
  * @author  Alexander Smirnov
  * @version 1.0
  * @copyright Copyright FDC (C) 2024
  ******************************************************************************
*/

#include "systick.h"
#include "at32f415.h"

static volatile uint64_t systick_us;

void systick_init(void)
{
    systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);

    systick_us = 0;

    SysTick->LOAD = (uint32_t)(system_core_clock / 1000000U - 1UL);
    SysTick->VAL = 0UL;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk |
                     SysTick_CTRL_ENABLE_Msk;

    nvic_irq_enable(SysTick_IRQn, 15, 0);
}

uint32_t systick_get_tick_sec(void)
{
    return systick_us / 1000000;
}

uint32_t systick_get_tick_ms(void)
{
    return systick_us / 1000;
}

uint64_t systick_get_tick_us(void)
{
    return systick_us;
}

void systick_delay_sec(uint32_t sec)
{
    uint32_t start = 0;

    start = systick_get_tick_sec();
    while ((systick_get_tick_sec() - start) < sec);
}

void systick_delay_ms(uint32_t ms)
{
    uint32_t start = 0;

    start = systick_get_tick_ms();
    while ((systick_get_tick_ms() - start) < ms);
}

void systick_delay_us(uint64_t us)
{
    uint64_t start = 0;

    start = systick_get_tick_us();
    while ((systick_get_tick_us() - start) < us);
}

void systick_interrupt(void)
{
    systick_us++;
}
