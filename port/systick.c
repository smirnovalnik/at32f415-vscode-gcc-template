/**
  ******************************************************************************
  * @file    systick.c
  * @author  Alexander Smirnov
  * @copyright Copyright FDC (C) 2024
  ******************************************************************************
*/

#include "systick.h"
#include "at32f415.h"

static volatile uint64_t systick_ms;

void systick_init(void)
{
    systick_ms = 0;

    SysTick->LOAD = (uint32_t)((system_core_clock / 1000UL) - 1UL);
    SysTick->VAL = 0UL;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk |
                     SysTick_CTRL_ENABLE_Msk;
}

uint32_t systick_get_tick_sec(void)
{
    return systick_ms / 1000;
}

uint32_t systick_get_tick_ms(void)
{
    return systick_ms;
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

void systick_interrupt_handler(void)
{
    systick_ms++;
}
