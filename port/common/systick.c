/**
 ******************************************************************************
 * @file    systick.c
 * @author  Alexander Smirnov
 * @copyright Copyright FDC (C) 2024-2025
 ******************************************************************************
 */

#include "systick.h"
#include "hal.h"

static volatile int64_t systick_ms = 0;

void systick_init(void)
{
    SysTick->LOAD = (uint32_t)((system_core_clock / 1000UL) - 1UL);
    SysTick->VAL = 0UL;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

int64_t systick_get_tick_sec(void)
{
    return systick_get_tick_ms() / 1000;
}

int64_t systick_get_tick_ms(void)
{
    int64_t ms;

    do
    {
        ms = systick_ms;
    } while (ms != systick_ms);

    return ms;
}

int64_t systick_get_tick_us(void)
{
    uint32_t load = SysTick->LOAD;
    uint32_t val;
    int64_t ms;

    do
    {
        ms = systick_get_tick_ms();
        val = SysTick->VAL;

        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) || (val > load))
        {
            continue;
        }

        if (ms != systick_get_tick_ms())
        {
            continue;
        }

        break;

    } while (1);

    uint32_t ticks = load - val;
    uint64_t us_offset = (uint64_t)ticks * 1000000UL / system_core_clock;

    return (ms * 1000) + us_offset;
}

void systick_delay_sec(int32_t sec)
{
    int64_t start = 0;

    start = systick_get_tick_sec();
    while ((systick_get_tick_sec() - start) < sec);
}

void systick_delay_ms(int32_t ms)
{
    int64_t start = 0;

    start = systick_get_tick_ms();
    while ((systick_get_tick_ms() - start) < ms);
}

void systick_delay_us(int32_t us)
{
    int64_t start = 0;

    start = systick_get_tick_us();
    while ((systick_get_tick_us() - start) < us);
}

void systick_interrupt_handler(void)
{
    systick_ms++;
}

void systick_deinit(void)
{
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
}
