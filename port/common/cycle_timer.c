/**
 ******************************************************************************
 * @file    cycle_timer.c
 * @author  Alexander Smirnov
 * @copyright Copyright FDC (C) 2025-2026
 ******************************************************************************
 */

#include "cycle_timer.h"
#include "hal.h"

static uint32_t _cpu_hz = 0;

int cycle_timer_init(void)
{
    _cpu_hz = system_core_clock;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0u)
    {
        return -1;
    }

    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __DSB();
    __ISB();

    return 0;
}

uint64_t cycle_timer_cycles_to_ns(uint32_t cycles)
{
    if (_cpu_hz == 0u)
        return 0u;
    // ns = cycles * 1e9 / Hz
    return (uint64_t)cycles * 1000000000ull / (uint64_t)_cpu_hz;
}

uint32_t cycle_timer_cycles_to_us(uint32_t cycles)
{
    if (_cpu_hz == 0u)
        return 0u;
    // us = cycles * 1e6 / Hz
    return (uint32_t)((uint64_t)cycles * 1000000ull / (uint64_t)_cpu_hz);
}

void cycle_timer_delay_cycles(uint32_t cycles)
{
    uint32_t start = cycle_timer_now();
    while (cycle_timer_elapsed(start, cycle_timer_now()) < cycles)
    {
        __NOP();
    }
}

int cycle_timer_deinit(void)
{
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0u;

    __DSB();
    __ISB();

    _cpu_hz = 0;

    return 0;
}
