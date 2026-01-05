/**
  ******************************************************************************
  * @file    cycle_timer.h
  * @author  Alexander Smirnov
  * @copyright Copyright FDC (C) 2025
  ******************************************************************************
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>

int cycle_timer_init(uint32_t cpu_hz);

static inline uint32_t cycle_timer_now(void)
{
    return *((volatile uint32_t*)0xE0001004u); // DWT->CYCCNT
}

static inline uint32_t cycle_timer_elapsed(uint32_t start, uint32_t end)
{
    return (uint32_t)(end - start);
}

uint32_t cycle_timer_cpu_hz(void);

uint64_t cycle_timer_cycles_to_ns(uint32_t cycles);

uint32_t cycle_timer_cycles_to_us(uint32_t cycles);

void cycle_timer_delay_cycles(uint32_t cycles);
