/**
  ******************************************************************************
  * @file    systick.h
  * @author  Alexander Smirnov
  * @copyright Copyright FDC (C) 2024-2025
  ******************************************************************************
*/

#pragma once

#include <stdint.h>

void systick_init(void);

int64_t systick_get_tick_sec(void);

int64_t systick_get_tick_ms(void);

int64_t systick_get_tick_us(void);

void systick_delay_sec(int32_t sec);

void systick_delay_ms(int32_t ms);

void systick_delay_us(int32_t us);

void systick_interrupt_handler(void);

void systick_deinit(void);
