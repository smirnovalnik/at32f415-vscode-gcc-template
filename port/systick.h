/**
  ******************************************************************************
  * @file    systick.h
  * @author  Alexander Smirnov
  * @version 1.0
  * @copyright Copyright FDC (C) 2024
  ******************************************************************************
*/

#pragma once

#include <stdint.h>

void systick_init(void);

uint32_t systick_get_tick_sec(void);

uint32_t systick_get_tick_ms(void);

uint64_t systick_get_tick_us(void);

void systick_delay_sec(uint32_t sec);

void systick_delay_ms(uint32_t ms);

void systick_delay_us(uint64_t us);

void systick_interrupt(void);
